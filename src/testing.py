"""GNSSログ(.txt)を読み込み、各点に 取得時刻・hAcc・heading・経路角・相対角 を埋め込んだ地図(HTML)を作る。

入力形式 (1点1行。先頭の [[ や末尾の ]] 、カンマ後の空白があってもOK):
    [18:07:25, 35.892298, 139.944564, 2.1, -12.0, 100.0, 88.0],
     時刻      緯度       経度        hAcc  相対角 heading 経路角
  hAcc ... 水平位置精度の推定値 [m]
  相対角 ... 経路角 - heading (-180〜180。右が正)
  角度はすべて度で、0°=北、時計回りに増加。
  (昔の [時刻, 緯度, 経度] だけの行も読める。その点は矢印なし・hAccなし(未評価)として扱う)

地図の見方:
  ・各点の矢印 = その点で進むべき方向 (既定: heading + 相対角 = 地図上の方位)
  ・ホバー = 時刻・hAcc・heading・経路角・相対角
  ・hAcc が閾値より大きい点は「除外」(灰色の×)。線にも含めない

使い方:
    python testing.py track.txt                       # → track_map.html
    python testing.py track.txt -o out.html --hacc-th 5 --gap 10 --max-speed 3
    python testing.py track.txt --ref route.txt --e-req 5
        # 基準の経路(緯度,経度の並び)までの距離を「誤差」として実測し、
        # 誤差 5 m 以内の点を「正しい点」とみなして、hAccで正しい点を見分けるのに最適な閾値(YoudenのJ最大)を求める
"""
import argparse
import csv
import os
import re
from collections import namedtuple
from datetime import datetime, timedelta
from math import atan2, cos, isnan, radians, sin, sqrt

import folium
import numpy as np
from branca.colormap import LinearColormap

# 1点 = [HH:MM:SS, 数値, 数値, ...]。 数値の個数が 2 (緯度,経度) または 6 (緯度,経度,hAcc,相対角,heading,経路角)
BRACKET = re.compile(r"\[\s*(\d{1,2}:\d{2}:\d{2})\s*((?:,[^,\[\]]*)+)\]")
PAIR = re.compile(r"(-?\d{1,3}\.\d+)\s*,\s*(-?\d{1,3}\.\d+)")
COORDS=[(35.891746, 139.944436), (35.891925, 139.944346), (35.891896, 139.944265), (35.891887, 139.94424), (35.891831, 139.944079), (35.892331, 139.943822), (35.892494, 139.943734), (35.892713, 139.943623), (35.892729, 139.943614), (35.892704, 139.943529), (35.892807, 139.943475), (35.8929, 139.943429), (35.892862, 139.943311), (35.892886, 139.943315), (35.893037, 139.943233), (35.892967, 139.942988), (35.892825, 139.94272), (35.892732, 139.942501), (35.892686, 139.942351), (35.892657, 139.942231), (35.892641, 139.942108), (35.892625, 139.941979), (35.89262, 139.941896), (35.892616, 139.941771), (35.892633, 139.941598), (35.892648, 139.941542), (35.892743, 139.941472)]

Point = namedtuple("Point", "t lat lon hacc rel heading route")
NAN = float("nan")
CARTO_API_KEY="cb1_44j7_1_420580e3a4f5727c0fe3bb6e"

DEFAULT_HACC_TH = 10.0       # 【暫定値。根拠なし】 --hacc-th も --ref/--e-req も無いときだけ使う
BIN_EDGES = [0, 1, 2, 3, 4, 5, 7, 10, 15, 25, float("inf")]     # hAccの区間 [m]
MIN_BAD = 5                 # 推奨閾値を求めるには、『正しい点』『誤った点』がそれぞれ この点数以上必要
BLOCK = 30                  # ブロック・ブートストラップのブロック長 (点)。連続した点は誤差が似るため
EARTH_R = 6371000.0


# ---------------------------------------------------------------
# 読み込み
# ---------------------------------------------------------------
def _num(tok):
    tok = tok.strip()
    return NAN if tok.lower() in ("", "nan", "none", "null") else float(tok)


def load_points(path):
    """txtから Point のリストを返す。日付を跨いだ場合は翌日として扱う。 (points, 読み飛ばした行数)"""
    with open(path, encoding="utf-8") as f:
        text = f.read()

    base = datetime(2000, 1, 1)  # 日付は不明なので仮置き(時刻の差だけ使う)
    day_offset = timedelta(0)
    prev = None
    points, skipped = [], 0
    for t_str, rest in BRACKET.findall(text):
        try:
            vals = [_num(x) for x in rest.split(",")[1:]]
        except ValueError:
            skipped += 1
            continue
        if len(vals) == 2:
            lat, lon = vals
            hacc = rel = heading = route = NAN
        elif len(vals) == 6:
            lat, lon, hacc, rel, heading, route = vals
        else:
            skipped += 1
            continue
        if isnan(lat) or isnan(lon):
            skipped += 1
            continue
        t = datetime.combine(base.date(), datetime.strptime(t_str, "%H:%M:%S").time())
        t += day_offset
        if prev is not None and t < prev:      # 0時を跨いだ
            day_offset += timedelta(days=1)
            t += timedelta(days=1)
        prev = t
        points.append(Point(t, lat, lon, hacc, rel, heading, route))
    return points, skipped


def read_latlon(path):
    with open(path, encoding="utf-8") as f:
        pts = [(float(a), float(b)) for a, b in PAIR.findall(f.read())]
    if len(pts) < 2:
        raise SystemExit(f"{path} から座標を読み取れませんでした")
    return pts


def haversine_m(p1, p2):
    lat1, lon1, lat2, lon2 = map(radians, (p1[0], p1[1], p2[0], p2[1]))
    a = sin((lat2 - lat1) / 2) ** 2 + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) ** 2
    return 2 * EARTH_R * atan2(sqrt(a), sqrt(1 - a))


# ---------------------------------------------------------------
# hAccの評価 (閾値の根拠を作るための集計)
# ---------------------------------------------------------------
def local_xy(latlon, lat0, lon0):
    """緯度経度 → lat0,lon0 を原点とする平面座標[m] (数km以内なら十分な近似)"""
    arr = np.asarray(latlon, float)
    k = EARTH_R * np.pi / 180.0
    return np.column_stack([(arr[:, 1] - lon0) * k * cos(radians(lat0)), (arr[:, 0] - lat0) * k])


def cross_track_errors(xy, ref_xy):
    """各点から基準の経路(折れ線)までの最短距離[m]"""
    A, B = ref_xy[:-1], ref_xy[1:]
    AB = B - A
    L2 = (AB ** 2).sum(axis=1)
    L2 = np.where(L2 > 0, L2, 1.0)
    out = np.empty(len(xy))
    for i, p in enumerate(xy):
        t = np.clip(((p - A) * AB).sum(axis=1) / L2, 0.0, 1.0)
        out[i] = np.min(np.linalg.norm(A + t[:, None] * AB - p, axis=1))
    return out


def implied_speed_outliers(points, max_speed):
    """前後の点との間の見かけの速さが max_speed を超える点 (位置が飛んだ可能性が高い点) を True にする"""
    n = len(points)
    sp = np.full(n - 1, np.nan)
    for i in range(1, n):
        dt = (points[i].t - points[i - 1].t).total_seconds()
        if dt > 0:
            sp[i - 1] = haversine_m((points[i - 1].lat, points[i - 1].lon), (points[i].lat, points[i].lon)) / dt
    out = np.zeros(n, bool)
    with np.errstate(invalid="ignore"):
        bad = sp > max_speed
    out[1:] |= bad
    out[:-1] |= bad
    return out


def plateau_mid(score, grid, tol=1e-9):
    """最大値をとる閾値が連続して並ぶとき、その中央を返す"""
    idx = np.where(score >= np.nanmax(score) - tol)[0]
    runs, cur = [], [idx[0]]
    for i in idx[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            runs.append(cur)
            cur = [i]
    runs.append(cur)
    run = max(runs, key=len)
    return float((grid[run[0]] + grid[run[-1]]) / 2)


def threshold_sweep(hacc, err, e_req, grid):
    """『誤差が e_req 以下の点 = 正しい点』とみなし、『hAcc ≤ H なら採用』という規則を、Hごとに評価する。
    各行: (H, 採用する割合, 感度=正しい点を採用できた割合, 特異度=誤った点を除外できた割合, J=感度+特異度-1, 適合率=採用した点のうち正しい割合)"""
    good = err <= e_req
    rows = []
    for H in grid:
        acc = hacc <= H
        tp, fp = int((acc & good).sum()), int((acc & ~good).sum())
        fn, tn = int((~acc & good).sum()), int((~acc & ~good).sum())
        sens = tp / (tp + fn) if tp + fn else np.nan
        spec = tn / (tn + fp) if tn + fp else np.nan
        prec = tp / (tp + fp) if tp + fp else np.nan
        rows.append((float(H), float(acc.mean()), sens, spec, sens + spec - 1, prec))
    return rows


def recommend_threshold(hacc, err, e_req, grid_step=0.5):
    """YoudenのJ (感度+特異度-1) が最大になる閾値を返す。 (H, その行) / 決められないときは (None, None)。
    正しい点か誤った点のどちらかが少なすぎる(MIN_BAD未満)ときは、決めない"""
    good = err <= e_req
    if good.sum() < MIN_BAD or (~good).sum() < MIN_BAD:
        return None, None
    grid = np.arange(grid_step, float(np.nanmax(hacc)) + grid_step, grid_step)
    rows = threshold_sweep(hacc, err, e_req, grid)
    J = np.array([r[4] for r in rows])
    if np.all(np.isnan(J)):
        return None, None
    H = plateau_mid(J, grid)
    return H, threshold_sweep(hacc, err, e_req, [H])[0]


def block_bootstrap_threshold(hacc, err, e_req, n_boot=1000, seed=0):
    """連続する点のまとまり(ブロック)ごとに復元抽出して、推奨閾値を求め直す。 (2.5%点, 97.5%点, 求まらなかった割合)"""
    rng = np.random.default_rng(seed)
    n = len(hacc)
    n_blocks = int(np.ceil(n / BLOCK))
    starts_max = max(n - BLOCK, 0)
    vals, fail = [], 0
    for _ in range(n_boot):
        idx = np.concatenate([np.arange(s, min(s + BLOCK, n)) for s in rng.integers(0, starts_max + 1, n_blocks)])[:n]
        H, _ = recommend_threshold(hacc[idx], err[idx], e_req)
        if H is None:
            fail += 1
        else:
            vals.append(H)
    if not vals:
        return None, None, 1.0
    lo, hi = np.percentile(vals, [2.5, 97.5])
    return float(lo), float(hi), fail / n_boot


def hacc_report(points, max_speed, ref_latlon=None, e_req=None, csv_path=None):
    """hAccと、(1)見かけの速さが異常な点の割合、(2)基準の経路からの誤差、の関係を表にして示す。
    戻り値: 推奨閾値 [m] (求まらなければ None)"""
    hacc = np.array([p.hacc for p in points])
    have = ~np.isnan(hacc)
    print(f"\n[hAccの評価] 点数 {len(points)} / hAccあり {int(have.sum())}")
    if have.sum() == 0:
        print("  hAccの値がないため、評価できません")
        return None
    q = np.percentile(hacc[have], [0, 25, 50, 75, 90, 95, 100])
    print("  hAccの分布 [m]: 最小 %.1f / 25%% %.1f / 中央 %.1f / 75%% %.1f / 90%% %.1f / 95%% %.1f / 最大 %.1f" % tuple(q))

    out_speed = implied_speed_outliers(points, max_speed)
    err = None
    if ref_latlon is not None:
        lat0 = float(np.mean([p.lat for p in points]))
        lon0 = float(np.mean([p.lon for p in points]))
        pos_xy = local_xy([(p.lat, p.lon) for p in points], lat0, lon0)
        err = cross_track_errors(pos_xy, local_xy(ref_latlon, lat0, lon0))

    header = "  hAcc区間[m]      点数   見かけの速さが異常(>%.1fm/s)の割合" % max_speed
    if err is not None:
        header += "   基準経路との誤差: 中央値 / 95%点 [m]   誤差≤hAccの割合"
    print(header)
    rows = []
    for lo, hi in zip(BIN_EDGES[:-1], BIN_EDGES[1:]):
        sel = have & (hacc >= lo) & (hacc < hi)
        n = int(sel.sum())
        if n == 0:
            continue
        label = f"{lo:g}〜{hi:g}" if np.isfinite(hi) else f"{lo:g}〜"
        row = [label, n, float(np.mean(out_speed[sel]))]
        line = f"  {label:>10}  {n:8d}   {100 * row[2]:6.1f} %"
        if err is not None:
            med, p95 = float(np.median(err[sel])), float(np.percentile(err[sel], 95))
            cov = float(np.mean(err[sel] <= hacc[sel]))
            row += [med, p95, cov]
            line += f"                  {med:6.2f} / {p95:6.2f}                {100 * cov:6.1f} %"
        print(line)
        rows.append(row)

    if csv_path:
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            cols = ["hacc_bin_m", "n", "implied_speed_outlier_rate"]
            if err is not None:
                cols += ["xtrack_err_median_m", "xtrack_err_p95_m", "share_err_le_hacc"]
            w.writerow(cols)
            w.writerows(rows)
        print(f"  → {csv_path}")

    if err is None:
        print("  (基準の経路 --ref がないため、位置の誤差は評価していません)")
        return None
    print("  ※「誤差」は基準の経路までの垂直な距離です。進行方向のずれは測れないので、実際の水平誤差より小さく出ます。")
    print("    また、基準が実際の歩行位置と違えば、その分が誤差に含まれます (実際の誤差には、位置が分かっている点での記録が必要)")
    print("  ※「誤差≤hAccの割合」が約68%ならhAccは1σ相当、約95%なら95%区間相当、と読めます (機種ごとに確かめる)")
    if e_req is None:
        print("  (--e-req [m] を指定すると、この表から推奨閾値を計算します)")
        return None

    h_, e_ = hacc[have], err[have]
    n_good, n_bad = int((e_ <= e_req).sum()), int((e_ > e_req).sum())
    print(f"\n  許容誤差 {e_req:g} m 以内を「正しい点」とすると: 正しい点 {n_good} 点 / 誤った点 {n_bad} 点")
    H, row = recommend_threshold(h_, e_, e_req)
    if csv_path:
        grid = np.arange(0.5, float(np.nanmax(h_)) + 0.5, 0.5)
        sweep_path = csv_path.replace("_hacc.csv", "_hacc_sweep.csv")
        with open(sweep_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["hacc_threshold_m", "share_accepted", "sensitivity", "specificity", "youden_j", "precision"])
            for r in threshold_sweep(h_, e_, e_req, grid):
                w.writerow([f"{r[0]:.2f}"] + [f"{v:.4f}" for v in r[1:]])
        print(f"  → {sweep_path}")
    if H is None:
        print(f"  正しい点か誤った点のどちらかが {MIN_BAD} 点未満のため、閾値は決められません"
              "(誤差が許容を超える点が少なすぎる、または多すぎる)")
        return None
    _, share, sens, spec, J, prec = row
    print(f"  推奨閾値: hAcc ≤ {H:.1f} m  (YoudenのJ = 感度+特異度-1 が最大。J = {J:.2f})")
    print(f"    採用する点: {100 * share:.0f} %  / 正しい点を採用できた割合(感度): {100 * sens:.0f} %"
          f"  / 誤った点を除外できた割合(特異度): {100 * spec:.0f} %  / 採用した点のうち正しい割合: {100 * prec:.0f} %")
    lo, hi, fail = block_bootstrap_threshold(h_, e_, e_req)
    if lo is not None:
        print(f"    ブロック・ブートストラップ(ブロック長 {BLOCK} 点)による95%区間: {lo:.1f} 〜 {hi:.1f} m"
              + (f"  (求まらなかった割合 {100 * fail:.0f} %)" if fail > 0 else ""))
    return H


# ---------------------------------------------------------------
# 地図
# ---------------------------------------------------------------
def fmt_rel(rel):
    if isnan(rel):
        return "-"
    return "0.0°" if abs(rel) < 0.05 else ("右 " if rel > 0 else "左 ") + f"{abs(rel):.1f}°"


def arrow_bearing(p, frame):
    """矢印の向き(地図上の方位)。 map: heading+相対角 / north: 相対角をそのまま (0°=真上)"""
    if isnan(p.rel):
        return None
    if frame == "north":
        return p.rel % 360
    if isnan(p.heading):
        return None
    return (p.heading + p.rel) % 360


def arrow_icon(bearing, color, opacity):
    svg = (f'<svg width="30" height="30" viewBox="-15 -15 30 30" style="overflow:visible">'
           f'<g transform="rotate({bearing:.1f})" opacity="{opacity}">'
           f'<line x1="0" y1="0" x2="0" y2="-11" stroke="{color}" stroke-width="2"/>'
           f'<polygon points="0,-14 -3.5,-8 3.5,-8" fill="{color}"/></g></svg>')
    return folium.DivIcon(html=svg, icon_size=(30, 30), icon_anchor=(15, 15), class_name="arrow-icon")


def build_map(points, out, gap_sec, max_speed, hacc_th, th_note, arrow_frame, ref_latlon, carto_key):
    coords = [(p.lat, p.lon) for p in points]
    if carto_key:
        m = folium.Map(
            location=coords[0], zoom_start=19, max_zoom=20,
            tiles=f"https://{{s}}.basemaps.cartocdn.com/rastertiles/voyager/{{z}}/{{x}}/{{y}}{{r}}.png?key={carto_key}",
            attr='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>',
        )
    else:          # キー不要のOpenStreetMap標準タイル
        m = folium.Map(location=coords[0], zoom_start=19, max_zoom=19, tiles="OpenStreetMap")

    # 矢印のアイコンが、下の点へのホバーを邪魔しないようにする
    folium.PolyLine(COORDS, color="red", weight=4, opacity=0.9).add_to(m)
    m.get_root().header.add_child(folium.Element("<style>.arrow-icon{pointer-events:none !important;}</style>"))

    cmap = LinearColormap(["blue", "blue", "blue", "blue"], vmin=0, vmax=max(len(points) - 1, 1))
    #["blue", "green", "yellow", "red"]
    def status_of(p):
        if isnan(p.hacc):
            return "unknown"
        return "valid" if p.hacc <= hacc_th else "invalid"

    status = [status_of(p) for p in points]
    n_valid, n_invalid, n_unknown = (status.count(s) for s in ("valid", "invalid", "unknown"))

    g_valid = folium.FeatureGroup(name=f"有効 (hAcc ≤ {hacc_th:g} m)  {n_valid} 点", show=True)
    g_invalid = folium.FeatureGroup(name=f"除外 (hAcc > {hacc_th:g} m)  {n_invalid} 点", show=True)
    g_unknown = folium.FeatureGroup(name=f"未評価 (hAccなし)  {n_unknown} 点", show=True)
    groups = {"valid": g_valid, "invalid": g_invalid, "unknown": g_unknown}
    g_line = folium.FeatureGroup(name="経路 (除外した点は含めない)", show=True)

    # --- 線: 除外した点は通らない。計測が途切れた箇所(gap_sec超)では線を切る ---
    seg, last_t = [], None
    for p, st in zip(points, status):
        if st == "invalid":
            continue
        if last_t is not None and (p.t - last_t).total_seconds() > gap_sec:
            if len(seg) > 1:
                folium.PolyLine(seg, color="gray", weight=3, opacity=0.6).add_to(g_line)
            seg = []
        seg.append((p.lat, p.lon))
        last_t = p.t
    if len(seg) > 1:
        folium.PolyLine(seg, color="gray", weight=3, opacity=0.6).add_to(g_line)

    if ref_latlon is not None:
        folium.PolyLine(ref_latlon, color="#1d6fe0", weight=3, opacity=0.8, dash_array="6 6",
                        tooltip="基準の経路").add_to(g_line)

    # --- 点: 時刻・hAcc・heading・経路角・相対角をホバー(tooltip)とクリック(popup)に埋め込む ---
    for i, p in enumerate(points):
        st = status[i]
        label = f"{p.t.strftime('%H:%M:%S')}"
        speed = None
        if i > 0:
            dt = (p.t - points[i - 1].t).total_seconds()
            if dt > 0:
                speed = haversine_m(coords[i - 1], coords[i]) / dt
        is_jump = speed is not None and speed > max_speed
        speed_txt = f"{speed:.1f} m/s" if speed is not None else "-"

        hacc_txt = f"{p.hacc:.1f} m" if not isnan(p.hacc) else "-"
        head_txt = f"{p.heading:.1f}°" if not isnan(p.heading) else "-"
        route_txt = f"{p.route:.1f}°" if not isnan(p.route) else "-"
        mark = {"valid": "", "invalid": "✕除外 ", "unknown": "(hAccなし) "}[st]
        tip = (f"{mark}{label} | hAcc {hacc_txt} | heading {head_txt} | 経路角 {route_txt} | 相対角 {fmt_rel(p.rel)}"
               + (" | ⚠急なジャンプ" if is_jump else ""))
        popup = (f"<b>{label}</b> #{i}<br>"
                 f"判定: {'有効' if st == 'valid' else '除外 (hAcc > ' + format(hacc_th, 'g') + ' m)' if st == 'invalid' else '未評価 (hAccなし)'}<br>"
                 f"hAcc: {hacc_txt}<br>heading: {head_txt}<br>経路角: {route_txt}<br>相対角: {fmt_rel(p.rel)}<br>"
                 f"lat: {p.lat:.6f}<br>lon: {p.lon:.6f}<br>前点からの速度: {speed_txt}")

        if st == "invalid":
            color, fill_color, opacity = "#888888", "#cccccc", 0.55
        else:
            color = fill_color = cmap(i)[:7]
            opacity = 0.9
        folium.CircleMarker(
            coords[i],
            radius=5 if is_jump else 3,
            color="black" if is_jump else color,    # 速度が異常な点は黒の縁取りで強調
            weight=2 if is_jump else 1,
            fill=True, fill_color=fill_color, fill_opacity=opacity,
            tooltip=tip,
            popup=folium.Popup(popup, max_width=240),
        ).add_to(groups[st])

        b = arrow_bearing(p, arrow_frame)
        if b is not None:
            folium.Marker(coords[i], icon=arrow_icon(b, "#888888" if st == "invalid" else color,
                                                    0.45 if st == "invalid" else 0.95)).add_to(groups[st])

    for g in (g_line, g_valid, g_unknown, g_invalid):
        g.add_to(m)

    # 開始/終了は、除外されていない点から選ぶ
    keep = [i for i, st in enumerate(status) if st != "invalid"] or list(range(len(points)))
    s, e = keep[0], keep[-1]
    folium.Marker(coords[s], tooltip=f"開始 {points[s].t:%H:%M:%S}",
                  icon=folium.Icon(color="green", icon="play")).add_to(m)
    folium.Marker(coords[e], tooltip=f"終了 {points[e].t:%H:%M:%S}",
                  icon=folium.Icon(color="red", icon="stop")).add_to(m)

    cmap.caption = "時間経過 (青=開始 → 赤=終了)"
    cmap.add_to(m)

    frame_txt = "heading + 相対角 (地図上の方位)" if arrow_frame == "map" else "相対角 (0°=真上)"
    legend = f"""
    <div style="position: fixed; bottom: 30px; left: 30px; z-index: 9999; background: white;
                padding: 8px 12px; border: 1px solid #888; border-radius: 4px; font-size: 13px; line-height: 1.6;">
      <b>hAcc の判定</b>: 閾値 {hacc_th:g} m <span style="color:#666">({th_note})</span><br>
      有効 {n_valid} 点 / 除外 {n_invalid} 点 / 未評価 {n_unknown} 点<br>
      <b>矢印</b> = 進むべき方向: {frame_txt}
    </div>"""
    m.get_root().html.add_child(folium.Element(legend))
    folium.LayerControl(collapsed=False).add_to(m)

    m.fit_bounds([[min(c[0] for c in coords), min(c[1] for c in coords)],
                  [max(c[0] for c in coords), max(c[1] for c in coords)]])
    m.save(out)
    return n_valid, n_invalid, n_unknown


def main():
    ap = argparse.ArgumentParser(description="GNSSログtxtを、hAcc・heading・経路角・相対角つきでfolium地図にする")
    ap.add_argument("input", help="ログのtxtファイル")
    ap.add_argument("-o", "--output", default="track_map.html")
    ap.add_argument("--gap", type=float, default=10.0,
                    help="この秒数より間が空いたら線を切る(既定10秒)")
    ap.add_argument("--max-speed", type=float, default=3.0,
                    help="前の点からの速度がこれを超えたらジャンプとして強調(m/s, 既定3。歩行者向け)")
    ap.add_argument("--hacc-th", type=float, default=None,
                    help="hAccがこの値[m]以下の点だけを有効とする。省略時は --ref/--e-req から求め、それも無ければ暫定値")
    ap.add_argument("--ref", default=None, help="基準の経路 (緯度,経度が並んだtxt)。誤差の実測に使う")
    ap.add_argument("--e-req", type=float, default=None,
                    help="許容する位置誤差[m]。誤差がこれ以下の点を「正しい点」とみなし、hAccの閾値を求める")
    ap.add_argument("--arrow-frame", choices=["map", "north"], default="map",
                    help="矢印の向き。map: heading+相対角 (既定) / north: 相対角を0°=真上で描く")
    ap.add_argument("--carto-key", default=os.environ.get("CARTO_API_KEY"),
                    help="CARTOのAPIキー(省略時は環境変数 CARTO_API_KEY、無ければOpenStreetMap)")
    args = ap.parse_args()

    points, skipped = load_points(args.input)
    if not points:
        raise SystemExit("座標を読み取れませんでした。形式を確認してください。")
    if skipped:
        print(f"注意: 形式が合わず読み飛ばした行が {skipped} 行あります")

    ref = read_latlon(args.ref) if args.ref else None
    csv_path = os.path.splitext(args.output)[0] + "_hacc.csv"
    recommended = hacc_report(points, args.max_speed, ref, args.e_req, csv_path)

    if args.hacc_th is not None:
        th, note = args.hacc_th, "--hacc-th で指定"
    elif recommended is not None:
        th, note = recommended, f"基準経路との誤差から決定 (許容誤差 {args.e_req:g} m, YoudenのJ最大)"
    else:
        th, note = DEFAULT_HACC_TH, "暫定値。根拠なし"
        print(f"\n注意: hAccの閾値は暫定値 {th:g} m です (根拠なし)。 --ref と --e-req で、データから求められます")

    nv, ni, nu = build_map(points, args.output, args.gap, args.max_speed, th, note,
                           args.arrow_frame, ref, CARTO_API_KEY)
    print(f"\n{len(points)}点を読み込み → {args.output}  (有効 {nv} / 除外 {ni} / 未評価 {nu}, 閾値 {th:g} m)")


if __name__ == "__main__":
    main()