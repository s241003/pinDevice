import folium
import numpy as np
from pyproj import Transformer



# ---------------------------------------------------------------
# パラメータ
# ---------------------------------------------------------------
EPSG_M = "EPSG:6677"     # 平面直角座標系IX系 (関東)。地域に合わせて変更する
STEP_M = 1.0             # 再サンプリング間隔 [m]
SMOOTH_K = 5             # 方位の平滑化に使う点数
RATE_THR = 2.0           # 方位の変化率がこれ [deg/m] を超えた所を「曲がり区間」とみなす
MERGE_GAP = 4            # この点数以内に近接する曲がり区間は1つにまとめる
CORNER_MAX_R = 10.0      # 平均曲率半径がこれ [m] 以下ならコーナー、超えればカーブ
SHOW_POINTS = True       # 元の点を小さな円で表示する
OUTPUT = "turn_map.html"
CARTO_KEY = "cb1_44j7_1_420580e3a4f5727c0fe3bb6e"
MIN_TURN_DEG = 90        # 総曲がり角がこれ未満なら「直線」扱い (小さな屈曲を含む)                   #
COLORS = {"直線": "#1f77b4", "コーナー": "#d62728", "カーブ": "#ff7f0e"}


# ---------------------------------------------------------------
# 座標変換・再サンプリング
# ---------------------------------------------------------------
_to_m = Transformer.from_crs("EPSG:4326", EPSG_M, always_xy=True)
_to_deg = Transformer.from_crs(EPSG_M, "EPSG:4326", always_xy=True)


def latlon_to_xy(latlon):
    """(緯度, 経度) のリスト → メートル座標の (N,2) 配列"""
    lat = np.array([p[0] for p in latlon])
    lon = np.array([p[1] for p in latlon])
    x, y = _to_m.transform(lon, lat)
    return np.column_stack([x, y])


def xy_to_latlon(P):
    """メートル座標の (N,2) 配列 → [(緯度, 経度), ...]"""
    lon, lat = _to_deg.transform(P[:, 0], P[:, 1])
    return list(zip(lat, lon))


def resample(P, step):
    """弧長 step[m] ごとの点に取り直す"""
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    s_new = np.arange(0, s[-1], step)
    return np.column_stack([np.interp(s_new, s, P[:, 0]),
                            np.interp(s_new, s, P[:, 1])])


# ---------------------------------------------------------------
# 曲がり区間の検出と分類
# ---------------------------------------------------------------
def detect_turns(Q, step, k, thr, merge_gap):
    """Q(1m間隔の点列)から曲がり区間を検出する。

    各区間は dict(a, b, L, Phi, R)。a, b は Q の線分番号 (a番目〜b+1番目の線分が曲がり区間)。
    """
    d = np.diff(Q, axis=0)
    theta = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    ker = np.ones(k) / k
    th = np.convolve(np.pad(theta, (k // 2, k // 2), mode="edge"), ker, mode="valid")

    rate = np.degrees(np.diff(th)) / step      # 方位の変化率 [deg/m]
    mask = np.abs(rate) > thr

    regions, i = [], 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j + 1 < len(mask) and mask[j + 1]:
                j += 1
            if regions and i - regions[-1][1] <= merge_gap:
                regions[-1] = (regions[-1][0], j)
            else:
                regions.append((i, j))
            i = j + 1
        else:
            i += 1

    turns = []
    for a, b in regions:
        L = (b - a + 1) * step
        Phi = np.degrees(th[min(b + 1, len(th) - 1)] - th[a])
        R = L / abs(np.radians(Phi)) if abs(Phi) > 1e-6 else np.inf
        turns.append(dict(a=a, b=b, L=L, Phi=Phi, R=R))
    return turns


def classify(turn,min_phi=MIN_TURN_DEG):
    """曲がり区間を「直線 / コーナー / カーブ」に分類する (検出側がlabelを付けていればそれを使う)"""
    if "label" in turn:
        return turn["label"]
    if abs(turn["Phi"]) < min_phi:
        return "直線"
    return "コーナー" if turn["R"] <= CORNER_MAX_R else "カーブ"


# ---------------------------------------------------------------
# 描画
# ---------------------------------------------------------------
def make_tile_kwargs():
    if CARTO_KEY:
        return dict(
            tiles=("https://{s}.basemaps.cartocdn.com/rastertiles/voyager/"
                   "{z}/{x}/{y}{r}.png?key=" + CARTO_KEY),
            attr=('&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> '
                  'contributors &copy; <a href="https://carto.com/attributions">CARTO</a>'),
        )
    return dict(tiles="OpenStreetMap")


LEGEND_HTML = """
<div style="position: fixed; bottom: 30px; left: 30px; z-index: 9999;
            background: white; padding: 8px 12px; border: 1px solid #888;
            border-radius: 4px; font-size: 14px; line-height: 1.6;">
  <b>凡例</b><br>
  <span style="color:%(直線)s;">━━</span> 直線<br>
  <span style="color:%(コーナー)s;">━━</span> コーナー<br>
  <span style="color:%(カーブ)s;">━━</span> カーブ
</div>
""" % COLORS


def build_map(latlon, detector=None):
    P = latlon_to_xy(latlon)
    Q = resample(P, STEP_M)

    turns = (detector or detect_turns)(Q, STEP_M, SMOOTH_K, RATE_THR, MERGE_GAP)
    corners = extract_corners(latlon,turns)


    # 線分ごとにラベルと詳細を付ける (既定は直線)
    n_seg = len(Q) - 1
    seg_label = ["直線"] * n_seg
    seg_info = [None] * n_seg
    for t in turns:
        label = classify(t)
        for i in range(t["a"], min(t["b"] + 2, n_seg)):
            seg_label[i] = label
            seg_info[i] = t

    # 同じラベルが続く線分を1本のポリラインにまとめる
    runs, start = [], 0
    for i in range(1, n_seg + 1):
        if i == n_seg or seg_label[i] != seg_label[start] or seg_info[i] is not seg_info[start]:
            runs.append((start, i - 1))
            start = i

    center = np.mean(np.array(latlon), axis=0).tolist()
    m = folium.Map(location=center, zoom_start=17, max_zoom=20, **make_tile_kwargs())

    for s, e in runs:
        label = seg_label[s]
        coords = xy_to_latlon(Q[s:e + 2])
        info = seg_info[s]
        if info is not None:
            popup = (f"{label}<br>総曲がり角: {info['Phi']:.0f}°<br>"
                     f"長さ: {info['L']:.0f} m<br>平均曲率半径: {info['R']:.1f} m")
        else:
            popup = label
        folium.PolyLine(coords, color=COLORS[label],
                        weight=7 if label != "直線" else 4,
                        opacity=0.9, tooltip=label,
                        popup=folium.Popup(popup, max_width=220)).add_to(m)

    if SHOW_POINTS:
        for lat, lon in latlon:
            folium.CircleMarker((lat, lon), radius=2, color="#444",
                                fill=True, fill_opacity=1, weight=1).add_to(m)

    m.fit_bounds(latlon)
    m.get_root().html.add_child(folium.Element(LEGEND_HTML))
    return m, turns,[list((corners[i]["lat"],corners[i]["lon"])) for i in range(len(corners))]

def extract_corners(coords,turns):
    P = latlon_to_xy(coords)
    Q = resample(P, STEP_M)
    #d = np.diff(Q, axis=0)
    theta = np.zeros(len(Q))
    """theta[1:-1] = np.arctan2(d[:-1, 0] * d[1:, 1] - d[:-1, 1] * d[1:, 0],
                             (d[:-1] * d[1:]).sum(axis=1))"""
    corners = []
    for t in turns:
        if classify(t) != "コーナー":
            continue
        # コーナーの位置: 区間内で、曲がり角の絶対値を重みにした重心の点 (経路上の点に丸める)
        lo = max(t["a"] + 1, 1)
        hi = min(t["b"] + 2, len(Q) - 2)
        idx = np.arange(lo, hi + 1)
        w = np.abs(theta[idx])
        i = int(round(np.average(idx, weights=w))) if w.sum() > 1e-9 else int((lo + hi) // 2)

        lat, lon = xy_to_latlon(Q[i:i + 1])[0]
        corners.append(dict(
            no=len(corners) + 1,
            lat=float(lat), lon=float(lon),
            dist_m=float(i * STEP_M),          # 経路の始点からの道のり
            angle_deg=float(t["Phi"]),                    # 総曲がり角 (左が正、右が負)
            direction="左" if t["Phi"] > 0 else "右",
            L_m=float(t["L"]), R_m=float(t["R"]),
        ))
    return corners