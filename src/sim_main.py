#!/usr/bin/env python3
"""センサ・GNSS・ソレノイド・ラズパイ なしで main.py の動作をテストするシミュレータ。
Valhalla は PC で動いている本物を使う (--router fake にすると偽物で代用できる)。

事前準備: Valhalla の Docker コンテナを起動しておく  (例: docker start valhalla_gis-ops)
          確認:  curl http://localhost:8002/status

使い方 (main.py などがあるフォルダ = プロジェクトを指定する):
    python sim_main.py --project ~/Project/src                           # 標準シナリオ (本物の Valhalla)
    python sim_main.py --project ~/Project/src --scenario all
    python sim_main.py --project ~/Project/src --start 35.89195,139.944594 --dest 35.893922,139.952411
    python sim_main.py --project ~/Project/src --valhalla-url http://localhost:8002
    python sim_main.py --project ~/Project/src --router fake              # Valhalla なしで試す

必要なもの: numpy pyproj folium polyline requests (matplotlib は任意: 軌跡の図を出す)
※ gpiozero / smbus2 / pyubx2 / adafruit-bno055 / pyserial は不要 (偽物で置き換えるため)

何をしているか:
  1. 静的チェック : 全ファイルの構文エラー、ソレノイドのGPIO番号、磁気偏角の値などを調べる
  2. 計画経路     : main.py の fetch_route() で Valhalla から出発地→目的地の経路を取る
  3. 偽の世界     : 歩行者がその経路を歩く (GNSS誤差・方位誤差つき)。時計は仮想時間 (1時間分が数秒)
  4. 本物の main.py を実行: GNSS / HeadingReader / solenoid だけを偽物に差し替え、
                    main.py からの経路・再ルート要求は本物の Valhalla に送る
  5. 結果を判定   : 到着できたか、例外が出ないか、コーナーで正しい向きのソレノイドが上がったか など
"""
import argparse
import ast
import contextlib
import glob
import io
import math
import os
import re
import sys
import tempfile
import traceback
import types

import numpy as np

# ---------------------------------------------------------------------------
# シナリオ
# ---------------------------------------------------------------------------
SCENARIOS = {
    "normal":  dict(strict=True),
    "stop":    dict(strict=True, stops=[(60.0, 25.0)]),          # 60m地点で25秒立ち止まる
    "dropout": dict(strict=True, gnss_off=(60, 80), heading_off=(100, 110)),
    "deviate": dict(deviate=True),                               # 曲がり角を曲がらずに直進 → 再ルート
    "noisy":   dict(gnss_sigma=4.0, heading_sigma=12.0),         # 悪条件 (判定は参考)
    "valhalla_late": dict(strict=True, valhalla_fail_first=5),   # Valhalla起動前に5回失敗
}
WARN_DISTANCE = 15.0


class SimTimeout(BaseException):
    """仮想時間が上限を超えた (main が終わらない)"""


# ---------------------------------------------------------------------------
# 世界 (歩行者・仮想時計)
# ---------------------------------------------------------------------------
def bearing_xy(v):
    return math.degrees(math.atan2(v[0], v[1])) % 360


class World:
    def __init__(self, path_xy, to_latlon, speed=1.3, gnss_sigma=1.5, heading_sigma=4.0,
                 heading_offset=0.0, stops=(), seed=0):
        self.P = np.asarray(path_xy, float)
        self.to_latlon = to_latlon
        seg = np.linalg.norm(np.diff(self.P, axis=0), axis=1)
        self.cum = np.r_[0, np.cumsum(seg)]
        self.L = float(self.cum[-1])
        self.speed = speed
        self.gnss_sigma = gnss_sigma
        self.heading_sigma = heading_sigma
        self.heading_offset = heading_offset
        self.stops = list(stops)
        self.pause_left = 0.0
        self.rng = np.random.default_rng(seed)
        self.bias = np.zeros(2)
        self.t = 0.0
        self.s = 0.0

    def pos_at(self, s):
        s = min(max(s, 0.0), self.L)
        return np.array([np.interp(s, self.cum, self.P[:, 0]), np.interp(s, self.cum, self.P[:, 1])])

    def true_pos(self):
        return self.pos_at(self.s)

    def true_heading(self):
        a, b = self.pos_at(self.s), self.pos_at(self.s + 3.0)
        if np.linalg.norm(b - a) < 1e-6:
            a, b = self.pos_at(self.L - 3.0), self.pos_at(self.L)
        return bearing_xy(b - a)

    def advance(self, t_new):
        dt = t_new - self.t
        self.t = t_new
        n = int(round(dt / 0.1))
        for _ in range(n):
            if self.pause_left > 0:
                self.pause_left -= 0.1
                continue
            self.s = min(self.s + self.speed * 0.1, self.L)
            if self.stops and self.s >= self.stops[0][0]:
                self.pause_left = self.stops.pop(0)[1]

    def gnss_fix(self):
        self.bias = 0.9 * self.bias + self.rng.normal(0, self.gnss_sigma * 0.3, 2)   # ゆっくり変わる誤差
        xy = self.true_pos() + self.bias + self.rng.normal(0, self.gnss_sigma * 0.5, 2)
        lat, lon = self.to_latlon(xy[None, :])[0]
        return lat, lon

    def heading(self):
        return (self.true_heading() + self.heading_offset + self.rng.normal(0, self.heading_sigma)) % 360


class Clock:
    """仮想時計。sleep() は実際には待たず、時間を進めて世界を更新する"""

    def __init__(self, world, limit_sec):
        self.world = world
        self.t = 0.0
        self.limit = limit_sec

    def monotonic(self):
        return self.t

    def sleep(self, sec):
        self.t += max(sec, 0.0)
        self.world.advance(self.t)
        if self.t > self.limit:
            raise SimTimeout(f"仮想時間 {self.limit:.0f} 秒で終了しませんでした")


# ---------------------------------------------------------------------------
# 偽の Valhalla / ハードウェア
# ---------------------------------------------------------------------------
class FakeResp:
    def __init__(self, data):
        self._d = data

    def raise_for_status(self):
        pass

    def json(self):
        return self._d


class FakeRouter:
    """requests.post の代わり。planned (計画経路) の残りを返す。Valhalla 未起動の再現もできる"""

    def __init__(self, planned_latlon, to_xy, to_latlon, fail_first=0, rejoin_idx=None):
        import polyline
        self.polyline = polyline
        self.planned = list(planned_latlon)
        self.pxy = to_xy(self.planned)
        self.to_xy, self.to_latlon = to_xy, to_latlon
        self.fail_first = fail_first
        self.rejoin_idx = rejoin_idx
        self.calls = 0
        self.ok_calls = 0

    def post(self, url, json=None, timeout=None):
        self.calls += 1
        if self.calls <= self.fail_first:
            raise ConnectionError("(sim) Valhalla に接続できません")
        loc = json["locations"][0]
        cur = [loc["lat"], loc["lon"]]
        if self.ok_calls == 0:
            coords = [cur] + self.planned[1:]
        else:
            if self.rejoin_idx is not None:
                j = self.rejoin_idx
            else:                                    # 現在地に最も近い線分の次の頂点から合流
                p = self.to_xy([cur])[0]
                A, B = self.pxy[:-1], self.pxy[1:]
                AB = B - A
                t = np.clip(((p - A) * AB).sum(1) / np.where((AB ** 2).sum(1) > 0, (AB ** 2).sum(1), 1), 0, 1)
                d = np.linalg.norm(A + t[:, None] * AB - p, axis=1)
                j = int(np.argmin(d)) + 1
            coords = [cur] + self.planned[j:]
        self.ok_calls += 1
        shape = self.polyline.encode([tuple(c) for c in coords], precision=6)
        return FakeResp({"trip": {"legs": [{"shape": shape}]}})


class FakeSolenoid(types.ModuleType):
    def __init__(self):
        super().__init__("solenoid")
        self.solid, self.blink = [], []
        self.blink_all_called = False

    def LowAll(self):
        self.solid, self.blink = [], []

    def show(self, solid_list, blink_list=()):
        self.solid, self.blink = sorted(solid_list), sorted(blink_list)

    def blink_all_for(self, seconds):
        self.blink_all_called = True


# ---------------------------------------------------------------------------
# 経路の作成
# ---------------------------------------------------------------------------
START = (35.891950, 139.944594)
DEST = (35.893922, 139.952411)


def build_sample_route(latlon_to_xy, xy_to_latlon):
    """START→DEST を、左・右・左・右の4つの直角コーナーで結ぶ経路を作る (頂点は約25mおき)"""
    s = latlon_to_xy([START])[0]
    e = latlon_to_xy([DEST])[0]
    dx, dy = e - s
    legs = [(np.array([1.0, 0.0]), 0.35 * dx), (np.array([0.0, 1.0]), 0.55 * dy),
            (np.array([1.0, 0.0]), 0.30 * dx), (np.array([0.0, 1.0]), 0.45 * dy),
            (np.array([1.0, 0.0]), 0.35 * dx)]
    pts, cur, corner_idx = [s], s.copy(), []
    for k, (d, length) in enumerate(legs):
        n = max(int(length // 25), 1)
        for i in range(1, n + 1):
            cur = cur + d * (length / n)
            pts.append(cur.copy())
        if k < len(legs) - 1:
            corner_idx.append(len(pts) - 1)
    pts = np.array(pts)
    pts[-1] = e                                           # 終点を DEST に合わせる
    return pts, corner_idx


def parse_route_file(path):
    """logs/routes.txt の最後の経路 [(lat, lon), ...] を読む"""
    text = open(path, encoding="utf-8").read()
    found = re.findall(r"\[\(.*?\)\]", text, flags=re.S)
    if not found:
        raise SystemExit("routes.txt から経路を読めませんでした")
    txt = re.sub(r"np\.float64\(([^)]*)\)", r"\1", found[-1])      # numpy の数値表記が混ざっていても読む
    return [tuple(map(float, c)) for c in ast.literal_eval(txt)]


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def find_corners(P, win=3.0, min_turn=35.0, sep=10.0):
    """経路の形から曲がり角を探す。各地点で「手前win[m]の向き」と「先win[m]の向き」の差を調べ、
    大きい所を極大点として拾う。頂点が1つの直角でも、複数の頂点に分かれた折れ曲がりでも同じように見つかる。
    戻り値: [dict(s=弧長[m], dir='L'/'R', mag=曲がる角度[deg])]  (min_turn度以上のもの)"""
    P = np.asarray(P, float)
    cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    L = float(cum[-1])
    pos = lambda s: np.array([np.interp(s, cum, P[:, 0]), np.interp(s, cum, P[:, 1])])
    ss = np.arange(win, L - win + 1e-9, 1.0)
    delta = np.array([_wrap180(bearing_xy(pos(s + win) - pos(s)) - bearing_xy(pos(s) - pos(s - win))) for s in ss])
    out = []
    for i in np.argsort(-np.abs(delta)):
        if abs(delta[i]) < min_turn:
            break
        if all(abs(ss[i] - c["s"]) >= sep for c in out):
            out.append(dict(s=float(ss[i]), dir="R" if delta[i] > 0 else "L", mag=float(abs(delta[i]))))
    return sorted(out, key=lambda c: c["s"])


REQUIRED_TURN = 70.0     # この角度以上の曲がり角は、必ず接近表示が出るべきとして判定する
# (35〜70度の曲がりは、出ても出なくても良い「あいまいな曲がり」として、誤表示の判定から除外する)


# ---------------------------------------------------------------------------
# 静的チェック
# ---------------------------------------------------------------------------
def static_checks(project):
    res = []
    for f in sorted(glob.glob(os.path.join(project, "*.py"))):
        name = os.path.basename(f)
        try:
            compile(open(f, encoding="utf-8").read(), f, "exec")
            res.append(("PASS", f"構文: {name}"))
        except SyntaxError as e:
            res.append(("FAIL", f"構文エラー: {name} {e.lineno}行目: {e.msg}"))
    sp = os.path.join(project, "solenoid.py")
    if os.path.exists(sp):
        try:
            tree = ast.parse(open(sp, encoding="utf-8").read())
            for node in ast.walk(tree):
                if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "GPIOS" for t in node.targets):
                    g = ast.literal_eval(node.value)
                    bad = [x for x in g if x in (0, 1, 2, 3, 14, 15)]
                    if len(g) < 6:
                        res.append(("FAIL", f"solenoid.py: GPIOS が {len(g)} 個しかありません (6個必要)"))
                    elif len(set(g)) != len(g):
                        res.append(("FAIL", "solenoid.py: GPIOS に重複があります"))
                    else:
                        res.append(("PASS", f"solenoid.py: GPIOS={g}"))
                    if bad:
                        res.append(("WARN", f"solenoid.py: GPIOS に予約ピン {bad} を含みます "
                                            "(0,1=HAT用ID / 2,3=I2C / 14,15=UART)。配線を確認してください"))
        except Exception as e:
            res.append(("WARN", f"solenoid.py のGPIOS確認に失敗: {e}"))
    up = os.path.join(project, "userDirection.py")
    if os.path.exists(up):
        m = re.search(r"^MAG_DECLINATION_DEG\s*=\s*([-\d.]+)", open(up, encoding="utf-8").read(), flags=re.M)
        if m and abs(float(m.group(1))) > 15:
            res.append(("WARN", f"userDirection.py: MAG_DECLINATION_DEG = {m.group(1)}。日本の磁気偏角は約 -7°前後です。"
                                "意図した値か確認してください"))
    return res


# ---------------------------------------------------------------------------
# 本物の Valhalla を呼ぶ (呼び出し回数を数え、起動待ちの失敗を再現できる)
# ---------------------------------------------------------------------------
class RealRouter:
    def __init__(self, fail_first=0):
        import requests
        self._post = requests.post
        self.fail_first = fail_first
        self.calls = 0

    def post(self, url, json=None, timeout=None):
        self.calls += 1
        if self.calls <= self.fail_first:
            raise ConnectionError("(sim) Valhalla に接続できません (起動前の再現)")
        return self._post(url, json=json, timeout=timeout)


# ---------------------------------------------------------------------------
# 1シナリオの実行
# ---------------------------------------------------------------------------
def run_scenario(name, project, a, out_dir):
    cfg = SCENARIOS[name]
    sys.path.insert(0, project)
    for m in ("main", "turn_map", "GNSS", "userDirection", "solenoid", "cornerGuide", "evaluateIsCorner"):
        sys.modules.pop(m, None)

    # --- 偽のハードウェアモジュールを差し込む ---
    fake_sol = FakeSolenoid()
    sys.modules["solenoid"] = fake_sol
    holder = {}

    class FakeGNSSReader:
        def get(self, max_age=5.0):
            w, c = holder["world"], holder["clock"]
            off = cfg.get("gnss_off")
            if off and off[0] <= c.t < off[1]:
                return None
            lat, lon = w.gnss_fix()
            return dict(lat=lat, lon=lon, hacc_m=cfg.get("gnss_sigma", 1.5), speed_mps=w.speed,
                        course_deg=w.true_heading(), num_sv=12, t=c.t)

    class FakeHeadingReader:
        def get(self, max_age=2.0):
            w, c = holder["world"], holder["clock"]
            off = cfg.get("heading_off")
            if off and off[0] <= c.t < off[1]:
                return None
            return w.heading(), 3

    g = types.ModuleType("GNSS"); g.GNSSReader = FakeGNSSReader; sys.modules["GNSS"] = g
    u = types.ModuleType("userDirection"); u.HeadingReader = FakeHeadingReader; sys.modules["userDirection"] = u

    # --- main を読み込む (import 時の makedirs だけ止める) ---
    real_makedirs = os.makedirs
    os.makedirs = lambda *a_, **k: None
    sandbox = tempfile.mkdtemp(prefix=f"sim_{name}_")
    cwd0 = os.getcwd()
    os.chdir(sandbox)                                  # turn_map_lipar.html などはここに出る
    try:
        import turn_map
        import main as M
    finally:
        os.makedirs = real_makedirs

    M.BASE_DIR = sandbox
    M.LOG, M.ROUTES_LOG, M.COORDS_LOG = (os.path.join(sandbox, n) for n in ("log.txt", "routes.txt", "coords.txt"))
    to_latlon = lambda arr: np.array(turn_map.xy_to_latlon(np.asarray(arr)))
    start = a.start or START
    dest = a.dest or DEST
    use_fake = a.router == "fake"
    if not use_fake and a.valhalla_url:
        M.VALHALLA_ROUTE_URL = a.valhalla_url.rstrip("/") + "/route"

    result = dict(name=name, cfg=cfg, sandbox=sandbox, msgs=[], trace=[], err=None, finished=False,
                  corners=[], required=[], pxy=np.zeros((2, 2)), walk_xy=np.zeros((2, 2)), router=None,
                  world=None, clock=None, stdout="", sol=fake_sol, fake=use_fake)

    def abort(msg, tb=""):
        os.chdir(cwd0)
        sys.path.remove(project)
        result["err"] = ("setup", msg, tb)
        return result

    # --- 計画経路 (歩行者が歩く道) ---
    planning_out = io.StringIO()
    if a.route_file:
        planned = parse_route_file(os.path.expanduser(a.route_file))
        dest = planned[-1]
    elif use_fake:
        pxy0, _ = build_sample_route(turn_map.latlon_to_xy, turn_map.xy_to_latlon)
        planned = [tuple(p) for p in turn_map.xy_to_latlon(pxy0)]
        dest = planned[-1]
    else:
        # 本物の Valhalla に、main.py 自身の fetch_route() で問い合わせる (設定が main.py と必ず一致する)
        try:
            with contextlib.redirect_stdout(planning_out):
                planned = [tuple(c) for c in M.fetch_route(list(start), list(dest), False)["coords"]]
        except Exception as e:
            hint = ("\n  Valhalla が起動しているか確認してください:  curl http://localhost:8002/status"
                    "\n  起動していない場合:  docker start valhalla_gis-ops"
                    "\n  経路が見つからない場合は、--start / --dest がタイル(地図データ)の範囲内か確認してください。"
                    "\n  Valhalla なしで試すには --router fake")
            return abort(f"Valhalla から計画経路を取得できませんでした: {type(e).__name__}: {e}{hint}",
                         traceback.format_exc())
    M.DESTINATION = list(dest)
    pxy = turn_map.latlon_to_xy(planned)
    corners = find_corners(pxy)
    required = [c for c in corners if c["mag"] >= REQUIRED_TURN]

    walk_xy = pxy
    cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pxy, axis=0), axis=1))]
    posf = lambda s_: np.array([np.interp(s_, cum, pxy[:, 0]), np.interp(s_, cum, pxy[:, 1])])
    if cfg.get("deviate") and required:
        s0 = required[0]["s"]                          # 最初のコーナーを曲がらずに80m直進してから経路へ戻る
        din = posf(s0) - posf(s0 - 2.0)
        din = din / np.linalg.norm(din)
        s1 = min(s0 + 60.0, cum[-1] - 1.0)
        walk_xy = np.vstack([pxy[cum < s0], posf(s0), posf(s0) + din * 80.0, posf(s1), pxy[cum > s1]])

    world = World(walk_xy, to_latlon, gnss_sigma=cfg.get("gnss_sigma", 1.5),
                  heading_sigma=cfg.get("heading_sigma", 4.0), heading_offset=a.heading_offset,
                  stops=cfg.get("stops", ()), seed=a.seed)
    clock = Clock(world, a.limit_sec)
    holder.update(world=world, clock=clock)

    if use_fake:
        router = FakeRouter(planned, turn_map.latlon_to_xy, turn_map.xy_to_latlon,
                            fail_first=cfg.get("valhalla_fail_first", 0))
    else:
        router = RealRouter(fail_first=cfg.get("valhalla_fail_first", 0))
    M.requests = types.SimpleNamespace(post=router.post)
    M.time = types.SimpleNamespace(sleep=clock.sleep, monotonic=clock.monotonic)

    msgs, trace = [], []
    orig_logger, orig_show = M.logger, M.show_direction

    def lg(*a_, **k):                                  # 引数はそのまま渡す (呼び出しミスも本物どおり例外になる)
        msgs.append(str(a_[1] if len(a_) > 1 else k.get("content")))
        return orig_logger(*a_, **k)

    def sd(solid_deg, blink_deg, heading):
        orig_show(solid_deg, blink_deg, heading)
        trace.append(dict(t=clock.t, s=world.s, pos=world.true_pos().copy(), heading=heading,
                          solid=list(fake_sol.solid), blink=list(fake_sol.blink),
                          solid_deg=solid_deg, blink_deg=blink_deg))

    M.logger, M.show_direction = lg, sd

    # --- 実行 ---
    err, finished = None, False
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            M.main()
            finished = True
        except SimTimeout as e:
            err = ("timeout", str(e))
        except Exception:
            err = ("exception", traceback.format_exc())
    os.chdir(cwd0)
    sys.path.remove(project)
    result.update(msgs=msgs, trace=trace, err=err, finished=finished, router=router, world=world, clock=clock,
                  corners=corners, required=required, pxy=pxy, walk_xy=walk_xy,
                  stdout=planning_out.getvalue() + buf.getvalue())
    return result


# ---------------------------------------------------------------------------
# 判定
# ---------------------------------------------------------------------------
RIGHT, LEFT = {4, 5, 6}, {1, 2, 3}


def judge(r):
    res, cfg, tr = [], r["cfg"], r["trace"]

    if r["err"] and r["err"][0] == "setup":
        last = r["err"][2].strip().splitlines()[-1] if r["err"][2].strip() else ""
        res.append(("FAIL", r["err"][1] + (f"\n      (詳細: {last[:200]})" if last else "")))
        return res
    if r["err"] and r["err"][0] == "exception":
        res.append(("FAIL", "実行できませんでした:\n" + "".join("      " + l + "\n"
                    for l in r["err"][1].strip().splitlines()[-9:]).rstrip()))
        return res
    elif r["err"] and r["err"][0] == "timeout":
        res.append(("FAIL", f"main() が終わりません ({r['err'][1]})"))
    else:
        res.append(("PASS", "main() が最後まで実行された"))

    arrived = any("到着しました" in m for m in r["msgs"])
    res.append(("PASS" if arrived else "FAIL", "目的地への到着を検出した" if arrived
                else "目的地に到着できなかった (到着の検出が出ていない)"))

    fails = [m for m in r["msgs"] if "ルート取得失敗" in m or "再ルート失敗" in m]
    expect_fail = cfg.get("valhalla_fail_first", 0)
    if fails and not expect_fail:
        res.append(("FAIL", f"経路取得でエラー {len(fails)} 回: {fails[0][:160]}"))
    elif expect_fail:
        res.append(("PASS" if len(fails) == expect_fail and arrived else "FAIL",
                    f"Valhalla 起動待ち: {len(fails)} 回失敗後に取得できた (期待 {expect_fail} 回)"))

    reroutes = sum("再ルートします" in m for m in r["msgs"])
    if cfg.get("deviate"):
        if not r["required"]:
            res.append(("WARN", "この経路には曲がり角が無いため、逸脱シナリオを実行できませんでした"))
        # 歩行者は新しい経路に従わず、決めた道を歩き続ける。逸脱中は再ルートが繰り返されるので、回数は参考値
        res.append(("PASS" if reroutes >= 1 else "FAIL", f"逸脱を検出して再ルートした: {reroutes} 回 (期待 1回以上)"))
        if reroutes > 10:
            res.append(("WARN", f"再ルートが {reroutes} 回と多い。逸脱中の再ルート間隔(再ルートの連打)を確認してください"))
    else:
        res.append(("PASS" if reroutes == 0 else "FAIL", f"不要な再ルート {reroutes} 回 (期待 0)"))

    if cfg.get("heading_off") and any("Heading未取得" in m for m in r["msgs"]):
        res.append(("PASS", "Heading 欠測の間も落ちずに継続した"))
    if cfg.get("gnss_off") and any("GNSS未取得" in m for m in r["msgs"]):
        res.append(("PASS", "GNSS 欠測の間も落ちずに継続した"))

    if not tr:
        res.append(("FAIL", "ソレノイドへの表示が1回も行われなかった"))
        return res

    # --- コーナー案内 (歩行経路 = 計画経路 のシナリオのみ) ---
    if cfg.get("strict"):
        res.append(("INFO", f"計画経路の長さ {r['world'].L:.0f}m / 曲がり角 {len(r['corners'])} 個 "
                            f"(うち {REQUIRED_TURN:.0f}°以上の判定対象 {len(r['required'])} 個)"))
        if not r["required"]:
            res.append(("WARN", f"{REQUIRED_TURN:.0f}°以上の曲がり角が無い経路です。コーナー案内は確認できません"))
        for n, c in enumerate(r["required"], start=1):
            sc, d = c["s"], c["dir"]
            win = [x for x in tr if sc - WARN_DISTANCE - 3 <= x["s"] <= sc - 1 and x["blink"]]
            side = RIGHT if d == "R" else LEFT
            label = f"コーナー{n} ({'右' if d == 'R' else '左'} {c['mag']:.0f}°, {sc:.0f}m地点)"
            if not win:
                res.append(("FAIL", f"{label}: 接近の表示(点滅)が出なかった"))
                continue
            good = sum(1 for x in win if x["solid"] and set(x["solid"]) <= side)
            ratio = good / len(win)
            first = min(x["s"] for x in win)
            res.append(("PASS" if ratio >= 0.8 else "FAIL",
                        f"{label}: 接近表示 {len(win)}回, 正しい側のソレノイド {ratio * 100:.0f}% "
                        f"(表示開始 {sc - first:.0f}m手前)"))
        near = lambda s_: any(c["s"] - WARN_DISTANCE - 5 <= s_ <= c["s"] + 5 for c in r["corners"])
        false_ap = [x for x in tr if x["blink"] and not near(x["s"])]
        res.append(("PASS" if len(false_ap) <= 2 else "FAIL", f"曲がり角でない所での誤った接近表示: {len(false_ap)} 回"))
        straight = [x for x in tr if not near(x["s"]) and not x["blink"]]
        if straight:
            front = sum(1 for x in straight if x["solid"] == [1, 4]) / len(straight)
            st = "PASS" if front >= 0.8 else ("WARN" if front >= 0.6 else "FAIL")
            res.append((st, f"直進区間で「正面(①④)」が上がった割合: {front * 100:.0f}% "
                            "(道が緩やかに曲がる所は正面にならないことがある)"))

    changes = sum(1 for a_, b_ in zip(tr, tr[1:]) if a_["solid"] != b_["solid"])
    res.append(("INFO", f"表示 {len(tr)} 回 / 実線パターンの切り替え {changes} 回 / 仮想時間 {r['clock'].t:.0f} 秒 / "
                        f"歩行距離 {r['world'].s:.0f}m / 経路API呼び出し {r['router'].calls} 回 "
                        f"({'偽Valhalla' if r['fake'] else '本物のValhalla'})"))
    return res


def plot(r, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    if not r["trace"]:
        return False
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(*r["pxy"].T, "-", color="#bbb", lw=6, label="planned route")
    if len(r["walk_xy"]) != len(r["pxy"]):
        ax.plot(*r["walk_xy"].T, "--", color="#1f77b4", lw=1.5, label="actual walk (detour)")
    T = np.array([x["pos"] for x in r["trace"]])
    ax.plot(*T.T, ".", ms=2, color="#1f77b4", label="positions")
    if any(x["blink"] for x in r["trace"]):
        ap = np.array([x["pos"] for x in r["trace"] if x["blink"]])
        ax.plot(*ap.T, "o", ms=4, color="#d62728", label="approach (blink)")
    cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(r["pxy"], axis=0), axis=1))]
    for i, c in enumerate(r["required"], 1):
        xy = (np.interp(c["s"], cum, r["pxy"][:, 0]), np.interp(c["s"], cum, r["pxy"][:, 1]))
        ax.annotate(f"{i}{c['dir']}", xy, fontsize=12, color="k")
    ax.set_aspect("equal"); ax.legend(); ax.set_title(f"scenario: {r['name']}")
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return True


def parse_latlon(text):
    lat, lon = (float(x) for x in text.split(","))
    return lat, lon


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--project", default=".", help="main.py などがあるフォルダ")
    ap.add_argument("--scenario", default="normal", help=f"{', '.join(SCENARIOS)} / all")
    ap.add_argument("--router", choices=["valhalla", "fake"], default="valhalla",
                    help="valhalla: PC の本物を使う (既定) / fake: 偽物で代用")
    ap.add_argument("--valhalla-url", default="http://localhost:8002",
                    help="Valhalla のURL (main.py の VALHALLA_ROUTE_URL を、これに差し替えて実行する)")
    ap.add_argument("--start", type=parse_latlon, help="出発地 'lat,lon' (既定は main.py の例の地点)")
    ap.add_argument("--dest", type=parse_latlon, help="目的地 'lat,lon' (既定は main.py の DESTINATION)")
    ap.add_argument("--route-file", help="logs/routes.txt など。その経路を歩かせる (--start/--dest は無視)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--limit-sec", type=float, default=3600, help="仮想時間の上限(秒)")
    ap.add_argument("--heading-offset", type=float, default=0.0, help="方位センサの系統誤差[deg] (例: 25)")
    ap.add_argument("--out", default="sim_out", help="結果(図・ログ)の出力先")
    a = ap.parse_args()

    project = os.path.abspath(os.path.expanduser(a.project))
    if a.route_file:                                   # 実行中に作業フォルダを移るので、先に絶対パスにしておく
        a.route_file = os.path.abspath(os.path.expanduser(a.route_file))
        if not os.path.exists(a.route_file):
            sys.exit(f"--route-file が見つかりません: {a.route_file}")
    a.out = os.path.abspath(a.out)
    os.makedirs(a.out, exist_ok=True)
    names = list(SCENARIOS) if a.scenario == "all" else [a.scenario]
    total_fail = 0

    print("=" * 70)
    print("【1. 静的チェック】", project)
    checks = static_checks(project)
    for st, msg in checks:
        print(f"  [{st}] {msg}")
        total_fail += st == "FAIL"
    faked = ("GNSS.py", "userDirection.py", "solenoid.py")        # シミュレーションでは偽物に置き換えるファイル
    broken = [msg for st, msg in checks if st == "FAIL" and "構文" in msg]
    if any(not any(f in msg for f in faked) for msg in broken):
        print("\n本体側に構文エラーがあるため、シミュレーションは実行できません。FAIL を直してから再実行してください。")
        sys.exit(1)
    if broken:
        print("\n※ 構文エラーのファイルは、シミュレーションでは偽物に置き換えるため先へ進みますが、実機では動きません。")

    for name in names:
        print("\n" + "=" * 70)
        print(f"【2. シナリオ: {name}】{SCENARIOS[name]}  router={a.router}")
        r = run_scenario(name, project, a, a.out)
        for st, msg in judge(r):
            print(f"  [{st}] {msg}")
            total_fail += st == "FAIL"
        open(os.path.join(a.out, f"{name}_stdout.txt"), "w", encoding="utf-8").write(r["stdout"])
        if plot(r, os.path.join(a.out, f"{name}.png")):
            print(f"  → 軌跡の図: {os.path.join(a.out, name + '.png')}")
        log = os.path.join(r["sandbox"], "log.txt")
        if os.path.exists(log):
            import shutil
            shutil.copy(log, os.path.join(a.out, f"{name}_log.txt"))

    print("\n" + "=" * 70)
    print("結果:", "すべて PASS" if total_fail == 0 else f"FAIL が {total_fail} 件あります")
    sys.exit(0 if total_fail == 0 else 1)


if __name__ == "__main__":
    main()