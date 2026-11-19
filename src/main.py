import argparse
import os
import time
import datetime
from math import radians, sin, cos, sqrt, atan2, degrees

import requests
import polyline
import folium
import numpy as np
from pyproj import Transformer
from reroute_guard import is_same_route,RerouteGuard
from turn_map import extract_corners
from vincenty import vincenty_inverse
from cornerGuide import *
from userDirection import HeadingReader
from decideSolenoids import whichSolenoid
from lookahead import target_bearing
from solenoid import Display
from GNSS import GNSSReader
from destination_loader import wait_for_destination
from buttonPress import ButtonLogger
from velocity import WalkSpeed


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ログのpath
LOG_DIR = os.path.join(BASE_DIR, "..", "logs")
os.makedirs(LOG_DIR, exist_ok=True)
LOG = os.path.join(LOG_DIR, "log.txt")
ROUTES_LOG = os.path.join(LOG_DIR, "routes.txt")
COORDS_LOG = os.path.join(LOG_DIR, "coords.txt")
HEADING_LOG = os.path.join(LOG_DIR, "heading.txt")
ERROR_LOG=os.path.join(LOG_DIR, "error.txt")
MAP_DIR = os.path.join(BASE_DIR, "../Maps", "routes")
CORNERS_LOG = os.path.join(LOG_DIR, "corners.txt")

VALHALLA_ROUTE_URL = "http://localhost:8002/route"
CARTO_KEY = "cb1_44j7_1_420580e3a4f5727c0fe3bb6e"
MAP_TILE="voyager"


DESTINATION_FILE = os.path.join(BASE_DIR, "destination.json")
DESTINATION = [35.892690,139.941418]
COSTING = "pedestrian"
REROUTE_THRESHOLD_M = 20      # 逸脱とみなす距離の閾値(m)
POLL_INTERVAL_SEC = 1.0       # 更新間隔(s)  GNSS(1Hz)にあわせる
SNAP_RADIUS_M=50              # GPS始点とマップ始点の距離の許容範囲(m)
TIMEOUT_SEC = 10              # 応答が遅すぎると判断する時間(s)
ARRIVAL_RADIUS_M = 15         # 到着とみなす距離の閾値(m)
ARRIVAL_CONFIRM_COUNT = 3     # 何回連続で範囲内なら到着とするか(回)
REJECT_HACC=10                # 棄却するGNSSの誤差(m)
ARRIVAL_WAVE = [(1, 2, 3), (4, 5, 6)]   # 2x3用：到着の波（左列→右列）
ARRIVAL_WAVE_4 = [(1, 5), (2, 4)]        # 2x2用：到着の波（左列→右列）
REROUTE_WAVE = [(1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 1)]  # 2x3用：追いかけ点滅
REROUTE_WAVE_4 = [(1, 2), (2, 5), (5, 4), (4, 1)]  # 2x2用：追いかけ点滅（外周を1周）
# --pin-mode で選んだモードに応じて、__main__ が次の2つを書き換える
arrival_wave = ARRIVAL_WAVE
reroute_wave = REROUTE_WAVE

# ピンモード → ピンの数。2x2 は上2行の4本（ピン番号 1,2,4,5）だけ使う
PIN_MODES = {"4": 4, "6": 6}


BUTTON_GPIO = 12             # タクトスイッチの GPIO (ソレノイド 23,24,25,8,7,1 / I2C 2,3 / UART 14,15 と重ならない番号)
BUTTON_LOG = os.path.join(LOG_DIR, "button.txt")
SPEED_WINDOW_SEC = 30.0      # 歩行速度の平均を取る窓 [s]
WARN_LEAD_SEC = 15.0         # 点滅開始は「コーナーの約この秒数だけ手前」にする (提案値・分類B)
WARN_ENTER_MIN_M = 8.0       # 点滅開始距離の下限・上限 [m]。速度推定が外れても破綻しないための囲い
WARN_ENTER_MAX_M = 25.0
ARRIVE_LEAD_SEC = 15.0       # 到着半径も「目的地の約この秒数だけ手前」を基準にする (提案値・分類B)
ARRIVE_RADIUS_MIN_M = 10.0   # hAcc の棄却基準 (REJECT_HACC=10m) より小さくしない
ARRIVE_RADIUS_MAX_M = 25.0


def parse_args(argv=None):
    """コマンドライン引数を読み取る。--pin-mode 2x2 / 2x3（既定は 2x3）"""
    parser = argparse.ArgumentParser(description="点字ピン方向定位デバイス（2x2 / 2x3 ピンモード切替）")
    parser.add_argument("--pin-mode", choices=["4", "6"], default="6",
                        help="ソレノイドの並び: 4=2x2(ピン1,2,4,5)、6=2x3(既定)")
    return parser.parse_args(argv)

EPSG_M = "EPSG:6677"
_to_m = Transformer.from_crs("EPSG:4326", EPSG_M, always_xy=True)
_to_deg = Transformer.from_crs(EPSG_M, "EPSG:4326", always_xy=True)

class ArrivalDetector:
    def __init__(self, destination, radius_m=ARRIVAL_RADIUS_M, confirm=ARRIVAL_CONFIRM_COUNT):
        self.dest = destination
        self.radius_m = radius_m
        self.confirm_count = confirm
        self._count = 0
        self.arrived = False

    def update(self, p):
        """output: 到着したか判定(bool)"""
        if self.arrived:
            return True
        if vincenty_inverse(p, self.dest)["distance"] <= self.radius_m:
            self._count += 1
        else:
            self._count = 0
        if self._count >= self.confirm_count:
            self.arrived = True
        return self.arrived


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))

def warn_enter_m(speed_mps):
    """点滅開始距離 [m] = 速度 × WARN_LEAD_SEC (囲い付き)。速度が無ければ従来の固定値"""
    if speed_mps is None:
        return CORNER_ENTER_DIST
    return _clamp(speed_mps * WARN_LEAD_SEC, WARN_ENTER_MIN_M, WARN_ENTER_MAX_M)

def arrival_radius_m(speed_mps):
    """到着半径 [m] = 速度 × ARRIVE_LEAD_SEC (囲い付き)。速度が無ければ従来の固定値"""
    if speed_mps is None:
        return ARRIVAL_RADIUS_M
    return _clamp(speed_mps * ARRIVE_LEAD_SEC, ARRIVE_RADIUS_MIN_M, ARRIVE_RADIUS_MAX_M)


def logger(filePath, content, mode="indentionW"):
    print(content)
    dt_now = datetime.datetime.now()
    t = dt_now.strftime('[%Y-%m-%d %H:%M:%S]    ')
    if filePath == COORDS_LOG or filePath== HEADING_LOG:
        t = dt_now.strftime('[%H:%M:%S, ')
    sep = "\n"
    if mode == "indention":
        sep = "   "
    elif mode == "init":
        t, sep = "\n\n=========================\n", "\n"
    elif mode == "end":
        t, sep = "\n", "\n=========================\n\n"
    with open(filePath, "a", encoding="utf-8") as f:     # 文字コードを明示 (環境のロケールに依存しない)
        print(f"{t}{content}{sep}", file=f)

def cumulative_route_distance(coords):
    """
    route上の各点までの累積距離[m]
    """
    acc = [0.0]
    for i in range(len(coords) - 1):
        acc.append(acc[-1] + vincenty_inverse(coords[i], coords[i + 1])["distance"])

    return acc

def check_route_shape(cur, coords, leg_length_m):
    """Valhallaが返した形が「使える経路」か、3つの条件で確かめる。"""
    if leg_length_m <= 0:
        return False, "legの長さが0"
    if len(coords) < 2:
        return False, "形の点が2点未満"
    if coords[0] == coords[-1]:
        return False, "形の始点と終点が同じ点"
    v = vincenty_inverse(cur, coords[0])
    if v is None or v["distance"] > SNAP_RADIUS_M:
        return False, "形の始点が現在地から50mより遠い"
    return True, ""

def fetch_route(cur, dest, save_map=False):
    body = {
        "locations": [
            {"lat": cur[0], "lon": cur[1],"radius": SNAP_RADIUS_M},
            {"lat": dest[0], "lon": dest[1],"radius": SNAP_RADIUS_M},
        ],
        "costing": COSTING,
        "costing_options": {"pedestrian": {"sidewalk_factor":0.5,"walkway_factor":0.5,"alley_factor": 3.0, "driveway_factor": 25.0}},
        "id": "reroute_demo",
    }
    resp = requests.post(VALHALLA_ROUTE_URL, json=body, timeout=TIMEOUT_SEC)
    resp.raise_for_status()
    leg = resp.json()["trip"]["legs"][0]
    coords = polyline.decode(leg["shape"], precision=6)
    ok, reason = check_route_shape(cur, coords, leg.get("summary", {}).get("length", 0))
    if not ok:
        logger(ERROR_LOG,"valhalla:不正経路を取得")
        raise ValueError(f"Valhallaの経路が使えない: {reason}")

    logger(ROUTES_LOG, coords, "indention")

    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    distances = cumulative_route_distance(coords)
    bearings = []
    for i in range(len(coords) - 1):
        v = vincenty_inverse([lats[i], lons[i]], [lats[i + 1], lons[i + 1]])
        bearings.append(v["azimuth1"])

    if save_map:
        m = folium.Map(
            location=[(lats[0] + lats[-1]) / 2, (lons[0] + lons[-1]) / 2],
            zoom_start=18,
            tiles=f"https://{{s}}.basemaps.cartocdn.com/rastertiles/{MAP_TILE}/{{z}}/{{x}}/{{y}}{{r}}.png?key={CARTO_KEY}",
            attr='&copy; OpenStreetMap contributors &copy; CARTO',
            max_zoom=20,
        )
        folium.PolyLine(coords, color="red", weight=4, opacity=0.9).add_to(m)
        folium.Marker([lats[0], lons[0]], popup="現在地", icon=folium.Icon(color="blue")).add_to(m)
        folium.Marker([lats[-1], lons[-1]], popup="目的地", icon=folium.Icon(color="green")).add_to(m)
        os.makedirs(MAP_DIR, exist_ok=True)         # フォルダが無いと保存に失敗し、経路取得を延々やり直してしまう
        dt_now = datetime.datetime.now()
        t = dt_now.strftime('%m%d_%H%M')
        m.save(os.path.join(MAP_DIR, f"{t}route_map.html"))

    corners = extract_corners(coords,distances,save_map)
    return {"coords": coords,
            "distances": distances,
            "corners": corners,
            "bearings": bearings}

def latlon_to_xy(latlon):
    """(緯度, 経度) のリスト → メートル座標の (N,2) 配列"""
    lat = np.array([p[0] for p in latlon])
    lon = np.array([p[1] for p in latlon])
    x, y = _to_m.transform(lon, lat)
    return np.column_stack([x, y])

def find_edge(coords, current, max_dist=REROUTE_THRESHOLD_M):
    """最傍エッジの番号と、その距離[m]を返す"""
    P = latlon_to_xy(coords)
    p = latlon_to_xy([current])[0]
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = (AB ** 2).sum(axis=1)
    t = ((p - A) * AB).sum(axis=1) / np.where(L2 > 0, L2, 1.0)
    t = np.clip(t, 0.0, 1.0)
    d = np.linalg.norm(A + t[:, None] * AB - p, axis=1)
    i = int(np.argmin(d))
    return (i if d[i] <= max_dist else None), float(d[i])

def plotElevSolenoids(sole_set, mode,pinN):
    marks = ["①", "②", "③", "④", "⑤", "⑥"]
    if mode=="solid":
        for i in sole_set:
            marks[i - 1] = "●"
    elif mode.startswith("blink"):
        for i in sole_set:
            marks[i - 1] = "○"
    a, b, c, d, e, f = marks
    if pinN==6:
        elevSoles=f"\n[{a}] [{d}]\n[{b}] [{e}]\n[{c}] [{f}]"
    elif pinN==4:
        elevSoles=f"\n[{a}] [{d}]\n[{b}] [{e}]"
    return elevSoles

def show_direction(solenoid,edge_direction, mode, heading,pinN):
    sole_set = whichSolenoid((edge_direction - heading) % 360, pinN)
    solenoid.set(sole_set, mode)
    elevSoles=plotElevSolenoids(sole_set, mode,pinN)
    return sole_set,elevSoles

def main(solenoid,pinN): #core
    """
    logger(LOG, "目的地を待っています… スマホで目的地を設定してください")
    solenoid.set(set(), "solid")
    dest = wait_for_destination(DESTINATION_FILE, poll_sec=2.0, log=lambda m: logger(LOG, m))
    destination = [dest["lat"], dest["lon"]]
    logger(LOG, f"目的地: {dest['name']} lat={dest['lat']} lon={dest['lon']}")
    """
    destination=DESTINATION
    gnss = GNSSReader()
    head = HeadingReader()
    detector = ArrivalDetector(destination)
    guard = RerouteGuard()
    speed = WalkSpeed(window_sec=SPEED_WINDOW_SEC)
    blink_state = {"start_mono": None}   # 直近の点滅開始の単調時刻。ボタン記録が参照する
    prev_mode = "solid"
    try:
        button = ButtonLogger(BUTTON_GPIO, CORNERS_LOG,
                              blink_start_fn=lambda: blink_state["start_mono"])
    except Exception as e:
        logger(ERROR_LOG, f" ボタン初期化失敗: {e}")
        button = None
    route = None
    logger(LOG, "開始", "init")

    while True:
        t0 = time.monotonic()
        fix = gnss.get(max_age=5.0)
        if fix is None:
            logger(LOG, "GNSS未取得...")
            logger(ERROR_LOG,f" GNSS：未取得")
            solenoid.set(set(),"solid")
            time.sleep(POLL_INTERVAL_SEC)
            continue
        pos = [fix["lat"], fix["lon"]]
        if fix["hacc_m"] >= REJECT_HACC:
            logger(LOG, f"信頼できない座標データ。hAcc{fix['hacc_m']}")
            time.sleep(POLL_INTERVAL_SEC)
            continue


        if fix.get("speed_mps") is not None:
            speed.add(fix["speed_mps"])
        v_mps = speed.speed_mps() if speed.ready() else None

        # --- ルート取得 (初回) ---
        if route is None:
            logger(LOG, "初回ルート取得中...")
            print(f"current:{pos},目的地:{destination}")
            try:
                route = fetch_route(pos, destination, True)
            except Exception as e:
                logger(LOG, f"ルート取得失敗(Valhallaエラー)")
                logger(ERROR_LOG,f" Valhalla：{e}")
                solenoid.set(reroute_wave,"wave",0.4)
                route = None
                time.sleep(2)
                continue

        curIndex, distance = find_edge(route["coords"], pos)
        # --- 逸脱したら再ルート ---
        if curIndex is None:
            if not guard.should_reroute(distance, time.monotonic()):
                # まだ確認中 (3回連続で20m以上、かつ前回から30秒以上を待つ)。
                # 連続失敗で止まっている場合もここに入る。
                solenoid.set(set(), "solid")
                time.sleep(POLL_INTERVAL_SEC)
                continue
            try:
                new_route = fetch_route(pos, destination, False)
                if is_same_route(route, new_route):
                    logger(LOG, "新しい経路は古い経路と同じなので上書きしない")
                    guard.note_success()
                else:
                    route = new_route
                    reset_corner_state()
                    prev_mode = "solid"
                    guard.note_success()
                curIndex, distance = find_edge(route["coords"], pos)
            except Exception as e:
                if guard.note_failure():
                    logger(LOG, "再ルートを3回連続で失敗したので止めます")
                logger(LOG, f"再ルート失敗")
                logger(ERROR_LOG,f" 再ルート：{e}")
            if curIndex is None:                         # try/except のどちらでも拾う
                solenoid.set(reroute_wave,"wave",0.4)
                time.sleep(POLL_INTERVAL_SEC)
                continue

        # --- メイン処理 ---
        edge_bearing = vincenty_inverse(pos,route["coords"][curIndex + 1])["azimuth1"]  # 目標は「次の頂点」(curIndex は今いるエッジの始点=後ろの頂点)  #route["bearings"][curIndex] if curIndex is not None else None
        # 目標方位は「経路上の少し先の点 (lookahead)」への方位にする。
        # 経路から横にずれていれば方位が経路側へ傾き、歩くほど経路へ戻る。
        # 目標点が定まらない場合だけ、従来のエッジ方位にフォールバックする。
        guide_bearing = edge_bearing

        hd = head.get()
        if hd is None:
            logger(LOG, "Heading未取得")
            logger(ERROR_LOG,f" 9軸：未取得")
            solenoid.set(set(),"solid")
        else:
            heading, mag_cal = hd
            mode = get_guidance_mode(pos,curIndex,route, enter_dist=warn_enter_m(v_mps))
            if mode.startswith("blink") and not prev_mode.startswith("blink"):
                blink_state["start_mono"] = time.monotonic()
                logger(LOG, f"点滅開始 mono={blink_state['start_mono']:.3f}")
            prev_mode = mode
            sets,elevs = show_direction(solenoid,guide_bearing,mode,heading,pinN)
            warn = f"警告: 磁気校正が不足 (mag={mag_cal}) " if mag_cal < 2 else ""
            v_min = speed.per_min()
            spd = f" 分速:{v_min:.0f}m/分" if v_min is not None else ""
            logger(LOG, f"{warn}現在地: {pos[0]:.6f},{pos[1]:.6f} (edge={curIndex}) 距離: {distance:.1f}m "
                        f"hAcc:{fix['hacc_m']:.1f}m 相対角:{(guide_bearing - heading)%360:.1f} 磁気校正値:{mag_cal}{spd} {elevs}")

            logger(HEADING_LOG,f"{(guide_bearing - heading)%360:.1f}, {heading:.1f}, {guide_bearing:.1f}],")
            guide_bearing = guide_bearing if guide_bearing is not None else float("nan")
            logger(COORDS_LOG, f"{pos[0]},{pos[1]},{fix['hacc_m']},{(guide_bearing - heading)%360:.1f}, {heading:.1f}, {guide_bearing:.1f}],", "indention")

        detector.radius_m = arrival_radius_m(v_mps)
        if detector.update(pos):
            logger(LOG, "目的地に到着しました", "end")
            break

        time.sleep(max(0.0, POLL_INTERVAL_SEC - (time.monotonic() - t0)))


if __name__ == "__main__":
    args = parse_args()
    pinN = int(args.pin_mode)
    arrival_wave = ARRIVAL_WAVE_4 if pinN == 4 else ARRIVAL_WAVE
    reroute_wave = REROUTE_WAVE_4 if pinN == 4 else REROUTE_WAVE
    logger(LOG_DIR,f"ピンモード: {args.pin_mode}（{pinN}ピン）")
    solenoid=Display()
    try:
        main(solenoid,pinN)
        solenoid.set(arrival_wave, "wave",0.5)
        time.sleep(60)
    finally:
        print("")
        solenoid.set(set(),"solid")

