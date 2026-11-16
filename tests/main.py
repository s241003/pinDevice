import os
import time
import datetime
from math import radians, sin, cos, sqrt, atan2, degrees

import requests
import polyline
import folium
import numpy as np

from evaluateIsCorner import evaluateCorner
from turn_map import latlon_to_xy
from vincenty import vincenty_inverse
from cornerGuide import CornerGuide
from userDirection import HeadingReader
from decideSolenoids import whichSolenoid
import solenoid
from GNSS import GNSSReader


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ログのpath
LOG_DIR = os.path.join(BASE_DIR, "..", "logs")
os.makedirs(LOG_DIR, exist_ok=True)
LOG = os.path.join(LOG_DIR, "log.txt")
ROUTES_LOG = os.path.join(LOG_DIR, "routes.txt")
COORDS_LOG = os.path.join(LOG_DIR, "coords.txt")

VALHALLA_ROUTE_URL = "http://localhost:8002/route"
CARTO_KEY = "cb1_44j7_1_420580e3a4f5727c0fe3bb6e"
MAP_TILE="voyager"

"""
以下パラメータ ここをいじろう
"""

DESTINATION = [35.893922, 139.952411]
COSTING = "pedestrian"
REROUTE_THRESHOLD_M = 30      # 再ルート閾値(m)
POLL_INTERVAL_SEC = 1.0       # 更新間隔(s)  GNSS(1Hz)に合わせる。sleepは「残り時間」だけ待つ
TIMEOUT_SEC = 10
ARRIVAL_RADIUS_M = 30         # 到着とみなす距離(m)
ARRIVAL_CONFIRM_COUNT = 3     # 何回連続で範囲内なら到着とするか
WARN_DISTANCE = 15.0


class ArrivalDetector:
    def __init__(self, destination, radius_m=ARRIVAL_RADIUS_M, confirm=ARRIVAL_CONFIRM_COUNT):
        self.dest = destination
        self.radius_m = radius_m
        self.confirm = confirm
        self._count = 0
        self.arrived = False

    def update(self, p):
        """output: 到着したか判定(bool)"""
        if self.arrived:
            return True
        if haversine_m(p, self.dest) <= self.radius_m:
            self._count += 1
        else:
            self._count = 0
        if self._count >= self.confirm:
            self.arrived = True
        return self.arrived


def logger(filePath, content, mode="indentionW"):
    print(content)
    dt_now = datetime.datetime.now()
    t = dt_now.strftime('[%Y-%m-%d %H:%M:%S]    ')
    if filePath is COORDS_LOG:
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


def haversine_m(p1, p2):
    """2点(lat, lon)間の直線距離をメートルで返す"""
    R = 6371000
    lat1, lon1 = radians(p1[0]), radians(p1[1])
    lat2, lon2 = radians(p2[0]), radians(p2[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    return 2 * R * atan2(sqrt(a), sqrt(1 - a))

def fetch_route(cur, dest, save_map=False):
    body = {
        "locations": [
            {"lat": cur[0], "lon": cur[1]},
            {"lat": dest[0], "lon": dest[1]},
        ],
        "costing": COSTING,
        "costing_options": {"pedestrian": {"alley_factor": 3.0, "driveway_factor": 9.0}},
        "id": "reroute_demo",
    }
    resp = requests.post(VALHALLA_ROUTE_URL, json=body, timeout=TIMEOUT_SEC)
    resp.raise_for_status()
    leg = resp.json()["trip"]["legs"][0]
    coords = polyline.decode(leg["shape"], precision=6)
    logger(ROUTES_LOG, coords, "indention")

    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    distances = [haversine_m(coords[i], coords[i + 1]) for i in range(len(coords) - 1)]
    bearings = []
    for i in range(len(coords) - 1):
        v = vincenty_inverse(lats[i], lons[i], lats[i + 1], lons[i + 1])
        bearings.append(v["azimuth1"] if v else bearing_fallback(coords[i], coords[i + 1]))

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
        map_dir = os.path.join(BASE_DIR, "Maps", "routes")
        os.makedirs(map_dir, exist_ok=True)           # フォルダが無いと保存に失敗し、経路取得を延々やり直してしまう
        m.save(os.path.join(map_dir, "route_map.html"))

    # corners/turns は初回・再ルートどちらでも必要 (元コードは初回のみ計算していて再ルートで落ちた)
    corners, turns = evaluateCorner(coords)
    return {"coords": coords, "distances": distances, "corners": corners, "turns": turns, "bearings": bearings}


def bearing_fallback(p1, p2):
    # fetchでbearingがなかったら発動
    lat1, lon1, lat2, lon2 = map(radians, (p1[0], p1[1], p2[0], p2[1]))
    x = sin(lon2 - lon1) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1)
    return (degrees(atan2(x, y)) + 360) % 360

def find_edge(route_latlon, current, max_dist):
    """最傍エッジの番号と、その距離[m]を返す"""
    P = latlon_to_xy(route_latlon)
    p = latlon_to_xy([current])[0]
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = (AB ** 2).sum(axis=1)
    t = ((p - A) * AB).sum(axis=1) / np.where(L2 > 0, L2, 1.0)
    t = np.clip(t, 0.0, 1.0)
    d = np.linalg.norm(A + t[:, None] * AB - p, axis=1)
    i = int(np.argmin(d))
    return (i if d[i] <= max_dist else None), float(d[i])

def plotElevSolenoids(solid_set, blink_set):
    marks = ["①", "②", "③", "④", "⑤", "⑥"]
    for i in solid_set:
        marks[i - 1] = "●"
    for i in blink_set:
        if i not in solid_set:
            marks[i - 1] = "○"
    a, b, c, d, e, f = marks
    logger(LOG, f"\n[{a}] [{d}]\n[{b}] [{e}]\n[{c}] [{f}]", "indentionW")

def show_direction(solid_deg, blink_deg, heading):
    solid_set = whichSolenoid((solid_deg - heading) % 360)
    blink_set = whichSolenoid((blink_deg - heading) % 360) if blink_deg is not None else []
    solenoid.show(solid_set, blink_set)
    plotElevSolenoids(solid_set, blink_set)


def main():
    gnss = GNSSReader()
    head = HeadingReader()
    detector = ArrivalDetector(DESTINATION)
    route = guide = None
    solenoid.LowAll()
    logger(LOG, "開始", "init")

    while True:
        t0 = time.monotonic()
        fix = gnss.get(max_age=5.0)
        if fix is None:
            logger(LOG, "GNSS未取得...")
            time.sleep(POLL_INTERVAL_SEC)
            continue
        pos = [fix["lat"], fix["lon"]]

        # --- ルート取得 (初回) ---
        if route is None:
            logger(LOG, "初回ルート取得中...")
            try:
                route = fetch_route(pos, DESTINATION, True)
                guide = CornerGuide.from_latlon(route["coords"], route["turns"], warn_dist=WARN_DISTANCE)
            except Exception as e:
                logger(LOG, f"ルート取得失敗(Valhalla起動待ち?): {e}")
                route = None
                time.sleep(2)
                continue

        # --- 逸脱したら再ルート ---
        curIndex, distance = find_edge(route["coords"], pos, REROUTE_THRESHOLD_M)
        if curIndex is None:
            logger(LOG, f"{REROUTE_THRESHOLD_M}m以上逸脱したため再ルートします...")
            
            try:
                route = fetch_route(pos, DESTINATION, False)
                guide = CornerGuide.from_latlon(route["coords"], route["turns"], warn_dist=WARN_DISTANCE)
                curIndex, distance = find_edge(route["coords"], pos, REROUTE_THRESHOLD_M)
            except Exception as e:
                logger(LOG, f"再ルート失敗: {e}")

        # --- 案内 (毎周期 guide を更新する。元コードは再ルート時しか呼ばれず r が未定義になっていた) ---
        r = guide.update_latlon(pos)
        hd = head.get()
        if hd is None:
            logger(LOG, "Heading未取得")
        else:
            heading, mag_cal = hd
            if mag_cal < 2:
                logger(LOG, f"警告: 磁気キャリブレーション不足 (mag={mag_cal}) → 方位が不正確")
            # 接近中: 今の進行方向を点滅 + 曲がった先の方向を実線 / 通常: 進行方向を実線
            show_direction(r["solid_deg"], r["blink_deg"], heading)
            logger(LOG, f"現在地: {pos[0]:.6f},{pos[1]:.6f} (edge={curIndex}) 経路との距離: {distance:.1f}m "
                        f"hAcc:{fix['hacc_m']:.1f}m state:{r['state']} 残り:{r['remaining_m']} "
                        f"目標方位:{r['solid_deg']:.0f}° Heading:{heading:.0f}°")
            edge_bearing = route["bearings"][max(curIndex - 1, 0)] if curIndex is not None else float("nan")
            logger(COORDS_LOG, f"[{pos[0]},{pos[1]},{r['solid_deg']},{edge_bearing},{heading:.0f}],", "indention")

        if detector.update(pos):
            logger(LOG, "目的地に到着しました", "end")
            break

        time.sleep(max(0.0, POLL_INTERVAL_SEC - (time.monotonic() - t0)))


if __name__ == "__main__":
    try:
        main()
        solenoid.blink_all_for(60)      # 目的地についたら1分間点滅
    finally:
        solenoid.LowAll()