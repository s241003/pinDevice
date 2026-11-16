import datetime
import folium
import numpy as np
import os
from vincenty import vincenty_inverse

CORNER_THRESHOLD_DEG = 60



# ---------------------------------------------------------------
# パラメータ
# ---------------------------------------------------------------
    # 平面直角座標系IX系 (関東)。地域に合わせて変更する
STEP_M = 1.0             # 再サンプリング間隔 [m]
SMOOTH_K = 5             # 方位の平滑化に使う点数
RATE_THR = 2.0           # 方位の変化率がこれ [deg/m] を超えた所を「曲がり区間」とみなす
MERGE_GAP = 4            # この点数以内に近接する曲がり区間は1つにまとめる
CORNER_MAX_R = 10.0      # 平均曲率半径がこれ [m] 以下ならコーナー、超えればカーブ
SHOW_POINTS = True       # 元の点を小さな円で表示する
CARTO_KEY = "cb1_44j7_1_420580e3a4f5727c0fe3bb6e"
MAP_TILE="voyager"
MIN_TURN_DEG = 90        # 総曲がり角がこれ未満なら「直線」扱い (小さな屈曲を含む)                   #
COLORS = {"直線": "#1f77b4", "コーナー": "#d62728", "カーブ": "#ff7f0e"}

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def save_corner_map(coords,corners):

    center = coords[len(coords)//2]

    m = folium.Map(location=center,
                    tiles=f"https://{{s}}.basemaps.cartocdn.com/rastertiles/{MAP_TILE}/{{z}}/{{x}}/{{y}}{{r}}.png?key={CARTO_KEY}",
                    attr='&copy; OpenStreetMap contributors &copy; CARTO',
                    zoom_start=18)
    folium.PolyLine(coords,color="blue",weight=4).add_to(m)

    for c in corners:
        folium.CircleMarker([c["lat"], c["lon"]],radius=8,color="red",fill=True,
            popup=(
                f"idx={c['corner_idx']} "
                f"{c['direction']} "
                f"{c['angle_deg']:.1f}deg"
            )
        ).add_to(m)
        map_dir = os.path.join(BASE_DIR, "Maps", "turn")
        os.makedirs(map_dir, exist_ok=True)         # フォルダが無いと保存に失敗し、経路取得を延々やり直してしまう
        dt_now = datetime.datetime.now()
        t = dt_now.strftime('%m%d%H%M')
        m.save(os.path.join(map_dir, f"{t}corner_map.html"))

def bearing(p1, p2):
    v = vincenty_inverse(p1,p2)
    return v["azimuth1"]

def angle_diff(a, b):
    d = (a - b + 180) % 360 - 180
    return d

def extract_corners(coords,dists,save_map=False):    #core
    corners = []
    if len(coords) < 3:
        return corners

    for i in range(0, len(coords) - 1):

        if i == 0:
            continue
        b1 = bearing(coords[i-1], coords[i])
        b2 = bearing(coords[i], coords[i+1])

        turn_deg = angle_diff(b1, b2)

        if abs(turn_deg) < CORNER_THRESHOLD_DEG:
            continue

        corners.append({
            "corner_idx": i,
            "lat": coords[i][0],
            "lon": coords[i][1],
            "angle_deg": turn_deg,
            "cumul_dist_m":dists[i],
            "direction": ("左" if turn_deg > 0 else "右")})
        if save_map:
            save_corner_map(coords,corners)
    return corners
