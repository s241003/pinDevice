"""逸脱を考慮した誘導：経路上の「少し先の点」を目標にする (lookahead)。

これまでの main は、現在地に最も近い経路エッジの方位をそのまま
目標方位にしていた。そのため、経路から横にずれていても
「経路と平行にまっすぐ進め」という指示しか出ず、ずれが直らなかった
（2026-11-14 の実験で、13〜17m ずれていても相対角がほぼ 0°＝直進に
なっていたことが coords ログで確認できた）。

このモジュールは次の3ステップで目標方位を決める。

  1. 現在地を経路へ射影し、最も近い経路上の点 Q を求める
  2. Q から経路沿いに LOOKAHEAD_M だけ進んだ点を目標点 T とする
     （経路の残りが LOOKAHEAD_M より短ければ、目標点は目的地そのもの）
  3. 現在地 → T の方位を目標方位として返す

経路の上にいれば T は真っすぐ先にあり、いままでと同じ「直進」になる。
横にずれていれば T は経路側へ斜め前にあり、目標方位が経路側へ傾くので、
歩くほど経路へ戻る。傾きの大きさは atan(横ずれ / LOOKAHEAD_M) で決まる。

LOOKAHEAD_M = 10m は提案値（根拠分類 B）：
  - 横ずれ 5m で約27°（約1セクタ分）、10m で 45°、20m（再ルート閾値）で
    約63° の修正角になる、という幾何からの導出
  - 実験時の hAcc は平均 約2m で、10m はその5倍。目標点が GNSS の
    ばらつきで暴れにくい長さでもある
  - 到着半径 15m 以下なので、最後は目的地そのものを狙う形になる
現場での歩行テストで妥当性を確認すること。
"""

import numpy as np
from pyproj import Transformer

from vincenty import vincenty_inverse

LOOKAHEAD_M = 10.0

_to_m = Transformer.from_crs("EPSG:4326", "EPSG:6677", always_xy=True)


def _latlon_to_xy(latlon):
    """(緯度, 経度) のリスト → メートル座標の (N,2) 配列（main の latlon_to_xy と同じ）"""
    lat = np.array([p[0] for p in latlon])
    lon = np.array([p[1] for p in latlon])
    x, y = _to_m.transform(lon, lat)
    return np.column_stack([x, y])


def project_to_route(coords, pos):
    """現在地に最も近い経路上の点を求める。

    戻り値: (エッジ番号, t, 横ずれ[m])
      t はそのエッジの始点から終点への割合 (0..1)。
    """
    P = _latlon_to_xy(coords)
    p = _latlon_to_xy([pos])[0]
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = (AB ** 2).sum(axis=1)
    t = ((p - A) * AB).sum(axis=1) / np.where(L2 > 0, L2, 1.0)
    t = np.clip(t, 0.0, 1.0)
    d = np.linalg.norm(A + t[:, None] * AB - p, axis=1)
    i = int(np.argmin(d))
    return i, float(t[i]), float(d[i])


def point_at_distance(coords, distances, s):
    """経路の累積距離 s[m] にある点 (lat, lon) を、頂点間の直線補間で返す。"""
    if s <= 0:
        return coords[0]
    if s >= distances[-1]:
        return coords[-1]
    for i in range(len(coords) - 1):
        if distances[i] <= s <= distances[i + 1]:
            seg = distances[i + 1] - distances[i]
            f = 0.0 if seg <= 0 else (s - distances[i]) / seg
            lat = coords[i][0] + (coords[i + 1][0] - coords[i][0]) * f
            lon = coords[i][1] + (coords[i + 1][1] - coords[i][1]) * f
            return (lat, lon)
    return coords[-1]


def target_bearing(coords, distances, pos, lookahead_m=LOOKAHEAD_M):
    """目標方位 [deg] と横ずれ [m] を返す。

    目標点が現在地とほぼ一致する場合（目的地に着いている等）は
    方位が定まらないので (None, 横ずれ) を返す。呼び出し側は
    None のとき従来のエッジ方位にフォールバックすること。
    """
    i, t, cross = project_to_route(coords, pos)
    s_q = distances[i] + t * (distances[i + 1] - distances[i])
    tp = point_at_distance(coords, distances, s_q + lookahead_m)
    v = vincenty_inverse([pos[0], pos[1]], [tp[0], tp[1]])
    if v is None or v["distance"] < 0.5:
        return None, cross
    return v["azimuth1"], cross
