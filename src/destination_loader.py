# destination_loader.py — destination.json を読むだけの小さな部品
# main.py から使う。標準ライブラリだけで、ハードウェアには触らない。
# 使い方:
#   from destination_loader import load_destination, wait_for_destination
#   dest = wait_for_destination("destination.json")  # ファイルができるまで待つ
#   lat, lon = dest["lat"], dest["lon"]
#
# destination.json の形:
#   {"name": "柏駅", "lat": 35.8617, "lon": 139.9773,
#    "display_name": "柏駅, ...", "query": "柏駅",
#    "updated_at": "2026-11-09T10:00:00+09:00"}

import json
import math
import os
import time

# 日本のだいたいの範囲。ここから外れた緯度経度は、入力ミスの可能性が高いので受け取らない。
LAT_MIN, LAT_MAX = 24.0, 46.0
LON_MIN, LON_MAX = 122.0, 154.0


def validate_destination(data):
    """辞書が目的地として使えるか確かめる。使えるなら (lat, lon, name) を返す。
    使えないなら ValueError を出す（理由をメッセージに入れる）。"""
    if not isinstance(data, dict):
        raise ValueError("目的地は JSON の辞書でなければなりません")
    lat = data.get("lat")
    lon = data.get("lon")
    name = data.get("name")
    if lat is None or lon is None:
        raise ValueError("lat と lon が必要です")
    # bool は int の仲間なので先にはじく（True が 1 にならないように）
    if isinstance(lat, bool) or isinstance(lon, bool):
        raise ValueError("lat / lon は数値でなければなりません")
    try:
        lat = float(lat)
        lon = float(lon)
    except (TypeError, ValueError):
        raise ValueError("lat / lon は数値でなければなりません")
    if not (math.isfinite(lat) and math.isfinite(lon)):
        raise ValueError("lat / lon が有限の数ではありません")
    if lat == 0.0 and lon == 0.0:
        raise ValueError("lat=0, lon=0 無効な目的地（未設定の可能性あり）")
    if not (LAT_MIN <= lat <= LAT_MAX and LON_MIN <= lon <= LON_MAX):
        raise ValueError(f"緯度経度が日本の範囲外です: lat={lat}, lon={lon}")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("name（場所の名前）が必要です")
    if len(name) > 200:
        raise ValueError("name が長すぎます（200文字まで）")
    return lat, lon, name.strip()


def load_destination(path, default=None):
    """destination.json を読んで辞書を返す。ファイルがなければ default を返す。
    ファイルはあるが中身が壊れていたら ValueError を出す（黙って古い目的地を使わないため）。"""
    if not os.path.exists(path):
        return default
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    validate_destination(data)
    return data


def wait_for_destination(path, poll_sec=2.0, timeout_sec=None, log=print):
    """destination.json ができて、中身が有効になるまで待つ。有効な辞書を返す。
    timeout_sec を超えたら TimeoutError を出す（None ならずっと待つ）。
    ファイルはあるが中身が壊れている間は、ログを出して待ち続ける（落ちない）。"""
    t0 = time.monotonic()
    warned_bad = False
    while True:
        try:
            dest = load_destination(path, default=None)
        except (ValueError, json.JSONDecodeError, OSError) as e:
            if not warned_bad:
                log(f"目的地ファイルが読めません（待ち続けます）: {e}")
                warned_bad = True
            dest = None
        if dest is not None:
            return dest
        if timeout_sec is not None and time.monotonic() - t0 > timeout_sec:
            raise TimeoutError(f"{timeout_sec}秒待っても目的地が設定されませんでした: {path}")
        log("目的地を待っています… スマホで目的地を設定してください")
        time.sleep(poll_sec)
