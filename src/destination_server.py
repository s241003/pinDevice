#!/usr/bin/env python3
# destination_server.py — Pi 側の目的地サーバ（GASなし）
#
# 役割は2つだけ：
#   1. スマホに index.html（音声入力のページ）を配る
#   2. スマホから送られた目的地（緯度経度）を受け取り、destination.json に保存する
#
# 標準ライブラリだけで動く。Flask などの追加インストールは要らない。
# スマホと Pi は同じ Wi-Fi にいることが条件。Pi をインターネットに公開しないこと。
#
# 起動（Pi 上で、main.py と同じフォルダに3ファイルを置く）：
#   python3 destination_server.py
#   → http://<PiのIP>:8000/ をスマホで開く。IPは `hostname -I` で確かめる。
#      `http://raspberrypi.local:8000/` でも開けることが多い。
#
# 環境変数：
#   DESTINATION_PORT  ポート番号（既定 8000）
#   DESTINATION_FILE  保存先（既定：このファイルと同じフォルダの destination.json）
#   DESTINATION_TOKEN  設定すると、POST 時にヘッダ X-Destination-Token が必要になる。
#                      設定しなければトークンなし（LAN内だけの実験用）。
#
# エンドポイント：
#   GET  /              index.html を返す
#   GET  /health        {"ok": true, "has_destination": true/false, ...}
#   GET  /destination   現在の目的地（なければ 404）
#   POST /destination   {"name","lat","lon",...} を受け取り保存する
#   DELETE /destination 目的地を消す（実験のリセット用）

import json
import math
import os
import sys
import tempfile
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PORT = int(os.environ.get("DESTINATION_PORT", "8000"))
DEST_FILE = os.environ.get(
    "DESTINATION_FILE", os.path.join(BASE_DIR, "destination.json")
)
TOKEN = os.environ.get("DESTINATION_TOKEN", "")
INDEX_FILE = os.path.join(BASE_DIR, "index.html")
LOG_FILE = os.path.join(BASE_DIR, "destination.log")

MAX_BODY_BYTES = 64 * 1024  # 64KB。これ以上は受け取らない

# 日本のだいたいの範囲。ここから外れたら入力ミスの可能性が高いので 400 で返す。
LAT_MIN, LAT_MAX = 24.0, 46.0
LON_MIN, LON_MAX = 122.0, 154.0


def log(msg):
    line = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
    print(line, flush=True)
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except OSError:
        pass


def validate(data):
    """POST された目的地を確かめる。使えるなら (lat, lon, name) を返す。
    使えないなら ValueError（理由付き）を出す。"""
    if not isinstance(data, dict):
        raise ValueError("JSON の辞書で送ってください")
    lat, lon, name = data.get("lat"), data.get("lon"), data.get("name")
    if lat is None or lon is None:
        raise ValueError("lat と lon が必要です")
    if isinstance(lat, bool) or isinstance(lon, bool):
        raise ValueError("lat / lon は数値でなければなりません")
    try:
        lat, lon = float(lat), float(lon)
    except (TypeError, ValueError):
        raise ValueError("lat / lon は数値でなければなりません")
    if not (math.isfinite(lat) and math.isfinite(lon)):
        raise ValueError("lat / lon が有限の数ではありません")
    if lat == 0.0 and lon == 0.0:
        raise ValueError("lat=0, lon=0 は無効です")
    if not (LAT_MIN <= lat <= LAT_MAX and LON_MIN <= lon <= LON_MAX):
        raise ValueError(f"緯度経度が日本の範囲外です: lat={lat}, lon={lon}")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("name（場所の名前）が必要です")
    if len(name) > 200:
        raise ValueError("name が長すぎます（200文字まで）")
    return lat, lon, name.strip()


def save_destination(data):
    """destination.json に原子的に保存する（書きかけのファイルを読ませないため、
    一時ファイルに書いてから os.replace で置き換える）。古いものは .prev に残す。"""
    lat, lon, name = validate(data)
    dest = {
        "name": name,
        "lat": lat,
        "lon": lon,
        "display_name": str(data.get("display_name") or "")[:500],
        "query": str(data.get("query") or "")[:200],
        "place_id": str(data.get("place_id") or "")[:200],
        "updated_at": datetime.now(timezone.utc)
        .astimezone()
        .isoformat(timespec="seconds"),
    }
    if os.path.exists(DEST_FILE):
        try:
            os.replace(DEST_FILE, DEST_FILE + ".prev")
        except OSError:
            pass
    fd, tmp = tempfile.mkstemp(dir=BASE_DIR, prefix=".destination-", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(dest, f, ensure_ascii=False, indent=2)
            f.write("\n")
        os.replace(tmp, DEST_FILE)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return dest


def load_current():
    if not os.path.exists(DEST_FILE):
        return None
    with open(DEST_FILE, encoding="utf-8") as f:
        return json.load(f)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):  # 既定のアクセスログは出さない（log() にまとめる）
        pass

    def _send_json(self, status, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _check_token(self):
        if not TOKEN:
            return True
        got = self.headers.get("X-Destination-Token", "")
        return got == TOKEN

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            if not os.path.exists(INDEX_FILE):
                self._send_json(500, {"error": "index.html がありません"})
                return
            with open(INDEX_FILE, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/health":
            cur = None
            try:
                cur = load_current()
            except (OSError, ValueError):
                cur = None
            self._send_json(200, {"ok": True, "has_destination": cur is not None,
                                  "destination": cur})
            return
        if self.path == "/destination":
            try:
                cur = load_current()
            except (OSError, ValueError) as e:
                self._send_json(500, {"error": f"目的地ファイルが読めません: {e}"})
                return
            if cur is None:
                self._send_json(404, {"error": "目的地はまだ設定されていません"})
                return
            self._send_json(200, cur)
            return
        self._send_json(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/destination":
            self._send_json(404, {"error": "not found"})
            return
        if not self._check_token():
            self._send_json(401, {"error": "トークンが違います"})
            return
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0 or length > MAX_BODY_BYTES:
            self._send_json(400, {"error": "送るデータの大きさがおかしいです"})
            return
        try:
            data = json.loads(self.rfile.read(length).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            self._send_json(400, {"error": "JSON として読めません"})
            return
        try:
            dest = save_destination(data)
        except ValueError as e:
            self._send_json(400, {"error": str(e)})
            return
        except OSError as e:
            self._send_json(500, {"error": f"保存に失敗しました: {e}"})
            return
        log(f"目的地を受け取りました: {dest['name']} lat={dest['lat']} lon={dest['lon']}")
        self._send_json(200, {"ok": True, "destination": dest})

    def do_DELETE(self):
        if self.path != "/destination":
            self._send_json(404, {"error": "not found"})
            return
        if not self._check_token():
            self._send_json(401, {"error": "トークンが違います"})
            return
        try:
            os.unlink(DEST_FILE)
        except FileNotFoundError:
            pass
        except OSError as e:
            self._send_json(500, {"error": f"削除に失敗しました: {e}"})
            return
        log("目的地を消しました")
        self._send_json(200, {"ok": True})


def main():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    log(f"目的地サーバを開始しました。ポート {PORT}")
    log(f"スマホで開くURL: http://<PiのIP>:{PORT}/  （`hostname -I` でIPを確認）")
    log(f"または http://raspberrypi.local:{PORT}/")
    log(f"保存先: {DEST_FILE}")
    log("止めるときは Ctrl+C")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("目的地サーバを止めました")
        sys.exit(0)


if __name__ == "__main__":
    main()
