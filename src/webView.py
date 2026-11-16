"""main.py の値をブラウザでリアルタイム表示する小さなWebサーバー (Flask)。

使い方 (main.py 側):
    import webview
    webview.start(port=8000)                     # 起動時に1回
    webview.set_route(route["coords"])           # 経路が決まった/変わったとき
    webview.update(lat=..., lon=..., heading=...) # 値が更新されるたびに (何度でも)

ブラウザで  http://<ラズパイのIP>:8000  を開く。
"""
import logging
import threading
import time

from flask import Flask, Response, jsonify

_lock = threading.Lock()
_state = {}
_route = []
_route_ver = 0

app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.ERROR)     # アクセスログでターミナルが埋まらないように


def update(**values):
    """表示する値を更新する。キーは自由 (下のPAGEのFIELDSに載せたものが表示される)"""
    with _lock:
        _state.update(values)
        _state["ts"] = time.time()


def set_route(coords):
    """経路 [(lat, lon), ...] を地図に表示する"""
    global _route, _route_ver
    with _lock:
        _route = [[float(a), float(b)] for a, b in coords]
        _route_ver += 1


@app.route("/api/state")
def api_state():
    with _lock:
        s = dict(_state)
        s["route_ver"] = _route_ver
    s["age"] = (time.time() - s["ts"]) if "ts" in s else None     # 最後の更新からの経過秒
    return jsonify(s)


@app.route("/api/route")
def api_route():
    with _lock:
        return jsonify({"coords": _route})


@app.route("/")
def index():
    return Response(PAGE, mimetype="text/html")


def start(host="0.0.0.0", port=8000):
    """バックグラウンドのスレッドでサーバーを起動する (main.py の処理は止まらない)"""
    t = threading.Thread(
        target=lambda: app.run(host=host, port=port, threaded=True, use_reloader=False),
        daemon=True,
    )
    t.start()


PAGE = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Raspberry Pi 5 Live</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
  body { margin: 0; font-family: system-ui, sans-serif; }
  #map { height: 55vh; }
  #bar { padding: 6px 10px; font-size: 14px; }
  #vals { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 8px; padding: 10px; }
  .card { border: 1px solid #8886; border-radius: 8px; padding: 8px; }
  .k { font-size: 12px; opacity: .7; }
  .v { font-size: 22px; font-variant-numeric: tabular-nums; }
  .stale { color: #c00; }
</style>
</head>
<body>
<div id="map"></div>
<div id="bar"><label><input type="checkbox" id="follow" checked> 現在地を追従</label></div>
<div id="vals"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
// [キー, 表示名, 小数点以下の桁数]
const FIELDS = [
  ["lat", "緯度", 6], ["lon", "経度", 6], ["hacc", "GNSS精度 (m)", 1],
  ["heading", "Heading (°)", 0], ["mag_cal", "磁気校正 (0-3)", 0],
  ["solid_deg", "実線の方位 (°)", 0], ["blink_deg", "点滅の方位 (°)", 0],
  ["state", "状態", null], ["remaining_m", "次のコーナーまで (m)", 1],
  ["edge", "エッジ番号", 0], ["route_dist", "経路との距離 (m)", 1],
  ["age", "最終更新から (秒)", 1],
];

const map = L.map("map").setView([35.0, 139.0], 17);
L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19, attribution: "&copy; OpenStreetMap contributors",
}).addTo(map);

const valsEl = document.getElementById("vals");
const cells = {};
for (const [key, label] of FIELDS) {
  const card = document.createElement("div");
  card.className = "card";
  const k = document.createElement("div"); k.className = "k"; k.textContent = label;
  const v = document.createElement("div"); v.className = "v"; v.textContent = "-";
  card.append(k, v);
  valsEl.appendChild(card);
  cells[key] = v;
}

let marker = null, line = null, routeVer = -1, first = true;

function arrowIcon(deg) {
  const d = (deg === null || deg === undefined) ? 0 : deg;
  return L.divIcon({
    className: "",
    html: '<div style="transform:rotate(' + d + 'deg);font-size:28px;line-height:28px;">▲</div>',
    iconSize: [28, 28], iconAnchor: [14, 14],
  });
}

async function tick() {
  try {
    const s = await (await fetch("/api/state")).json();
    for (const [key, , digits] of FIELDS) {
      const v = s[key];
      let text = "-";
      if (v !== null && v !== undefined) {
        text = (typeof v === "number" && digits !== null) ? v.toFixed(digits) : String(v);
      }
      cells[key].textContent = text;
    }
    cells.age.classList.toggle("stale", s.age !== null && s.age > 3);   // 3秒以上更新がなければ赤

    if (s.lat !== undefined && s.lat !== null) {
      const ll = [s.lat, s.lon];
      if (!marker) marker = L.marker(ll, { icon: arrowIcon(s.heading) }).addTo(map);
      marker.setLatLng(ll);
      marker.setIcon(arrowIcon(s.heading));
      if (first) { map.setView(ll, 18); first = false; }
      else if (document.getElementById("follow").checked) map.panTo(ll);
    }
    if (s.route_ver !== routeVer) {                       // 経路が変わったときだけ取得し直す
      routeVer = s.route_ver;
      const r = await (await fetch("/api/route")).json();
      if (line) map.removeLayer(line);
      if (r.coords.length) line = L.polyline(r.coords, { color: "red", weight: 4 }).addTo(map);
    }
  } catch (e) {
    cells.age.textContent = "接続エラー";
    cells.age.classList.add("stale");
  }
}
setInterval(tick, 1000);
tick();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    # 動作確認用: ダミーの値を流す  (python webview.py → http://localhost:8000)
    import math
    start()
    set_route([(35.0000, 139.0000), (35.0005, 139.0005), (35.0010, 139.0005)])
    i = 0
    while True:
        i += 1
        update(lat=35.0 + 0.00002 * i, lon=139.0 + 0.00002 * i, hacc=2.5,
               heading=(i * 5) % 360, mag_cal=3, solid_deg=45, blink_deg=None,
               state="straight", remaining_m=max(0.0, 100 - i), edge=0, route_dist=1.2)
        time.sleep(1)