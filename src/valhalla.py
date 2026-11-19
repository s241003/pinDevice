import os
import requests
import polyline
import folium
import datetime
from turn_map import extract_corners
from main import check_route_shape, cumulative_route_distance

DESTINATION = [35.889887,139.970766]
current= [35.889274,139.968819]
SNAP_RADIUS_M=50
TIMEOUT_SEC = 10
COSTING = "pedestrian"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(BASE_DIR, "../Maps", "routes")

VALHALLA_ROUTE_URL = "http://localhost:8002/route"
CARTO_KEY = "cb1_44j7_1_420580e3a4f5727c0fe3bb6e"
MAP_TILE="voyager"


def fetch_route(cur, dest, save_map=True):
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
        print("valhalla:不正経路を取得")
        raise ValueError(f"Valhallaの経路が使えない: {reason}")

    print(coords)

    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    distances = cumulative_route_distance(coords)

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
            "corners": corners}


if __name__=="__main__":
	route = fetch_route(current,DESTINATION) 
