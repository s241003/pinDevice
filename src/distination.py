import os
import math
from datetime import datetime
import osmnx as ox
import networkx as nx
import geopandas as gpd
import matplotlib.pyplot as plt
import contextily as cx
import numpy as np
from pyproj import Transformer
from shapely.geometry import LineString, Point


# -----------------------------
#  OSMグラフの準備
# -----------------------------

area="kashiwanoha"
netWorkType="walk"
type="point"
offDistanceThr = 30
offAngleThr = 15
searchWindow = 3    # 前後何エッジ探すか
transformer = Transformer.from_crs("EPSG:4326", "EPSG:6683", always_xy=True)

def latlon_to_xy(lat, lon):
    x, y = transformer.transform(lon, lat)
    return x, y

def build_route_geometry(G, route_edges):
    """route_edges: [(u, v), (u, v), ...] のノードIDリスト"""
    geoms = []
    for u, v in route_edges:
        u_lat, u_lon = G.nodes[u]['y'], G.nodes[u]['x']
        v_lat, v_lon = G.nodes[v]['y'], G.nodes[v]['x']
        ux, uy = latlon_to_xy(u_lat, u_lon)
        vx, vy = latlon_to_xy(v_lat, v_lon)
        geoms.append(LineString([(ux, uy), (vx, vy)]))
    return geoms

def perpendicular_distance(px, py, line: LineString) -> float:
    # 点(px,py)から線分lineまでの最短距離[m]
    return line.distance(Point(px, py))

def find_nearest_edge_on_route(route_geoms, current_idx, lat, lon, window=searchWindow):
    """
    前回のインデックス付近(前後window個)だけを線形探索し、
    最も距離が近いエッジのインデックスと距離を返す。
    """
    px, py = latlon_to_xy(lat, lon)
    lo = max(0, current_idx - window)
    hi = min(len(route_geoms), current_idx + window + 1)

    best_idx = current_idx
    best_dist = float("inf")
    for i in range(lo, hi):
        d = perpendicular_distance(px, py, route_geoms[i])
        if d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx, best_dist

def update_position(lat, lon, heading, current_idx):
    idx, dist = find_nearest_edge_on_route(route_geoms, current_idx, lat, lon)
    if (dist > offDistanceThr):
        return "REROUTE_NEEDED", idx, dist
    return "ON_ROUTE", idx, dist


def init(lat, lon, dest_lat, dest_lon):
    global G_proj, nodes_gdf, edges_gdf, signals

    if type=="address":
        GMLpath=f"../graphml/{area}_{netWorkType}.graphml"
    elif type=="point":
        distance = ox.distance.great_circle(lat, lon, dest_lat, dest_lon, earth_radius=6371009)
        radius = distance/2
        print(f"radius:{radius}")
        GMLpath=f"../graphml/{radius}_{netWorkType}.graphml"

    if (os.path.isfile(GMLpath)):
        G = ox.load_graphml(GMLpath)
        print("グラフ読込成功。")

    else:
        if type=="address":
            place = "Kashiwa, Chiba, Japan"
            gdf_place = ox.geocode_to_gdf(place)
            polygon = gdf_place.loc[0, "geometry"]
            G = ox.graph_from_polygon(polygon, network_type=netWorkType)

        elif type=="point":
            centY=(dest_lat+lat)/2
            centX=(dest_lon+lon)/2
            print(centX,centY)
            G = ox.graph_from_point((centY,centX),dist=radius, network_type=netWorkType)
            ox.save_graphml(G, GMLpath)
            print(f"{GMLpath[10:]} を保存。")


    G_proj = ox.project_graph(G)
    nodes_gdf, edges_gdf = ox.graph_to_gdfs(G_proj)
    signals = nodes_gdf[nodes_gdf["highway"] == "traffic_signals"]
    print(signals[["y", "x"]])  # 緯度・経度

def plotRouteMap(route):     # 地図の描画

    path_x = [G_proj.nodes[n]['x'] for n in route]
    path_y = [G_proj.nodes[n]['y'] for n in route]

    fig, ax = plt.subplots(figsize=(10, 10))

    #背景地図の描画
    xmin, ymin, xmax, ymax = edges_gdf.total_bounds
    #print(f"xmin: {xmin}, xmax: {xmax}, ymin: {ymin}, ymax: {ymax}")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    cx.add_basemap(ax, crs=G_proj.graph['crs'])

    #道路ネットワーク・道のり描画
    ox.plot_graph(G_proj, ax=ax, show=False, close=False, node_size=0, edge_color="lightgray")
    ax.plot(path_x, path_y, color='red', linewidth=2, alpha=0.7, label='Path')
    ax.scatter(signals["x"], signals["y"],c="red", s=25, zorder=5,edgecolor="black", linewidth=0.4,label=f"信号機 (n={len(signals)})")

    ax.legend(fontsize=12)
    plt.title(f"Map in {area} by {netWorkType}")
    dt =datetime.now()
    time = dt.strftime("%Y%m%d%H%M%S")
    plt.savefig(f"../image/{time}_{area}_{netWorkType}.png", format="png")
    plt.show()

def getDistAngle(cur_lon,cur_lat,dest_lon,dest_lat,isPlot):

    gdf_points = gpd.GeoDataFrame(
        {"name": ["current", "destination"],
        "lon": [cur_lon, dest_lon],
        "lat": [cur_lat, dest_lat]},
        geometry=gpd.points_from_xy([cur_lon, dest_lon],
                                    [cur_lat, dest_lat]),
        crs="EPSG:4326"
    ).to_crs(G_proj.graph["crs"])


    # -----------------------------
    # 最寄りノードを取得
    # -----------------------------

    current_node = ox.nearest_nodes(G_proj,
                                    X=gdf_points.geometry.x[0],
                                    Y=gdf_points.geometry.y[0])

    dest_node = ox.nearest_nodes(G_proj,
                                X=gdf_points.geometry.x[1],
                                Y=gdf_points.geometry.y[1])


    # -----------------------------
    # 最短経路を計算
    # -----------------------------

    route = nx.shortest_path(G_proj,
                            source=current_node,
                            target=dest_node,
                            weight="length")
    print(f"route: {route}")
    route_edges = list(zip(route[:-1], route[1:]))

    curRoute = nx.shortest_path(G_proj,
                            source=route[0],
                            target=route[1],
                            weight="length")
    routeRad= math.atan2(route[0], route[1])
    routeDegree = math.degrees(routeRad)

    if (isPlot):
        plotRouteMap(route)

    return routeDegree
    # print(f"route: {curRoute}\nrad: {routeRad}, degree: {routeDegree}°")

def update_position(lat, lon, heading):
    global current_edge_idx
    # (a) 現在地に最も近い経路上のエッジを探す(全体探索ではなく現在idx付近だけ)
    idx = find_nearest_edge_on_route(route_edges, current_edge_idx, lat, lon, search_window=3)
    
    # (b) 逸脱判定: 最近傍エッジまでの垂直距離が閾値超なら外れている
    dist_to_route = perpendicular_distance(lat, lon, route_edges[idx])
    if dist_to_route > OFFROUTE_THRESHOLD_M:  # 例: 30m
        return "REROUTE_NEEDED"
    
    current_edge_idx = idx
    return "ON_ROUTE", idx
current = {"lon":139.944807,"lat":35.891873 }
dist = {"lon":139.952411, "lat": 35.893922}

# initilize
init(current["lat"],current["lon"],dest["lat"],dest["lon"])

getDistAngle(current["lon"],current["lat"],dist["lon"],dist["lat"],True)

