import os
import osmnx as ox
import geopandas as gpd
import pandas as pd
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from shapely.geometry import Point
import contextily as cx
from scipy.spatial import cKDTree

# -----------------------------
# 1) OSMグラフの準備
# -----------------------------
area="yashioCity"
netWorkType="drive"
GMLpath=f"./graphml/{area}_{netWorkType}.graphml"

if (os.path.isfile(GMLpath)):
    G = ox.load_graphml(GMLpath)
    print("グラフ読込成功。")

else:
    place = "Yashio, Saitama, Japan"
    gdf_place = ox.geocode_to_gdf(place)
    polygon = gdf_place.loc[0, "geometry"]
    G = ox.graph_from_polygon(polygon, network_type=netWorkType, simplify=False)
    ox.save_graphml(G, GMLpath)
    print(f"{GMLpath[10:]} を保存。")

G_proj = ox.project_graph(G)
nodes_gdf, edges_gdf = ox.graph_to_gdfs(G_proj)

# -----------------------------
# 2) ユーザーの GPS データを GeoDataFrame に変換
# -----------------------------
df_1user={"lon":[139.840969, 139.844846],
          "lat":[35.800967, 35.807813]}

gdf_user = gpd.GeoDataFrame(
    df_1user,
    geometry=gpd.points_from_xy(df_1user["lon"], df_1user["lat"]),
    crs="EPSG:4326"
)
# CRSをグラフに合わせる（距離計算用）
gdf_user = gdf_user.to_crs(G_proj.graph['crs'])

# -----------------------------
# 3) KD-tree で最寄り交差点（次数>=3）を取得
# -----------------------------
deg_series = pd.Series(dict(G_proj.degree()))
nodes_gdf["degree"] = nodes_gdf.index.map(deg_series)

cand_nodes = nodes_gdf[nodes_gdf["degree"] >= 3].copy()
coords = np.vstack((cand_nodes["x"].values, cand_nodes["y"].values)).T


tree = cKDTree(coords)

pt_coords = np.vstack((gdf_user.geometry.x, gdf_user.geometry.y)).T
dists, idxs = tree.query(pt_coords, k=1)
gdf_user["nearest_node"] = cand_nodes.index.values[idxs]
gdf_user["snap_dist_m"] = dists

# -----------------------------
# 4) 経路復元（shortest path）／連続重複除去
# -----------------------------
node_seq = gdf_user["nearest_node"].tolist()
node_seq = [node_seq[i] for i in range(len(node_seq)) if i == 0 or node_seq[i] != node_seq[i-1]]

full_path_nodes = []
for a, b in zip(node_seq[:-1], node_seq[1:]):
    try:
        sp = nx.shortest_path(G_proj, a, b, weight="length")
    except nx.NetworkXNoPath:
        sp = [a, b]
    if full_path_nodes:
        full_path_nodes.extend(sp[1:])
    else:
        full_path_nodes.extend(sp)

# -----------------------------
# 5) 可視化（GPS vs マップマッチング）
# -----------------------------
path_x = [G_proj.nodes[n]['x'] for n in full_path_nodes]
path_y = [G_proj.nodes[n]['y'] for n in full_path_nodes]

fig, ax = plt.subplots(figsize=(10, 10))

#背景地図の描画
xmin, ymin, xmax, ymax = edges_gdf.total_bounds
#print(f"xmin: {xmin}, xmax: {xmax}, ymin: {ymin}, ymax: {ymax}")
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
cx.add_basemap(ax, crs=G_proj.graph['crs'], source=cx.providers.CartoDB.Positron)

#道路ネットワーク・道のり描画
ox.plot_graph(G_proj, ax=ax, show=False, close=False, node_size=0, edge_color="lightgray")
ax.plot(path_x, path_y, color='red', linewidth=2, alpha=0.7, label='Path')

ax.legend(fontsize=12)
plt.title("Map-Matched Path")
plt.savefig("./image/Yashio.png", format="png")
plt.show()
