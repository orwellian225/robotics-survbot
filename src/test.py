import matplotlib.pyplot as plt
import numpy as np
import cv2

from Vec2 import Vec2
from Graph import Graph

graph = Graph('../maps/graph_data.csv', '../maps/map2.yaml', '../maps/map_data.png')


# Show the graph-map overlay
map_image = cv2.imread('../maps/map2.pgm', cv2.IMREAD_GRAYSCALE)
plt.imshow(map_image, cmap='gray')
plt.axis('off')

vertices_x = []
vertices_y = []
for i in range(len(graph.vertices)):
    pixel = graph.world_to_pixel(graph.vertices[i])
    vertices_x.append(pixel.x)
    vertices_y.append(pixel.y)

for i in range(len(graph.adjacencies)):
    for n in graph.adjacencies[i]:
        #inverted because row-col is y-x
        line = np.array([vertices_y[i], vertices_y[n], vertices_x[i], vertices_x[n]])
        plt.plot(line[:2], line[2:4], linewidth=0.5, color="pink", zorder=0)
#inverted because row-col is y-x
plt.scatter(vertices_y, vertices_x, s=2, color='orange', zorder=1)

origin_world = Vec2(0,0)
start = Vec2(0.1, -0.1)
goal = Vec2(-3.0, 3.0)

print(origin_world, graph.world_to_pixel(origin_world))
print(start, graph.world_to_pixel(start))
print(goal, graph.world_to_pixel(goal))
poi_x = [
    graph.world_to_pixel(start).x,
    graph.world_to_pixel(goal).x
]
poi_y = [
    graph.world_to_pixel(start).y,
    graph.world_to_pixel(goal).y
]
plt.scatter(poi_y, poi_x, s=1, zorder=3, color='green')
plt.scatter(
    [graph.world_to_pixel(origin_world).y],
    [graph.world_to_pixel(origin_world).x],
    c='r', s=1)

plt.scatter(
    [graph.world_to_pixel(graph.map_origin).y],
    [graph.world_to_pixel(graph.map_origin).x],
    c='purple', s=1)

graph.insert_vertex(start)
graph.insert_vertex(goal)

path = graph.a_star(len(graph.vertices) - 2, len(graph.vertices) - 1)

print([ str(graph.vertices[x]) for x in path ])

path_vx = [ graph.world_to_pixel(start).x ]
path_vy = [ graph.world_to_pixel(start).y ]
for idx in path:
    path_pixel = graph.world_to_pixel(graph.vertices[idx])
    path_vx.append(path_pixel.x)
    path_vy.append(path_pixel.y)

for i in range(len(path) - 1):
    #inverted because row-col is y-x
    line = np.array([path_vy[i], path_vy[i + 1], path_vx[i], path_vx[i + 1]])
    plt.plot(line[:2], line[2:4], linewidth=0.3, color="blue", zorder=4)

plt.scatter(path_vy, path_vx, s=0.5, color='blue', zorder=2) 

plt.savefig("../test/map_graph_overlay.pdf")