import matplotlib.pyplot as plt
import numpy as np
import cv2

from Vec2 import Vec2
from Graph import Graph

graph = Graph('../maps/graph_data.csv', '../maps/map2.yaml', '../maps/map_data.png')

# # Show the graph-map overlay
map_image = cv2.imread('../maps/map2.pgm', cv2.IMREAD_GRAYSCALE)
plt.imshow(map_image, cmap='gray')
plt.axis('off')

start = np.array([ -0.5, 0. ])
goal = np.array([ -3.0, -3.0 ])
goal_2 = np.array([-3.1, -3])

pixel_vertices = []
for x in graph.vertices:
    pixel_vertices.append(graph.world_to_pixel(x))
pixel_vertices = np.array(pixel_vertices)

for i in range(len(graph.adjacencies) - 2):
    for n in graph.adjacencies[i]:
        #inverted because row-col is y-x
        line = np.array([pixel_vertices[i][1], pixel_vertices[n][1], pixel_vertices[i][0], pixel_vertices[n][0]])
        plt.plot(line[:2], line[2:4], linewidth=0.5, color="pink", zorder=0)
plt.scatter(pixel_vertices[:, 1], pixel_vertices[:, 0], s=1, color='orange', zorder=1)


pois = np.array([
    graph.world_to_pixel(start),
    graph.world_to_pixel(goal),
    graph.world_to_pixel(goal_2)
])
plt.scatter(pois[:,1], pois[:,0], s=0.5, color='green', zorder=2)
plt.scatter(graph.world_to_pixel(np.array([0,0]))[1], graph.world_to_pixel(np.array([0,0]))[0], s=0.5, color='red', zorder=2)

graph.insert_vertex(start)
graph.insert_vertex(goal)

path_1 = graph.a_star(len(graph.vertices) - 2, len(graph.vertices) - 1)

graph.remove_vertex(goal)
graph.remove_vertex(start)

pixel_path_1 = []
for x in path_1:
    pixel_path_1.append(graph.world_to_pixel(x))

pixel_path_1 = np.array(pixel_path_1)
if len(path_1) != 0:
    for i in range(len(path_1) - 1):
        line = np.array([pixel_path_1[i][1], pixel_path_1[i+1][1], pixel_path_1[i][0], pixel_path_1[i+1][0]])
        plt.plot(line[:2], line[2:4], linewidth=0.3, color="blue", zorder=0)

    pixel_path = np.array(pixel_path_1)
    plt.scatter(pixel_path_1[:, 1], pixel_path_1[:, 0], s=0.5, color='blue', zorder=2)

graph.insert_vertex(goal)
graph.insert_vertex(goal_2)

path_2 = graph.a_star(len(graph.vertices) - 2, len(graph.vertices) - 1)

graph.remove_vertex(goal_2)
graph.remove_vertex(goal)

pixel_path_2 = []
for x in path_2:
    pixel_path_2.append(graph.world_to_pixel(x))
pixel_path_2 = np.array(pixel_path_2)

if len(path_2) != 0:
    for i in range(len(path_2) - 1):
        line = np.array([pixel_path_2[i][1], pixel_path_2[i+1][1], pixel_path_2[i][0], pixel_path_2[i+1][0]])
        plt.plot(line[:2], line[2:4], linewidth=0.3, color="purple", zorder=0)

    pixel_path = np.array(pixel_path_2)
    plt.scatter(pixel_path_2[:, 1], pixel_path_2[:, 0], s=0.5, color='purple', zorder=2)

# plt.savefig('../test/map_graph_overlay.pdf')
plt.show()
