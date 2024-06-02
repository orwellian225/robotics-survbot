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

graph.insert_vertex(start)
graph.insert_vertex(goal)

path = graph.a_star(len(graph.vertices) - 2, len(graph.vertices) - 1)

pois = np.array([
    graph.world_to_pixel(start),
    graph.world_to_pixel(goal)
])
print((start, goal), pois)
plt.scatter(pois[:,1], pois[:,0], s=0.5, color='green', zorder=2)
plt.scatter(graph.world_to_pixel(np.array([0,0]))[1], graph.world_to_pixel(np.array([0,0]))[0], s=0.5, color='red', zorder=2)

pixel_path = []
for x in path:
    pixel_path.append(graph.world_to_pixel(x))

print(np.array(path))
for i in range(len(path) - 1):
    line = np.array([pixel_path[i][1], pixel_path[i+1][1], pixel_path[i][0], pixel_path[i+1][0]])
    plt.plot(line[:2], line[2:4], linewidth=0.3, color="blue", zorder=0)

pixel_path = np.array(pixel_path)
plt.scatter(pixel_path[:, 1], pixel_path[:, 0], s=0.5, color='blue', zorder=2)

# plt.savefig('../test/map_graph_overlay.pdf')
plt.show()
