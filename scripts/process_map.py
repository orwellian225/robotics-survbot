from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import yaml
import cv2
import sys

def world_to_pixel(world_vector, resolution, world_pixel_origin, pixel_origin):
    return (world_vector - (-resolution * pixel_origin + world_pixel_origin)) / resolution

def pixel_to_world(pixel_vector, resolution, world_pixel_origin, pixel_origin):
    # return resolution * (pixel_vector + pixel_origin) - world_pixel_origin
    return resolution * pixel_vector + (-resolution * pixel_origin + world_pixel_origin)

def is_valid_edge(v1, v2, map, map_dims, extent_around_x):
    t = 0
    result = True
    while t < 1.01:
        t += 0.01
        line_t = (v2 - v1) * t + v1

        # check surrouding x-pixels as well
        for i in range(-extent_around_x, extent_around_x + 1):
            # inverted dims
            if line_t[1] < 0 or line_t[1] >= map_dims[1] or line_t[0] + i < 0 or line_t[0] + i >= map_dims[0]:
                continue

            # if the map value is 255 on at least one point on the pixel, it'll become poisoned to false
            result = result and map[int(line_t[0]) + i, int(line_t[1])] == 255

    return result

def main():
    if len(sys.argv) == 6:
        map_image_filepath = str(sys.argv[1])
        map_yaml_filepath = str(sys.argv[2])
        output_dir = str(sys.argv[3])
        graph_size = int(sys.argv[4])
        num_neighbours = int(sys.argv[5])
    else:
        map_image_filepath = str(input("Enter the filepath to the map image: "))
        map_yaml_filepath = str(input("Enter the filepath to the map yaml: "))
        output_dir = str(input("Enter the output directory: "))
        graph_size = int(input("Enter the number of vertices in the graph: "))
        num_neighbours = int(input("Enter the maximum number of neighbours for a vertex: "))

    map_image = cv2.imread(map_image_filepath, 0)
    with open(map_yaml_filepath, 'r') as file:
        map_yaml = yaml.safe_load(file)
        map_width = map_image.shape[0]
        map_height = map_image.shape[1]
        map_resolution = map_yaml["resolution"]
        map_origin = np.array([map_yaml["origin"][0], map_yaml["origin"][1]])

    # Save the image to a pdf
    plt.imshow(map_image, cmap='gray')
    plt.axis('off')
    plt.savefig(output_dir + "/map.pdf")

    # Erode the image
    t, thresh = cv2.threshold(map_image, 127, 255, cv2.THRESH_BINARY)
    kernel = np.ones((10,10), np.uint8)
    eroded_image = cv2.erode(thresh, kernel, iterations=1)

    # Save the eroded image
    plt.imshow(eroded_image, cmap='gray')
    plt.axis('off')
    plt.savefig(output_dir + "/eroded_map.pdf")

    # Generate the graph points

    sampled_points = np.column_stack((
        np.random.randint(0, map_width, graph_size * 5), # \hat{x} positions
        np.random.randint(0, map_height, graph_size * 5) # \hat{y} positions
    ))

    valid_filter = np.ones(graph_size * 5, dtype=bool)
    for i in range(graph_size):
        valid_filter[i] = eroded_image[sampled_points[i,0], sampled_points[i,1]] 
    all_vertices = sampled_points[valid_filter]

    # Save the eroded map data for testing point validity
    cv2.imwrite(output_dir + '/map_data.png', eroded_image)

    # Reduce the graph vertices to nearest centroids
    kmeans = KMeans(n_clusters=graph_size, init='k-means++', max_iter=1000)
    kmeans.fit(all_vertices)
    graph_vertices = kmeans.cluster_centers_

    # Generate the graph edges
    knn = NearestNeighbors(n_neighbors=num_neighbours + 1)
    knn.fit(graph_vertices)
    _, neigbours_mat = knn.kneighbors(graph_vertices)

    graph_adjacencies = []
    for i in range(len(neigbours_mat)):
        if i % 50 == 0:
            print(f"\r{i} / {graph_size} adjacencies generated", end="", flush=True)
        neighbours = neigbours_mat[i, 1:]
        adjacency_filter = np.ones(num_neighbours, dtype=bool)

        for j in range(len(neighbours)):
            # CHECK IF THE ADJACENCY IS VALID HERE
            adjacency_filter[j] = is_valid_edge(graph_vertices[i], graph_vertices[neighbours[j]], eroded_image, (map_width, map_height), 5) and neighbours[j] < len(graph_vertices)

        graph_adjacencies.append(neighbours[adjacency_filter])
    print(f"\rGenerated all vertex adjacencies {'': <20}")

    ## Should be removing any vertices without edges, but this fucks with the adjacency lists because the new indices 
    ## of points after the deletion are not updated in the lists. So to avoid this, when a vertex is added, into the robot, 
    ## it will add the vertex to the closest vertex with a non-zero degree
    # remove_filter = np.ones(len(graph_vertices), dtype=bool)
    # for i in range(len(graph_vertices)):
    #     if len(graph_adjacencies[i]) == 0:
    #         remove_filter[i] = 0

    # graph_vertices = graph_vertices[remove_filter]
    # graph_adjacencies = [x for i, x in enumerate(graph_adjacencies) if remove_filter[i] != 0]

    # Save the graph to a textfile
    with open(output_dir + "/graph_data.csv", 'w') as f:
        f.write(f"Vertex ID,world x,world y,adjacencies\n")
        for i in range(len(graph_vertices)):
            # print(f"{i},{world_vertex[0]},{world_vertex[1]},{graph_adjacencies[i]}\n")
            # def pixel_to_world(pixel_vector, resolution, world_pixel_origin, pixel_origin):
            world_vertex = pixel_to_world(graph_vertices[i], map_resolution, map_origin, np.array([map_width, 0]))
            f.write(f"{i},{world_vertex[0]},{world_vertex[1]},{graph_adjacencies[i]}\n")

    world_origin = world_to_pixel(np.array([0,0]), map_resolution, map_origin, np.array([ map_width , 0 ]))
    # Show the graph-map overlay
    plt.imshow(map_image, cmap='gray')
    for i in range(len(graph_adjacencies)):
        for n in graph_adjacencies[i]:
            #inverted because row-col is y-x
            line = np.array([graph_vertices[i][1], graph_vertices[n][1], graph_vertices[i][0], graph_vertices[n][0]])
            plt.plot(line[:2], line[2:4], linewidth=0.5, color="blue", zorder=0)
    #inverted because row-col is y-x
    plt.scatter(graph_vertices[:, 1], graph_vertices[:, 0], s=1, color='red', zorder=1)
    plt.scatter(world_origin[1], world_origin[0], s=1, color='green', zorder=1)
    plt.savefig(output_dir + "/map_graph_overlay.pdf")

    # Show the graph-eroded overlay
    plt.imshow(eroded_image, cmap='gray')
    for i in range(len(graph_adjacencies)):
        for n in graph_adjacencies[i]:
            #inverted because row-col is y-x
            line = np.array([graph_vertices[i][1], graph_vertices[n][1], graph_vertices[i][0], graph_vertices[n][0]])
            plt.plot(line[:2], line[2:4], linewidth=0.2, color="blue", zorder=0)
    #inverted because row-col is y-x
    # plt.scatter(all_vertices[:, 1], all_vertices[:, 0], s=1, color='green', zorder=1)
    plt.scatter(graph_vertices[:, 1], graph_vertices[:, 0], s=1, color='red', zorder=2)
    plt.savefig(output_dir + "/eroded_graph_overlay.pdf")

if __name__ == "__main__":
    main()
