import Queue
# import queue
import numpy as np
import numpy.linalg as npl
import cv2
import csv
import yaml

def distance_to(v1, v2):
    return npl.norm(v1 - v2)

class QueueItem:
    # h: float
    # g: float
    # origin_idx: int
    # item_idx: int

    def __init__(self, item_idx, origin_idx, h, g):
        self.item_idx = item_idx
        self.origin_idx = origin_idx
        self.h = h
        self.g = g

    def __eq__(self, o):
        return self.item_idx == o.item_idx

    def __lt__(self, o):
        return self.h + self.g < o.h + o.g

    def __cmp__(self, o):
        return cmp(self.h + self.g, o.h + o.g)

    def __hash__(self):
        return hash(self.item_idx)

    def __str__(self):
        return "Parent {0} -> Point {1} | {2} | {3} | {4}".format(self.origin_idx, self.item_idx, self.g + self.h, self.g, self.h)

class Graph:
    def __init__(self, graph_filepath, map_yaml_filepath, map_filepath):
        self.map = cv2.imread(map_filepath, cv2.IMREAD_GRAYSCALE)

        with open(map_yaml_filepath, 'r') as file:
            map_yaml = yaml.safe_load(file)
            self.map_width = self.map.shape[0]
            self.map_height = self.map.shape[1]
            map_resolution = map_yaml["resolution"]
            map_origin = np.array([map_yaml["origin"][0], map_yaml["origin"][1]])

            self.pixel_to_world = np.array([
                [0, map_resolution, -map_origin[0] ],
                [-map_resolution, 0, self.map_height * map_resolution + map_origin[1] ],
                [0, 0, 1 ]
            ])
            self.world_to_pixel = npl.inv(self.pixel_to_world)

        with open(graph_filepath, 'r') as f:
            csvr = csv.reader(f, delimiter=',', )
            self.vertices = []
            self.adjacencies = []

            for row in csvr:
                # Skip the first line of column titles
                if csvr.line_num == 1:
                    continue

                self.vertices.append([ float(row[1]), float(row[2]) ])
                self.adjacencies.append([])
                adjacencies = list(row[3])
                adjacencies.pop() # remove last char ]
                adjacencies.pop(0) #remove first char [
                adjacencies = "".join(adjacencies)

                for val in adjacencies.split():
                    self.adjacencies[-1].append(int(val))

                self.adjacencies[-1] = np.array(self.adjacencies[-1])

            self.vertices = np.array(self.vertices)

    def insert_vertex(self, vertex):
        """
            insert a new vertex into the graph
            return values:
                0 => successfully added
                -1 => invalid point
                -2 => no connection to graph exists
        """
        pixel_point = np.matmul(self.world_to_pixel, np.append(vertex, [1])[:,None]).T.astype(np.uint32)[0]
        if self.map[pixel_point[0], pixel_point[1]] == 0:
            return -1

        direction_vectors = np.array([])
        for x in self.vertices:
            direction_vectors = np.append(direction_vectors, x - vertex)
        direction_vectors = np.reshape(direction_vectors, (len(self.vertices), 2))
        distances = npl.norm(direction_vectors, axis=1)
        closest_vertex = np.ravel(np.argmin(distances))[0]
        invalid_edge_filter = np.ones(len(self.vertices))

        while not self.is_valid_edge(vertex, self.vertices[closest_vertex], 5):
            invalid_edge_filter[closest_vertex] = 0
            closest_vertex = np.ravel(np.argmin(distances[invalid_edge_filter]))[0]

            if invalid_edge_filter.all() == False:
                return -2

        self.vertices = np.append(self.vertices, [vertex], axis=0)
        self.adjacencies.append(np.array([closest_vertex]))
        self.adjacencies[closest_vertex] = np.append(self.adjacencies[closest_vertex], [len(self.vertices) - 1]).astype(np.uint32)
        return 0

    def remove_vertex(self, vertex):
        """
            Remove a vertex from the adjacency list
            You must remove in reverse order of insertion because reasons
                * It messes with the indexing of the adjacency array, they no longer line up
        """

        filter = (self.vertices != vertex).any(axis=1)
        index = np.where(np.invert(filter))[0][0]
        self.vertices = self.vertices[filter]
        for i in self.adjacencies[index]:
            remove_adj_filter = (self.adjacencies[i] != index)
            self.adjacencies[i] = self.adjacencies[i][remove_adj_filter]
        self.adjacencies.pop(index)

    def a_star(self, start_idx, goal_idx):
        """
            A* Path Finding 
                * Assumes that the start and goal elements are already apart of the graph
        """

        goal_position = self.vertices[goal_idx]

        frontier = Queue.PriorityQueue()
        explored = [None] * len(self.vertices)
        in_frontier = [0] * len(self.vertices)

        frontier.put(QueueItem(start_idx, -1, npl.norm(self.vertices[start_idx] - goal_position), 0))
        in_frontier[start_idx] = 1
        size_frontier = 1

        while not frontier.empty():
            if size_frontier >= len(self.vertices) * 1e4:
                return []

            current = frontier.get()
            size_frontier -= 1
            in_frontier[current.item_idx] = 0
            
            if explored[current.item_idx] is None:
                explored[current.item_idx] = current

            for child in self.adjacencies[current.item_idx]:  
                if child == goal_idx:
                    path = [self.vertices[child]]

                    backtrace = current
                    while backtrace.item_idx != start_idx:
                        path.append(self.vertices[backtrace.item_idx])
                        backtrace = explored[backtrace.origin_idx]

                    path.reverse()
                    return path

                if explored[child] is None and in_frontier[child] == 0:
                    frontier.put(QueueItem(child, current.item_idx, npl.norm(self.vertices[child] - goal_position), current.g + npl.norm(self.vertices[child] - self.vertices[current.item_idx])))
                    size_frontier += 1
                    in_frontier[child] = 1

        return []

    # def world_to_pixel(self, world_vector):
    #     return result

    # def pixel_to_world(self, pixel_vector):
    #     return self.map_resolution * pixel_vector - (self.map_resolution * np.array([ self.map_height, 0 ]) - self.map_origin)

    def is_valid_edge(self, v1, v2, extent_around_x):
        t = 0
        result = True
        while t < 1.01:
            t += 0.01
            line_t = (v2 - v1) * t + v1

            # check surrouding x-pixels as well
            for i in range(-extent_around_x, extent_around_x + 1):
                # inverted dims
                if line_t[1] < 0 or line_t[1] >= self.map_width or line_t[0] + i < 0 or line_t[0] + i >= self.map_height:
                    continue

                # if the map value is 255 on at least one point on the pixel, it'll become poisoned to false
                result = result and self.map[int(line_t[0]) + i, int(line_t[1])] == 255

        return result
