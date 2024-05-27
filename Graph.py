import Queue
import numpy as np
import yaml

from Vec2 import Vec2

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
    def __init__(self, map_matrix_file, map_yaml_file):
        self.vertices = []
        self.adjacencies = []
        self.map_matrix = np.genfromtxt(map_matrix_file, delimiter=",")
        with open(map_yaml_file, 'r') as file:
            map_yaml = yaml.safe_load(file)
            self.map_width = map_yaml["width"]
            self.map_height = map_yaml["height"]
            self.map_resolution = map_yaml["resolution"]
            self.map_origin = Vec2(map_yaml["origin"][0], map_yaml["origin"][1])
        
    def insert_vertex(self, vertex, num_neighbours):
        vertex_pixel = self.world_to_pixel(vertex)
        if self.map_matrix[int(vertex_pixel.y)][int(vertex_pixel.x)] == 1:
            return False

        vertex_idx = len(self.vertices)
        self.vertices.append(vertex)
        self.adjacencies.append([])

        if len(self.adjacencies) == 1:
            return True

        distances = []
        for i in range(len(self.vertices) - 1):
            v = self.vertices[i]
            distances.append((i, v.distance_to(vertex)))
        distances.sort(key = lambda x: x[1])

        for i in range(min(num_neighbours, len(distances))):
            if self.valid_adjacency(vertex, self.vertices[distances[i][0]]):
                # Undirected graph so adjacent vertices must also have the new vertex added
                self.adjacencies[vertex_idx].append(distances[i][0])
                self.adjacencies[distances[i][0]].append(vertex_idx)

        return True
    
    def remove_vertex(self, vertex):
        vertex_index = self.vertices.index(vertex)
        self.vertices.remove(vertex)

        # remove the vertex from the adjacency list of the other vertices
        for adj in self.adjacencies:
            if vertex_index in adj:
                adj.remove(vertex_index)
        self.adjacencies.pop(vertex_index)

    def valid_adjacency(self, start, end):
        t = 0
        while t < 1:
            line_point = Vec2(
                (end.x - start.x) * t + start.x,
                (end.y - start.y) * t + start.y
            )
            line_pixel_point = self.world_to_pixel(line_point)

            left_bound = int(line_pixel_point.x) - 10
            right_bound = int(line_pixel_point.x) + 10

            top_bound = int(line_pixel_point.y) - 10
            bottom_bound = int(line_pixel_point.y) + 10

            for x in range(left_bound, right_bound + 1, 1):
                for y in range(top_bound, bottom_bound + 1, 1):
                    if self.map_width <= x or x < 0 or self.map_height <= y or y < 0:
                        continue
                    
                    if self.map_matrix[y][x] == 1:
                        return False

            t += 0.01

        return True

    def world_to_pixel(self, world_vector):
        return Vec2(
            (world_vector.x - self.map_origin.x) / self.map_resolution,
            (world_vector.y - self.map_origin.y) / self.map_resolution
        )

    def pixel_to_world(self, pixel_vector):
        return Vec2(
            pixel_vector.x * self.map_resolution + self.map_origin.x,
            pixel_vector.y * self.map_resolution + self.map_origin.y,
        )

    """
        A* Path Finding 
            * Assumes that the start and goal elements are already apart of the graph
    """
    def a_star(self, start_idx, goal_idx):

        goal_position = self.vertices[goal_idx]

        frontier = Queue.PriorityQueue()
        explored = [None] * len(self.vertices)
        in_frontier = [0] * len(self.vertices)

        frontier.put(QueueItem(start_idx, -1, self.vertices[start_idx].distance_to(goal_position), 0))
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
                    frontier.put(QueueItem(child, current.item_idx, self.vertices[child].distance_to(goal_position), current.g + self.vertices[child].distance_to(self.vertices[current.item_idx])))
                    size_frontier += 1
                    in_frontier[child] = 1

        return []
