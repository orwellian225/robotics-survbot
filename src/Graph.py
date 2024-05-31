import Queue
# import queue
import numpy as np
import cv2
import csv
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
    def __init__(self, graph_filepath, map_yaml_filepath, map_filepath):
        self.map = cv2.imread(map_filepath, cv2.IMREAD_GRAYSCALE)

        with open(map_yaml_filepath, 'r') as file:
            map_yaml = yaml.safe_load(file)
            self.map_width = self.map.shape[0]
            self.map_height = self.map.shape[1]
            self.map_resolution = map_yaml["resolution"]
            self.map_origin = Vec2(map_yaml["origin"][0], map_yaml["origin"][1])

        with open(graph_filepath, 'r') as f:
            csvr = csv.reader(f, delimiter=',', )
            self.vertices = []
            self.adjacencies = []

            for row in csvr:
                # Skip the first line of column titles
                if csvr.line_num == 1:
                    continue

                self.vertices.append(Vec2(float(row[1]), float(row[2])))
                self.adjacencies.append([])
                adjacencies = list(row[3])
                adjacencies.pop() # remove last char ]
                adjacencies.pop(0) #remove first char [
                adjacencies = "".join(adjacencies)

                for val in adjacencies.split():
                    self.adjacencies[-1].append(int(val))
        
    def insert_vertex(self, vertex):
        """
            insert a new vertex into the graph
            return values:
                0 => successfully added
                -1 => invalid point
                -2 => no connection to graph exists
        """
        # For reasons beyond me at this point (weird map transforms)
        # The inserted vector needs to be flipped x = -x, y = -y
        vertex.x *= -1
        vertex.y *= -1
        pixel_vector = self.world_to_pixel(vertex)

        if self.map[int(pixel_vector.y), int(pixel_vector.x)] == 0:
            return -1

        closest_vi = -1
        for i, v in enumerate(self.vertices):
            valid_edge = self.is_valid_edge(vertex, v, 5)
            if closest_vi == -1 and len(self.adjacencies[i]) != 0 and valid_edge:
                closest_vi = i
                continue 
            
            if len(self.adjacencies[i]) != 0 and vertex.distance_to(v) < vertex.distance_to(self.vertices[closest_vi]) and valid_edge:
                closest_vi = i

        if closest_vi == -1:
            return -2 

        self.vertices.append(vertex)
        self.adjacencies.append([ closest_vi ])
        self.adjacencies[closest_vi].append( len(self.vertices) - 1 )

        return 0
    
    def remove_vertex(self, vertex):
        vertex_index = self.vertices.index(vertex)
        self.vertices.remove(vertex)

        # remove the vertex from the adjacency list of the other vertices
        for adj in self.adjacencies:
            if vertex_index in adj:
                adj.remove(vertex_index)
        self.adjacencies.pop(vertex_index)

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

    def world_to_pixel(self, world_vector):
        return Vec2(
            self.map_width - 1 - (world_vector.x - self.map_origin.x) / self.map_resolution,
            (world_vector.y - self.map_origin.y) / self.map_resolution
        )

    def pixel_to_world(self, pixel_vector):
        return Vec2(
            (self.map_width - 1 - pixel_vector.x) * self.map_resolution + self.map_origin.x,
            pixel_vector.y * self.map_resolution + self.map_origin.y ,
        )

    def is_valid_edge(self, v1, v2, extent_around_x):
        t = 0
        result = True
        while t < 1.01:
            t += 0.01
            line_t = Vec2(
                (v2.x - v1.x) * t + v1.x,
                (v2.y - v1.y) * t + v1.y,
            )

            # check surrouding x-pixels as well
            for i in range(-extent_around_x, extent_around_x + 1):
                # inverted dims
                if line_t.y < 0 or line_t.y >= self.map_height or line_t.x + i < 0 or line_t.x + i >= self.map_width:
                    continue

                # if the map value is 255 on at least one point on the pixel, it'll become poisoned to false
                result = result and self.map[int(line_t.x) + i, int(line_t.y)] == 255

        return result