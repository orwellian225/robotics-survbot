import Queue

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
    def __init__(self, vertices, adjacencies):
        self.vertices = vertices
        self.adjacencies = adjacencies
        
    def insert_vertex(self, vertex):
        node_distances = []
        for node in self.vertices:
            node_distances.append(node.distance_to(vertex))
        adjacent_index = node_distances.index(min(node_distances))

        vertex_idx = len(self.vertices)
        self.vertices.append(vertex)
        # Undirected graph so adjacent vertices must also have the new vertex added
        self.adjacencies.append([adjacent_index])

        self.adjacencies[adjacent_index].append(vertex_idx)
    
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
            print("Frontier Size", size_frontier)
            if size_frontier >= len(self.vertices) * 1e4:
                return []

            current = frontier.get()
            size_frontier -= 1
            in_frontier[current.item_idx] = 0
            
            if explored[current.item_idx] is None:
                explored[current.item_idx] = current

            print("Current {} | Children {}".format(current.item_idx, self.adjacencies[current.item_idx]))
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
