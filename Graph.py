import queue

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

    def __hash__(self):
        return hash(self.item_idx)

    def __str__(self):
        return "Parent {0} -> Point {1} | {2} | {3} | {4}".format(self.origin_idx, self.item_idx, self.g + self.h, self.g, self.h)


class Graph:
    def __init__(self, vertices, adjacencies):
        self.vertices = vertices
        self.adjacencies = adjacencies

    """
        A* Path Finding 
            * Assumes that the start and goal elements are already apart of the graph
    """
    def a_star(self, start_idx, goal_idx):

        goal_position = self.vertices[goal_idx]

        frontier = queue.PriorityQueue()
        frontier.put(QueueItem(start_idx, -1, self.vertices[start_idx].distance_to(goal_position)))
        size_frontier = 1
        explored = None * len(self.vertices)
        
        while not frontier.empty():
            if size_frontier >= len(self.vertices) * 1e4:
                return []

            current = frontier.get()
            size_frontier -= 1

            if explored[current.item_idx] is None:
                explored[current.item_idx] = current
            elif (explored[current.item_idx].g + explored[current.item_idx].h) > (current.g + current.h):
                explored[current.item_idx] = current

            for child in self.adjacencies[current.item_idx]:
                if child == goal_position:
                    path = [child]

                    backtrace = current
                    while backtrace.item_idx != start_idx:
                        path.append(backtrace.item_idx)
                        backtrace = explored[backtrace.origin_idx]

                    path.append(start_idx)
                    path.reverse()
                    return path

                frontier.put(QueueItem(child, current.item_idx, self.vertices[child].distance_to(goal_position), current.g + self.vertices[child].distance_to(self.vertices[current.item_idx])))
                size_frontier += 1

        return []