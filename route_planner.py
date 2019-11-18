"""
A * Search

References:
    Data Structures & Algorithms Nanodegree Program - 4. Advanced Algorithms
    Manhattan distance - https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
    https://en.wikipedia.org/wiki/A*_search_algorithm
"""

import heapq

class Map:
    def __init__(self, intersections=None, roads=None):
        self.intersections = intersections
        self.roads = roads

class Location:
    """
    Location is the object that stores information of each location nodes.
    """
    def __init__(self, value):
        self.value = value
        self.parent = None
        self.h = 0
        self.g = 0

    @property
    def f(self):
        return self.h + self.g



def heuristic(map, current, target):
    """
    The manhattan distance is the heuristic function used to calculate h, the estimated distance

    :param map:
    :param current:
    :param target:
    :return:
    """
    return float((abs(float(map.intersections[current][0])-float(map.intersections[target][0]))
         + abs(float(map.intersections[current][1]) - float(map.intersections[target][1]))))


def a_star_path(current):
    """
    Reconstruct the path based on the current node and its parents

    :param current:
    :return:
    """
    path = []
    while current:
        path.append(current.value)
        current = current.parent
    path.reverse()
    return path



def shortest_path(M, start, goal):

    # THe structure to hold all Location nodes
    locations = {}

    # Initialize the start and goal nodes
    locations[start] = Location(start)
    locations[goal] = Location(goal)

    # Initialize data structures to keep track of explored, unexplored locations
    # And the min_heap is used to allow for log(n) search of the next minimum cost node
    unexplored, explored, min_heap = set(), set(), []
    unexplored.add(start)

    heapq.heappush(min_heap, (locations[start].g, locations[start].value))

    locations[start].h = heuristic(M, start, goal)

    while unexplored:

        # Extract the next minimum cost node
        (current_f, current) = heapq.heappop(min_heap)

        if current == goal:
            return a_star_path(locations[goal])

        unexplored.remove(current)
        explored.add(current)

        # Expand search to neighboring nodes, the frontier
        for frontier in M.roads[current]:
            if frontier in explored:
                continue

            locations[frontier] = locations.get(frontier, Location(frontier))

            tentative_g = locations[current].g + heuristic(M, current, frontier)

            # Check if the tentative_g is better than the prev. g
            ## or the frontier is not-yet in unexplored
            if (tentative_g < locations[frontier].g) or (frontier not in unexplored):

                locations[frontier].parent = locations[current]

                locations[frontier].g = tentative_g
                locations[frontier].h = heuristic(M, frontier, goal)

                heapq.heappush(min_heap, (locations[frontier].f, frontier))
                unexplored.add(frontier)

