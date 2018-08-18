"""
A* algorithm

Bootstraped by a python implementation under MIT License from:
Christian Careaga (christian.careaga7@gmail.com)

Available at:
http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/

Documented and fixed using the pseudocode in A* Wikipedia page:
https://en.wikipedia.org/wiki/A_star

And augmented to support:
- Python 3
- Non-binary occupation grids
- Manhattan distance
- Zig-zag suppression when using only N, S, E and W directions
plan_for_obstacle
TODO : PARAMETERIZE DIZ SHIT AND MOVE CONSTANTS OUT OF THE WAY

By:
Benoit Renault (benoit.renault@inria.fr)
"""

import numpy
import copy
from heapq import *

from utils import Utils

class GlobalPlanner:

    def __init__(self):
        pass

    def make_plan(self, occupancy_grid, start_pose, end_pose, resolution, frame_id):
        start_grid_coords = Utils.map_coord_from_ros_pose(start_pose, resolution)

        end_grid_coords = Utils.map_coord_from_ros_pose(end_pose, resolution)

        # Execute A*
        astar_path = self.astar(occupancy_grid, start_grid_coords, end_grid_coords, restrictTo4Neighbors = False)

        # Convert A* output to standard ROS path
        ros_path = Utils.ros_path_from_map_path(astar_path, start_pose, end_pose, resolution, frame_id)

        return ros_path


    def _dist_between(self, a, b):
        return self._manhattan_distance(a, b)


    def _heuristic_cost_estimate(self, a, b):
        return self._manhattan_distance(a, b)


    def _euclidean_distance(self, a, b):
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


    def _manhattan_distance(self, a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])


    def _reconstruct_path(self, came_from, end):
        total_path = [end]
        current = end
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path


    def print_path(self, nmap, path):
        matrix = copy.deepcopy(nmap)
        for point in path:
            matrix[point[0]][point[1]] = -1
        print(matrix)


    def astar(self, map_2d, start, goal, restrictTo4Neighbors = True):
        # Acceptable transitions from current grid element to neighbors
        if restrictTo4Neighbors:
            NEIGHBORHOOD = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        else:
            NEIGHBORHOOD = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        # Directions
        DIRECTIONS = [['NW', 'N', 'NE'],
                      ['W' , 'X', 'E' ],
                      ['SW', 'S', 'SE']]

        # The set of nodes already evaluated
        close_set = set()

        # The dictionary that remembers for each node, which node it can most efficiently be reached from.
        # If a node can be reached from many nodes, cameFrom will eventually contain the
        # most efficient previous step.
        came_from = {}
        came_from_direction = {}

        # The dictionary that remembers for each node, the cost of getting from the start node to that node.
        # The cost of going from start to start is zero.
        gscore = {start:0}

        # The dictionary that remembers for each node, the total cost of getting from the start node to the goal
        # by passing by that node. That value is partly known, partly heuristic.
        fscore = {start:self._heuristic_cost_estimate(start, goal)}

        # The set of currently discovered nodes that are not evaluated yet.
        open_heap = []
        # Initially, only the start node is known.
        heappush(open_heap, (fscore[start], start))

        # While open_heap is not empty == While there are discovered nodes that have not been evaluated
        while open_heap:

            # The node in open_heap having the lowest fScore[] value
            current = heappop(open_heap)[1]

            # Exit early if goal is reached
            if current == goal:
                return self._reconstruct_path(came_from, current)

            close_set.add(current)

            # For each neighbor of current node in the defined NEIGHBORHOOD
            for i, j in NEIGHBORHOOD:
                neighbor = current[0] + i, current[1] + j
                new_direction = DIRECTIONS[1 + i][1 + j]

                # Check that neighbor exists within the map
                if 0 <= neighbor[0] < map_2d.shape[0]:
                    if 0 <= neighbor[1] < map_2d.shape[1]:
                        # TODO This is here that the cost of traversing a node can be used
                        # Do not consider traversing neighbor if it is an obstacle
                        if map_2d[neighbor[0]][neighbor[1]] >= Utils.ROS_COST_POSSIBLY_CIRCUMSCRIBED:
                            continue
                        # Consider but apply cost
                        elif map_2d[neighbor[0]][neighbor[1]] >= Utils.ROS_COST_POSSIBLY_NONFREE:
                            extra_cost_ratio = (1.0 + float(map_2d[neighbor[0]][neighbor[1]]) /
                                                float(Utils.ROS_COST_POSSIBLY_CIRCUMSCRIBED - 1))
                            cost_between_current_and_neighbor = self._dist_between(current, neighbor) * extra_cost_ratio
                        # Consider without cost
                        else:
                            cost_between_current_and_neighbor = self._dist_between(current, neighbor)
                    else:
                        # Neighbor is outside of map in y axis
                        continue
                else:
                    # Neighbor is outside of map in x axis
                    continue

                if restrictTo4Neighbors:
                    try:
                        previous_direction = came_from_direction[current]
                    except KeyError:
                        previous_direction = new_direction
                    rotation_cost = (1.5 if new_direction != previous_direction else 0.0)
                    cost_between_current_and_neighbor = cost_between_current_and_neighbor + rotation_cost

                # The cost from start to a neighbor.
                tentative_g_score = gscore[current] + cost_between_current_and_neighbor

                if neighbor in close_set:
                    continue # Ignore the neighbor which is already evaluated.

                # Discover a new node or update info about known one :
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_heap]:
                    # This path is the best until now. Record it!
                    came_from[neighbor] = current
                    came_from_direction[neighbor] = new_direction
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self._heuristic_cost_estimate(neighbor, goal)
                    heappush(open_heap, (fscore[neighbor], neighbor))

        return []
