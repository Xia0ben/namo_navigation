#! /usr/bin/env python

import math
import numpy
import copy

from bresenham import bresenham

from pose import Pose

from sklearn.cluster import DBSCAN
from sklearn import metrics

class MultilayeredMap:
    LETHAL_COST_ROS = 254
    FREESPACE_COST_ROS = 0
    NEWPOINT_VALUE = -1

    DBSCAN_EPSILON = 10
    DBSCAN_MIN_SAMPLES = 1
    DBSCAN_ALGORITHM = 'auto'
    DBSCAN_METRIC = 'euclidean'
    DBSCAN_METRIC_PARAMS = None
    DBSCAN_LEAF_SIZE = 30
    DBSCAN_P = None
    N_JOBS = 1

    PSEUDO_INFLATION_FOOTPRINT = [[1, 1, 1],
                                  [1, 1, 1],
                                  [1, 1, 1]]

    def __init__(self, rosMap, robotDiameter):
        self.origin = rosMap.info.origin
        self.resolution = rosMap.info.resolution
        self.width = rosMap.info.width
        self.height = rosMap.info.height
        # Only contains static obstacles data (walls, static furniture, ...)
        # as defined in the map given to the map server
        self.static_occ_grid = numpy.array(rosMap.data).reshape(
            (self.width, self.height))
        self.static_obstacles_positions = _get_static_obstacles_positions()
        self.pseudo_inflated_static_occ_grid = _get_pseudo_inflated_static_occ_grid()

        # Only contains non-static (potentially movable) obstacles data (chairs, boxes, tables, ...)
        self.obstacle_occ_grid = numpy.zeros((self.width, self.height))
        self.obstacles = {}

        # Merged grid of static elements and obstacles, with inflation applied
        self.merged_occ_grid = copy.deepcopy(self.static_occ_grid)

        self.robot = Robot(robotDiameter, self.resolution)

    def update_laserscan(self, pointsWithDistance, robot_pose, obstacle_range, raytrace_range):
        updated_obstacle_occ_grid = copy.deepcopy(self.obstacle_occ_grid)
        updated_obstacles = copy.deepcopy(self.obstacles)

        ### Clearing ###
        ## Raytracing method ##
        # Use bresenham algorithm to trace lines in 2D:
        # (https://en.wikipedia.org/wiki/Bresenham's_line_algorithm)
        # list(bresenham) returns an array of tuples [(x,y), ...]
        pointsToClear = set()
        for point, distance in pointsWithDistance.items():
            if distance <= raytrace_range:
                line_points = list(
                    bresenham(robot_pose.pos.coords[0], # x1
                              robot_pose.pos.coords[1], # y1
                              point.coords[0],  # x2
                              point.coords[1])) # y2
                for point in line_points:
                    pointsToClear.add(point)

        # pointsToClear is a set of tuples set([(x,y), ...])
        new_cleared_points = []
        for point in pointsToClear:
            # TODO Maybe use a function that degrades over time, starting from
            # 127 (free space) going to 0, over 20 seconds ?
            # Would need to update inflation function to take this kind of case
            # into account.
            current_obstacle_id = self.updated_obstacle_occ_grid[point[0]][point[1]]
            if current_obstacle_id > 0:
                self.updated_obstacle_occ_grid[point[0]][point[1]] = MultilayeredMap.FREESPACE_COST_ROS
                updated_obstacles[current_obstacle_id].removePoin # FIXME FINISH THIS LINE
                new_cleared_points.append(SimplePosition(point[0], point[1]))

        ## "Conetracing" method ##
        # Draw polygons of successive clearable points of the laser_points list
        # And remove all registered points within them
        # TODO Implement this idea if appropriate

        ### Marking and Static Map Overlap Checking ###
        # Add points only if within marking distance, and that they are not in
        # a pseudo inflated static occupation spot
        for point, distance in pointsWithDistance.items():
            isInRange = (distance <= obstacle_range)
            isInMatrix = _is_in_matrix(point.coords[0], point.coords[1],
                                       self.width, self.height)
            isNotInPseudo = (self.pseudo_inflated_static_occ_grid[point.coords[0]][point.coords[1]] == 0)
            if (isInRange and isInMatrix and isNotInPseudo):
                self.updated_obstacle_occ_grid = MultilayeredMap.LETHAL_COST_ROS


        # Clusterize points into separate obstacles
        db = DBSCAN(eps=MultilayeredMap.DBSCAN_EPSILON,
            min_samples=MultilayeredMap.DBSCAN_MIN_SAMPLES,
            algorithm=MultilayeredMap.DBSCAN_ALGORITHM ,
            metric=MultilayeredMap.DBSCAN_METRIC,
            metric_params=MultilayeredMap.DBSCAN_METRIC_PARAMS,
            leaf_size=MultilayeredMap.DBSCAN_LEAF_SIZE,
            p=MultilayeredMap.DBSCAN_P,
            n_jobs=MultilayeredMap.N_JOBS).fit(npObstaclePoints)

        cluster_labels = db.labels_
        n_clusters = len(set(cluster_labels)) - (1 if -1 in cluster_labels else 0)

        # get the clusters
        # cluster_labels = -1 means outliers, we ignore it
        clusters = []
        for cluster_label in range(n_clusters):
            clusters.append([])

        for i in range(len(cluster_labels)):
            clusters[cluster_labels[i]].append(npObstaclePoints[i])

    def has_free_space_been_created(self):
        # FIXME return true if we moved an object (or if an object moved by
        # itself). Will require keeping track of calls to function. For
        # simulation, the simulator can directly provide data about whether an
        # obstacle has been moved or not, but for a real application, we will
        # need to evaluate if any of the points in the map integer coordinate
        # system have been moved for each obstacle.
        return True

    # def pushUsingGraspPointPossible(self, obstacle, graspPoint):
    # # FIXME first simulate a discrete push in d. Don't forget to update both obstacle.x and y !
    # # FIXME Check if graspPoint enters in collision with surrounding known space in the map.
    # pass

    def _get_static_obstacles_positions():
        for x in range(len(self.static_occ_grid)):
            for y in range(len(self.static_occ_grid[x])):
                if self.static_occ_grid[x][y] == MultilayeredMap.LETHAL_COST_ROS:
                    positions.append(2DPosition(x, y))
        return positions

    def _get_pseudo_inflated_static_occ_grid(self):
        return _get_inflastamped_matrix(self.static_obstacles_positions,
                                        MultilayeredMap.PSEUDO_INFLATION_FOOTPRINT,
                                        self.width,
                                        self.height)

    def _get_inflated_obstacle_grid(self, obstacle, footprint, removeObstaclePoints = False):
        inflation_width = len(footprint)
        inflation_heigth = len(footprint[0])

        matrixTopLeftX = ((obstacle.bbTopLeft.coords[0] - inflation_width)
            if (obstacle.bbTopLeft.coords[0] - inflation_width) < 0 else 0)
        matrixTopLeftY = ((obstacle.bbTopLeft.coords[1] - inflation_heigth)
            if (obstacle.bbTopLeft.coords[1] - inflation_heigth) < 0 else 0)
        matrixBottomRightX = ((obstacle.bbBottomRight.coords[0] + inflation_width)
            if (obstacle.bbBottomRight.coords[0] + inflation_width) >= self.width else self.width)
        matrixBottomRightY = ((obstacle.bbBottomRight.coords[0] + inflation_heigth)
            if (obstacle.bbBottomRight.coords[0] + inflation_heigth) >= self.height else self.height)

        width = matrixBottomRightX - matrixTopLeftX
        height = matrixBottomRightY - matrixTopLeftY

        return _get_inflastamped_matrix(obstacle.points,
                                        footprint,
                                        width,
                                        height,
                                        matrixTopLeftX,
                                        matrixTopLeftY,
                                        removeObstaclePoints)

    # Works best if footprint has odd width and height
    # Basically, you just stamp the inflation footprint on each obstacle point
    @staticmethod
    def _get_inflastamped_matrix(points, footprint, width, height, offsetX = 0, offsetY = 0, removeObstaclePoints = False):
        # TODO a funny way to implement even sized footprints would be to
        # add a column and/or line in the middle by copying one of the
        # columns/lines of the middle of the footprint

        matrix = numpy.zeros((width, height))

        fWidthLeft = len(footprint) / 2 if len(footprint) % 2 == 0 else (len(footprint) - 1) / 2
        fHeightLeft = len(footprint[0]) / 2 if len(footprint[0]) % 2 == 0 else (len(footprint[0]) - 1) / 2
        fWidthRight = len(footprint) / 2
        fHeightRight = len(footprint[0]) / 2

        # Inflate the obstacle <=> apply footprint at every obstacle point
        for obsPoint in obstacle_points:
            xInM = obsPoint[0] - offsetX
            yInM = obsPoint[1] - offsetY

            # Apply the footprint around the point
            xF, yF = 0, 0 # Current coordinates in footprint
            for xM in range(xInM - fWidthLeft, xInM + fWidthRight):
                for yM in range(yInM - fHeightLeft, yInM + fHeightRight):
                    if _is_in_matrix(xM, yM, width, height) and footprint[xF][yF] > M[xM][yM]:
                        M[xM][yM] = footprint[xF][yF]
                    yF = yF + 1
                xF = xF + 1

        # Remove points that correspond to the obstacle itself if asked for it
        # Do not combine with above loop : there would be overwriting
        if removeObstaclePoints:
            for obsPoint in obstacle_points:
                xInM = obsPoint[0] - offsetX
                yInM = obsPoint[1] - offsetY
                matrix[xInM][yInM] = 0

        return matrix

    # Helper function
    @staticmethod
    def _is_in_matrix(x, y, width, height) :
        return True if (x >= 0 and x < width and y >= 0 and y < height) else False
