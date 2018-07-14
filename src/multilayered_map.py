#! /usr/bin/env python

import math
import copy

from bresenham import bresenham

from pose import Pose

from sklearn.cluster import DBSCAN
from sklearn import metrics

from sensor_msgs import PointCloud, ChannelFloat32

class MultilayeredMap:
    LETHAL_COST_ROS = 254
    FREESPACE_COST_ROS = 0
    NEWPOINT_VALUE = -1

    # DBSCAN_EPSILON = 10
    # DBSCAN_MIN_SAMPLES = 1
    # DBSCAN_ALGORITHM = 'auto'
    # DBSCAN_METRIC = 'euclidean'
    # DBSCAN_METRIC_PARAMS = None
    # DBSCAN_LEAF_SIZE = 30
    # DBSCAN_P = None
    # N_JOBS = 1

    PSEUDO_INFLATION_FOOTPRINT = [[1, 1, 1],
                                  [1, 1, 1],
                                  [1, 1, 1]]

    def __init__(self, rosMap, robotDiameter, fov_radius, obstacles_dict = None):
        self.origin = rosMap.info.origin
        self.resolution = rosMap.info.resolution
        self.width = rosMap.info.width
        self.height = rosMap.info.height
        # Only contains static obstacles data (walls, static furniture, ...)
        # as defined in the map given to the map server
        self.static_occ_grid = np.array(rosMap.data).reshape(
            (self.width, self.height))
        self.static_obstacles_positions = _get_static_obstacles_positions()
        self.pseudo_inflated_static_occ_grid = _get_pseudo_inflated_static_occ_grid()

        # Only contains non-static (potentially movable) obstacles data (chairs, boxes, tables, ...)
        self.obstacles = {}
        if obstacles_dict is not None:
            for obstacle_id, obstacle_points_set in obstacles_dict.items():
                self.obstacles.update({obstacle_id: Obstacle(obstacle_points_set, self.resolution, obstacle_id)})

        # Merged grid of static elements and obstacles, with inflation applied
        self.merged_occ_grid = copy.deepcopy(self.static_occ_grid)
        compute_merged_occ_grid()

        self.robot = Robot(robotDiameter, fov_radius, self.resolution)

    def compute_merged_occ_grid(self):
        # FIXME
        # Add all obstacles to the

    @staticmethod
    def is_in_fov(point, pose_np, radius):
        return np.linalg.norm(np.array(point) - pose_np) <= radius

    def get_point_cloud_in_fov(self, current_pose):
        current_pose_np = np.array((current_pose.pose.position.x, current_pose.pose.position.y))
        fov_radius = self.robot.fov_radius
        
        fov_point_cloud = PointCloud()
        fov_point_cloud.header.seq = 1
        fov_point_cloud.header.frame_id = self.frame_id
        fov_point_cloud.header.stamp = rospy.Time(0)
        fov_point_cloud.points = []
        fov_point_cloud.channels = [ChannelFloat32()]
        fov_point_cloud.channels[0].name = "obstacle_id"
        fov_point_cloud.channels[0].values = []
        
        # For all obstacles that are close enough from current robot pose,
        # evaluate if their points are in the field of vision (fov) of the robot
        # and add them to the point cloud if they are.
        for obstacle_id, obstacle in obstacles.items():
            # If obstacle is characterized by more than four points, it is more
            # interesting to first check if its boundaries are within the fov
            # radius. If not, then we simply go on to estimating the next obstacle.
            obstacle_bb = obstacle.shapely_multipoint.envelope.exterior.coords
            if len(obstacle_bb) - 1 > 4:
                if not (MultilayeredMap.is_in_fov(obstacle_bb[0], current_pose_np, radius) or
                        MultilayeredMap.is_in_fov(obstacle_bb[1], current_pose_np, radius) or
                        MultilayeredMap.is_in_fov(obstacle_bb[2], current_pose_np, radius) or
                        MultilayeredMap.is_in_fov(obstacle_bb[3], current_pose_np, radius)):
                        continue
            
            # Iterate over the points of the obstacle's point cloud and add them
            # to the point cloud in fov
            for point in obstacle.ros_point_cloud.points:
                if MultilayeredMap.is_in_fov((point.x, point.y), current_pose_np, radius):
                    fov_point_cloud.points.append(point)
                    fov_point_cloud.channels[0].values.append(obstacle_id)
        
        return fov_point_cloud

    def point_cloud_to_point_dict(point_cloud):
        point_dict = {}
        for point, obstacle_id_float in zip(point_cloud.points, point_cloud.channels[0].values):
            obstacle_id = int(obstacle_id_float)
            # Try to add point to existing obstacle set
            try:
                point_dict[obstacle_id].add((point.x, point.y))
            except KeyError:
                # If obstacle doesn't exist add obstacle and add point to its set
                point_dict.update({obstacle_id: {(point.x, point.y)}})
        return point_dict

    def update_from_point_cloud(self, input_point_cloud):
        to_remove_point_dict = self.get_point_cloud_in_fov(self, current_pose)
        to_create_obs_dict = {}
        
        for point32, obstacle_id_float in zip(input_point_cloud.points, input_point_cloud.channels[0].values):
            obstacle_id = int(obstacle_id_float)
            point = (point32.x, point32.y)
            
            try:
                # Try to get the corresponding obstacle in obstacles dict
                obstacle = self.obstacles[obstacle_id]
            except KeyError:
                # If obstacle is not found, create obstacle and add point to it
                obstacle = Obstacle({point}, self.resolution, obstacle_id)
                # Get to the next point
                continue
            
            if obstacle.has_point(point):
                # If input point exists in both point clouds, don't remove later
                # from obstacle point cloud
                to_remove_point_dict[obstacle_id].remove(point)
            else:
                # If input point doesn't exist in obstacle, add it
                obstacle.add_point(point)
        
        # Remove all points that are not in common or have not just been added
        # from obstacle point cloud
        for obstacle_id, point_set in to_remove_point_dict.items():
            for point in point_set:
                self.obstacles[obstacle_id].remove_point(point)
        
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

        matrix = np.zeros((width, height))

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

    # def update_laserscan(self, pointsWithDistance, robot_pose, obstacle_range, raytrace_range):
    #     updated_obstacle_occ_grid = copy.deepcopy(self.obstacle_occ_grid)
    #     updated_obstacles = copy.deepcopy(self.obstacles)

    #     ### Clearing ###
    #     ## Raytracing method ##
    #     # Use bresenham algorithm to trace lines in 2D:
    #     # (https://en.wikipedia.org/wiki/Bresenham's_line_algorithm)
    #     # list(bresenham) returns an array of tuples [(x,y), ...]
    #     pointsToClear = set()
    #     for point, distance in pointsWithDistance.items():
    #         if distance <= raytrace_range:
    #             line_points = list(
    #                 bresenham(robot_pose.pos.coords[0], # x1
    #                           robot_pose.pos.coords[1], # y1
    #                           point.coords[0],  # x2
    #                           point.coords[1])) # y2
    #             for point in line_points:
    #                 pointsToClear.add(point)

    #     # pointsToClear is a set of tuples set([(x,y), ...])
    #     new_cleared_points = []
    #     for point in pointsToClear:
    #         # TODO Maybe use a function that degrades over time, starting from
    #         # 127 (free space) going to 0, over 20 seconds ?
    #         # Would need to update inflation function to take this kind of case
    #         # into account.
    #         current_obstacle_id = self.updated_obstacle_occ_grid[point[0]][point[1]]
    #         if current_obstacle_id > 0:
    #             self.updated_obstacle_occ_grid[point[0]][point[1]] = MultilayeredMap.FREESPACE_COST_ROS
    #             updated_obstacles[current_obstacle_id].removePoin # FIXME FINISH THIS LINE
    #             new_cleared_points.append(SimplePosition(point[0], point[1]))

    #     ## "Conetracing" method ##
    #     # Draw polygons of successive clearable points of the laser_points list
    #     # And remove all registered points within them
    #     # TODO Implement this idea if appropriate

    #     ### Marking and Static Map Overlap Checking ###
    #     # Add points only if within marking distance, and that they are not in
    #     # a pseudo inflated static occupation spot
    #     for point, distance in pointsWithDistance.items():
    #         isInRange = (distance <= obstacle_range)
    #         isInMatrix = _is_in_matrix(point.coords[0], point.coords[1],
    #                                   self.width, self.height)
    #         isNotInPseudo = (self.pseudo_inflated_static_occ_grid[point.coords[0]][point.coords[1]] == 0)
    #         if (isInRange and isInMatrix and isNotInPseudo):
    #             self.updated_obstacle_occ_grid = MultilayeredMap.LETHAL_COST_ROS


    #     # Clusterize points into separate obstacles
    #     db = DBSCAN(eps=MultilayeredMap.DBSCAN_EPSILON,
    #         min_samples=MultilayeredMap.DBSCAN_MIN_SAMPLES,
    #         algorithm=MultilayeredMap.DBSCAN_ALGORITHM ,
    #         metric=MultilayeredMap.DBSCAN_METRIC,
    #         metric_params=MultilayeredMap.DBSCAN_METRIC_PARAMS,
    #         leaf_size=MultilayeredMap.DBSCAN_LEAF_SIZE,
    #         p=MultilayeredMap.DBSCAN_P,
    #         n_jobs=MultilayeredMap.N_JOBS).fit(npObstaclePoints)

    #     cluster_labels = db.labels_
    #     n_clusters = len(set(cluster_labels)) - (1 if -1 in cluster_labels else 0)

    #     # get the clusters
    #     # cluster_labels = -1 means outliers, we ignore it
    #     clusters = []
    #     for cluster_label in range(n_clusters):
    #         clusters.append([])

    #     for i in range(len(cluster_labels)):
    #         clusters[cluster_labels[i]].append(npObstaclePoints[i])