#! /usr/bin/env python

import rospy
import numpy as np
import copy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from obstacle import Obstacle, Movability
from utils import Utils


class IntersectionWithObstaclesException(Exception):
    pass


class ObstacleOutOfBoundsException(Exception):
    pass


class IntersectionWithRobotException(Exception):
    pass


class MultilayeredMap:
    PSEUDO_INFLATION_FOOTPRINT = [[Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL],
                                  [Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL],
                                  [Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL]]

    def __init__(self, rosMap, robot_metadata):
        self.info = rosMap.info
        self.robot_metadata = robot_metadata
        self.frame_id = rosMap.header.frame_id
        # Only contains static obstacles data (walls, static furniture, ...)
        # as defined in the map given to the map server
        self.static_occ_grid = np.rot90(np.array(rosMap.data).reshape(
            (self.info.height, self.info.width)), 3)
        self.static_occ_grid[self.static_occ_grid == 100] = Utils.ROS_COST_LETHAL
        self.static_occ_grid[self.static_occ_grid == 0] = Utils.ROS_COST_FREE_SPACE
        self.static_obstacles_positions = self._get_static_obstacles_positions()
        # self.pseudo_inflated_static_occ_grid = self._get_pseudo_inflated_static_occ_grid(MultiLayeredMap.PSEUDO_INFLATION_FOOTPRINT)
        self.inflated_static_occ_grid = self._get_inflated_static_occ_grid(robot_metadata.footprint)

        self.free_space_created = False

        # Only contains non-static (potentially movable) obstacles data (chairs, boxes, tables, ...)
        self.obstacles = {}

        # Merged grid of static elements and obstacles, with inflation applied
        self.merged_occ_grid = copy.deepcopy(self.inflated_static_occ_grid)
        self.compute_merged_occ_grid()

    def manually_add_obstacle(self, points_set, obstacle_id, movability, robot_polygon):
        obstacle = Obstacle.from_points_make_polygon(
            points_set,
            self.info,
            self.frame_id,
            self.robot_metadata,
            obstacle_id,
            movability)
        # Raise exception if obstacle in intersection with robot
        if robot_polygon is not None and obstacle.polygon.intersects(robot_polygon):
            raise IntersectionWithRobotException

        # Raise exception if any of the obstacle's points are out of map bounds
        for map_point in obstacle.map_points_set:
            if not Utils._is_in_matrix(map_point[0], map_point[1], len(self.static_occ_grid), len(self.static_occ_grid[0])):
                raise ObstacleOutOfBoundsException

        # Raise exception if obstacle intersects another obstacle or the static obstacles
        if self.is_obstacle_intersecting_others(obstacle):
            raise IntersectionWithObstaclesException

        self.obstacles.update({obstacle.obstacle_id: obstacle})
        self.compute_merged_occ_grid()

    def is_obstacle_intersecting_others(self, obstacle):
        # Check for non-static obstacles
        for other_obstacle in self.obstacles.values():
            if obstacle.polygon.intersects(other_obstacle.polygon):
                return True

        # Check for static obstacles
        for map_point in obstacle.discretized_polygon:
            if map_point in self.static_obstacles_positions:
                return True

        # If all checks pass, return True
        return False

    def manually_move_obstacle(self, obstacle_id, translation):
        self.obstacles[obstacle_id].move(translation)
        self.compute_merged_occ_grid()

    def compute_merged_occ_grid(self):
        self.merged_occ_grid = copy.deepcopy(self.inflated_static_occ_grid)
        for obstacle_id, obstacle in self.obstacles.items():
            top_left_corner = obstacle.robot_inflated_obstacle["top_left_corner"]
            bottom_right_corner = obstacle.robot_inflated_obstacle["bottom_right_corner"]
            obstacle_matrix = obstacle.robot_inflated_obstacle["matrix"]

            xO, yO = 0, 0
            for x in range(top_left_corner[0], bottom_right_corner[0]):
                for y in range(top_left_corner[1], bottom_right_corner[1]):
                    if obstacle_matrix[xO][yO] > self.merged_occ_grid[x][y]:
                        self.merged_occ_grid[x][y] = obstacle_matrix[xO][yO]
                    yO = yO + 1
                xO = xO + 1
                yO = 0
            xO = 0

    #@staticmethod
    def is_in_fov(self, point, current_position, radius):
        return Utils.euclidean_distance(point, current_position) <= radius

    def get_point_cloud_in_fov(self, current_pose):
        current_position = (current_pose.pose.position.x, current_pose.pose.position.y)
        fov_radius = self.robot_metadata.fov_radius

        fov_point_cloud = PointCloud()
        fov_point_cloud.header.seq = 1
        fov_point_cloud.header.frame_id = self.frame_id
        fov_point_cloud.header.stamp = rospy.Time.now()
        fov_point_cloud.points = []
        fov_point_cloud.channels = [ChannelFloat32()]
        fov_point_cloud.channels[0].name = "obstacle_id"
        fov_point_cloud.channels[0].values = []

        # For all obstacles that are close enough from current robot pose,
        # evaluate if their points are in the field of vision (fov) of the robot
        # and add them to the point cloud if they are.
        for obstacle_id, obstacle in self.obstacles.items():
            # If obstacle is characterized by more than four points, it is more
            # interesting to first check if its boundaries are within the fov
            # radius. If not, then we simply go on to estimating the next obstacle.
            if len(obstacle.ros_point_cloud.points) > 4:
                if not (self.is_in_fov(obstacle.envelope.exterior.coords[0], current_position, fov_radius) or
                        self.is_in_fov(obstacle.envelope.exterior.coords[1], current_position, fov_radius) or
                        self.is_in_fov(obstacle.envelope.exterior.coords[2], current_position, fov_radius) or
                        self.is_in_fov(obstacle.envelope.exterior.coords[3], current_position, fov_radius)):
                        continue

            # Iterate over the points of the obstacle's point cloud and add them
            # to the point cloud in fov
            for point in obstacle.ros_point_cloud.points:
                if self.is_in_fov((point.x, point.y), current_position, fov_radius):
                    fov_point_cloud.points.append(point)
                    fov_point_cloud.channels[0].values.append(obstacle_id)

        return fov_point_cloud

    def get_point_dict_in_fov(self, current_pose):
        current_position = (current_pose.pose.position.x, current_pose.pose.position.y)
        fov_radius = self.robot_metadata.fov_radius

        fov_point_dict = {}

        # For all obstacles that are close enough from current robot pose,
        # evaluate if their points are in the field of vision (fov) of the robot
        # and add them to the point cloud if they are.
        for obstacle_id, obstacle in self.obstacles.items():
            # If obstacle is characterized by more than four points, it is more
            # interesting to first check if its boundaries are within the fov
            # radius. If not, then we simply go on to estimating the next obstacle.
            if len(obstacle.points_set) > 4:
                if not (self.is_in_fov(obstacle.envelope.exterior.coords[0], current_position, fov_radius) or
                        self.is_in_fov(obstacle.envelope.exterior.coords[1], current_position, fov_radius) or
                        self.is_in_fov(obstacle.envelope.exterior.coords[2], current_position, fov_radius) or
                        self.is_in_fov(obstacle.envelope.exterior.coords[3], current_position, fov_radius)):
                        continue

            # Iterate over the points of the obstacle's point cloud and add them
            # to the point cloud in fov
            for point in obstacle.points_set:
                if self.is_in_fov(point, current_position, fov_radius):
                    try:
                        fov_point_dict[obstacle_id].add(point)
                    except KeyError:
                        fov_point_dict.update({obstacle_id: {point}})

        return fov_point_dict

    # def point_cloud_to_point_dict(point_cloud):
    #     point_dict = {}
    #     for point, obstacle_id_float in zip(point_cloud.points, point_cloud.channels[0].values):
    #         obstacle_id = int(obstacle_id_float)
    #         # Try to add point to existing obstacle set
    #         try:
    #             point_dict[obstacle_id].add((point.x, point.y))
    #         except KeyError:
    #             # If obstacle doesn't exist add obstacle and add point to its set
    #             point_dict.update({obstacle_id: {(point.x, point.y)}})
    #     return point_dict

    def update_from_point_cloud(self, input_point_cloud, current_pose):
        to_remove_point_obs_dict = self.get_point_dict_in_fov(current_pose)
        to_add_point_obs_dict = {}
        to_create_obs_dict = {}

        # Translate pointcloud as modifications for obstacles
        for point32, obstacle_id_float in zip(input_point_cloud.points, input_point_cloud.channels[0].values):
            obstacle_id = int(obstacle_id_float)
            point = (point32.x, point32.y)

            try:
                # Try to get the corresponding obstacle in obstacles dict
                obstacle = self.obstacles[obstacle_id]

                # If input point exists in both point clouds, don't remove later from obstacle point cloud
                if point in obstacle.points_set:
                    to_remove_point_obs_dict[obstacle_id].remove(point)
                    # If set is empty, remove obstacle altogether
                    if not to_remove_point_obs_dict[obstacle_id]:
                        del to_remove_point_obs_dict[obstacle_id]
                else:
                    # If the obstacle does not already have this point, add it later
                    try:
                        obs_points_set = to_add_point_obs_dict[obstacle_id]
                        obs_points_set.add(point)
                    except KeyError:
                        to_add_point_obs_dict.update({obstacle_id: {point}})

            except KeyError:
                # If obstacle is not found, we will create it later with its points
                try:
                    obs_points_set = to_create_obs_dict[obstacle_id]
                    obs_points_set.add(point)
                except KeyError:
                    to_create_obs_dict.update({obstacle_id: {point}})

        # Modify obstacles, remove, add, and create
        for obstacle_id, obs_points_set in to_remove_point_obs_dict.items():
            self.obstacles[obstacle_id].remove_points(obs_points_set)
            self.free_space_created = True

        for obstacle_id, obs_points_set in to_add_point_obs_dict.items():
            self.obstacles[obstacle_id].add_points(obs_points_set)

        for obstacle_id, obs_points_set in to_create_obs_dict.items():
            new_obstacle = Obstacle(obs_points_set, self.info, self.frame_id, self.robot_metadata, obstacle_id, Movability.maybe_movable)
            self.obstacles.update({new_obstacle.obstacle_id: new_obstacle})

        self.compute_merged_occ_grid()

    def has_free_space_been_created(self):
        value_on_call = self.free_space_created
        self.free_space_created = False
        return value_on_call

    # def pushUsingGraspPointPossible(self, obstacle, graspPoint):
    # # FIXME first simulate a discrete push in d. Don't forget to update both obstacle.x and y !
    # # FIXME Check if graspPoint enters in collision with surrounding known space in the map.
    # pass

    def _get_static_obstacles_positions(self):
        positions = set()
        for x in range(len(self.static_occ_grid)):
            for y in range(len(self.static_occ_grid[0])):
                if self.static_occ_grid[x][y] == Utils.ROS_COST_LETHAL:
                    positions.add((x, y))
        return positions

    def _get_inflated_static_occ_grid(self, footprint):
        return Utils._get_inflastamped_matrix(self.static_obstacles_positions,
                                                footprint,
                                                len(self.static_occ_grid),
                                                len(self.static_occ_grid[0]))
