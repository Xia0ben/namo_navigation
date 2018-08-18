#! /usr/bin/env python

import rospy
import numpy as np
import copy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from obstacle import Obstacle
from utils import Utils

class MultilayeredMap:
    PSEUDO_INFLATION_FOOTPRINT = [[Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL],
                                  [Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL],
                                  [Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL, Utils.ROS_COST_LETHAL]]

    def __init__(self, rosMap, frame_id, robot_metadata):
        self.info = rosMap.info
        self.robot_metadata = robot_metadata
        self.frame_id = frame_id
        # Only contains static obstacles data (walls, static furniture, ...)
        # as defined in the map given to the map server
        self.static_occ_grid = np.rot90(np.array(rosMap.data).reshape(
            (self.info.height, self.info.width)), 3)
        self.static_occ_grid[self.static_occ_grid == 100] = Utils.ROS_COST_LETHAL
        self.static_occ_grid[self.static_occ_grid == 0] = Utils.ROS_COST_FREE_SPACE
        self.static_obstacles_positions = self._get_static_obstacles_positions()
        # self.pseudo_inflated_static_occ_grid = self._get_pseudo_inflated_static_occ_grid(MultiLayeredMap.PSEUDO_INFLATION_FOOTPRINT)
        self.inflated_static_occ_grid = self._get_inflated_static_occ_grid(robot_metadata.footprint)


        # Only contains non-static (potentially movable) obstacles data (chairs, boxes, tables, ...)
        self.obstacles = {}

        # Merged grid of static elements and obstacles, with inflation applied
        self.merged_occ_grid = copy.deepcopy(self.inflated_static_occ_grid)
        self.compute_merged_occ_grid()

    def manually_add_obstacle(self, obstacle):
        self.obstacles.update({obstacle.obstacle_id: obstacle})
        self.compute_merged_occ_grid()

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
    def is_in_fov(point, pose_np, radius):
        return np.linalg.norm(np.array(point) - pose_np) <= radius

    def get_point_cloud_in_fov(self, current_pose):
        current_pose_np = np.array((current_pose.pose.position.x, current_pose.pose.position.y))
        fov_radius = self.robot_metadata.fov_radius

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
        for obstacle_id, obstacle in self.obstacles.items():
            # If obstacle is characterized by more than four points, it is more
            # interesting to first check if its boundaries are within the fov
            # radius. If not, then we simply go on to estimating the next obstacle.
            if len(obstacle.envelope) - 1 > 4:
                if not (MultilayeredMap.is_in_fov(obstacle.envelope[0], current_pose_np, fov_radius) or
                        MultilayeredMap.is_in_fov(obstacle.envelope[1], current_pose_np, fov_radius) or
                        MultilayeredMap.is_in_fov(obstacle.envelope[2], current_pose_np, fov_radius) or
                        MultilayeredMap.is_in_fov(obstacle.envelope[3], current_pose_np, fov_radius)):
                        continue

            # Iterate over the points of the obstacle's point cloud and add them
            # to the point cloud in fov
            for point in obstacle.ros_point_cloud.points:
                if MultilayeredMap.is_in_fov((point.x, point.y), current_pose_np, fov_radius):
                    fov_point_cloud.points.append(point)
                    fov_point_cloud.channels[0].values.append(obstacle_id)

        return fov_point_cloud

    def get_point_dict_in_fov(self, current_pose):
        current_pose_np = np.array((current_pose.pose.position.x, current_pose.pose.position.y))
        fov_radius = self.robot_metadata.fov_radius

        fov_point_dict = {}

        # For all obstacles that are close enough from current robot pose,
        # evaluate if their points are in the field of vision (fov) of the robot
        # and add them to the point cloud if they are.
        for obstacle_id, obstacle in self.obstacles.items():
            # If obstacle is characterized by more than four points, it is more
            # interesting to first check if its boundaries are within the fov
            # radius. If not, then we simply go on to estimating the next obstacle.
            if len(obstacle.envelope) - 1 > 4:
                if not (MultilayeredMap.is_in_fov(obstacle.envelope[0], current_pose_np, fov_radius) or
                        MultilayeredMap.is_in_fov(obstacle.envelope[1], current_pose_np, fov_radius) or
                        MultilayeredMap.is_in_fov(obstacle.envelope[2], current_pose_np, fov_radius) or
                        MultilayeredMap.is_in_fov(obstacle.envelope[3], current_pose_np, fov_radius)):
                        continue

            fov_point_dict.update({obstacle_id, set()})

            # Iterate over the points of the obstacle's point cloud and add them
            # to the point cloud in fov
            for point in obstacle.points_set:
                if MultilayeredMap.is_in_fov(point, current_pose_np, fov_radius):
                    fov_point_dict[obstacle_id].add()

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
        to_remove_point_dict = self.get_point_cloud_in_fov(current_pose)
        to_add_point_dict = {}
        to_create_obs_dict = {}

        for point32, obstacle_id_float in zip(input_point_cloud.points, input_point_cloud.channels[0].values):
            obstacle_id = int(obstacle_id_float)
            point = (point32.x, point32.y)

            try:
                # Try to get the corresponding obstacle in obstacles dict
                obstacle = self.obstacles[obstacle_id]
            except KeyError:
                # If obstacle is not found, create obstacle and add point to it
                obstacle = Obstacle({point}, self.info, self.frame_id, self.robot_metadata, obstacle_id, True)
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

        self.compute_merged_occ_grid()

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
