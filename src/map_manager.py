#! /usr/bin/env python

import rospy
import copy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseArray, PoseStamped
from multilayered_map import MultilayeredMap
from robot_meta_data import RobotMetaData
from utils import Utils
from obstacle import Obstacle

from threading import Lock

class MapManager:
    map_lock = Lock()
    robot_pose_lock = Lock()

    def __init__(self, robot_radius, robot_fov_radius, static_map_topic, robot_occ_grid_topic, push_poses_topic,
                 simulated_pose_topic, simulated_fov_pointcloud_topic):
        # Get parameters
        self.static_map_topic = static_map_topic
        self.robot_occ_grid_topic = robot_occ_grid_topic
        self.push_poses_topic = push_poses_topic
        self.simulated_pose_topic = simulated_pose_topic
        self.simulated_fov_pointcloud_topic = simulated_fov_pointcloud_topic

        # Declare common parameters
        self.static_map = None
        self.init_map = None
        self.multilayered_map = None
        self.has_free_space_been_created = False
        self.current_robot_pose = None

        # Create subscribers
        rospy.Subscriber(self.static_map_topic, OccupancyGrid, self._static_map_callback)
        rospy.Subscriber(self.simulated_fov_pointcloud_topic, PointCloud, self._simulated_fov_pointcloud_callback)
        rospy.Subscriber(self.simulated_pose_topic, PoseStamped, self._simulated_pose_callback)

        # Create publishers
        self.robot_occ_grid_pub = rospy.Publisher(self.robot_occ_grid_topic, OccupancyGrid, queue_size=1)
        self.push_poses_pub = rospy.Publisher(self.push_poses_topic, PoseArray, queue_size=1)

        # Initialize map
        while self.static_map is None:
            rospy.sleep(0.2)
        self.robot_metadata = RobotMetaData(robot_radius, robot_fov_radius, self.static_map.info.resolution)
        self.init_map = MultilayeredMap(self.static_map, self.robot_metadata)
        self.multilayered_map = copy.deepcopy(self.init_map)

    def _static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map

    def _simulated_pose_callback(self, pose):
        MapManager.robot_pose_lock.acquire()
        self.current_robot_pose = pose
        MapManager.robot_pose_lock.release()

    def _simulated_fov_pointcloud_callback(self, pointcloud):
        MapManager.robot_pose_lock.acquire()
        robot_pose_copy = copy.deepcopy(self.current_robot_pose)
        MapManager.robot_pose_lock.release()

        if self.multilayered_map is not None and robot_pose_copy is not None:
            if (self.current_robot_pose.header.stamp.to_nsec() <= pointcloud.header.stamp.to_nsec()):
                MapManager.map_lock.acquire()
                self.multilayered_map.update_from_point_cloud(pointcloud, robot_pose_copy)
                MapManager.map_lock.release()
                self.publish_ros_merged_occ_grid()

    def get_init_map(self):
        self.publish_ros_static_inflated_grid()
        return copy.deepcopy(self.init_map) # We make a copy to be sure its not changed

    def get_map_copy(self):
        MapManager.map_lock.acquire()
        map_copy =  copy.deepcopy(self.multilayered_map)
        MapManager.map_lock.release()
        return map_copy

    # FIXME DEPRECATED
    def manually_add_obstacle(self, obstacle):
        self.multilayered_map.manually_add_obstacle(obstacle)

    def publish_ros_merged_occ_grid(self):
        ros_merged_occ_grid = Utils.convert_matrix_to_ros_occ_grid(self.multilayered_map.merged_occ_grid, self.static_map.header, self.static_map.info)
        Utils.publish_once(self.robot_occ_grid_pub, ros_merged_occ_grid)

    def publish_ros_static_inflated_grid(self):
        ros_merged_occ_grid = Utils.convert_matrix_to_ros_occ_grid(self.multilayered_map.inflated_static_occ_grid, self.static_map.header, self.static_map.info)
        Utils.publish_once(self.robot_occ_grid_pub, ros_merged_occ_grid)

    def publish_all_push_poses(self):
        all_push_poses = PoseArray()
        all_push_poses.header = self.static_map.header
        all_push_poses.header.stamp = rospy.Time.now()
        for obstacle_id, obstacle in self.multilayered_map.obstacles.items():
            for push_pose in obstacle.push_poses:
                all_push_poses.poses = all_push_poses.poses + [push_pose.pose]
        Utils.publish_once(self.push_poses_pub, all_push_poses)

if __name__ == '__main__':
    rospy.init_node('map_manager')
    map_manager = MapManager(0.05, 1.0, "/map", "/simulated/global_occupancy_grid", "/simulated/all_push_poses")

    # This obstacle with these specific coordinates can only be inserted in the 01 map
    obstacle1 = Obstacle.from_points_make_polygon(Utils.map_coords_to_real_coords(
                                                    {(7, 1), (8, 1), (7, 6), (8, 6)},
                                                    map_manager.multilayered_map.info.resolution),
                                                  map_manager.multilayered_map.info,
                                                  map_manager.map_frame,
                                                  map_manager.robot_metadata,
                                                  1,
                                                  True)

    map_manager.manually_add_obstacle(obstacle1)

    # print(obstacle1.robot_inflated_obstacle["matrix"])
    # print(obstacle1.obstacle_matrix["matrix"])
    #
    # # TEST
    # print(map_manager.multilayered_map.info)
    # print(map_manager.multilayered_map.robot_metadata)
    # print(map_manager.multilayered_map.frame_id)
    print(map_manager.multilayered_map.static_occ_grid)
    print(map_manager.multilayered_map.static_obstacles_positions)
    print(map_manager.multilayered_map.inflated_static_occ_grid)
    print(map_manager.multilayered_map.obstacles)
    print(map_manager.multilayered_map.merged_occ_grid)

    map_manager.publish_ros_merged_occ_grid()
    map_manager.publish_all_push_poses()

    rospy.spin()
