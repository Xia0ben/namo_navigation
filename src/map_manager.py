#! /usr/bin/env python

import rospy
import copy

from nav_msgs.msg import OccupancyGrid
from multilayered_map import MultilayeredMap
from robot_meta_data import RobotMetaData
from utils import Utils
from obstacle import Obstacle

class MapManager:
    def __init__(self, robot_radius, robot_fov_radius, map_frame, static_map_topic, merged_occ_grid_topic):
        # Get parameters
        self.static_map_topic = static_map_topic # rospy.get_param('~map_topic')
        self.merged_occ_grid_topic = merged_occ_grid_topic
        self.map_frame = map_frame # rospy.get_param('~map_frame')

        # Declare common parameters
        self.static_map = None
        self.multilayered_map = None

        # Subscribe to appropriate topics
        self.static_map_sub = rospy.Subscriber(self.static_map_topic, OccupancyGrid, self.static_map_callback)

        # Publish occupancy grid topic
        self.simulated_merged_occ_grid_pub = rospy.Publisher(self.merged_occ_grid_topic, OccupancyGrid, queue_size=1)

        # Initialize map
        while self.static_map is None:
            rospy.sleep(0.2)
        self.robot_metadata = RobotMetaData(robot_radius, robot_fov_radius, self.static_map.info.resolution)
        self.multilayered_map = MultilayeredMap(self.static_map, map_frame, self.robot_metadata)

    def static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map

    def get_map_copy(self):
        return copy.deepcopy(self.multilayered_map)

    def manually_add_obstacle(self, obstacle):
        self.multilayered_map.manually_add_obstacle(obstacle)

    def publish_ros_merged_occ_grid(self):
        ros_merged_occ_grid = OccupancyGrid()
        ros_merged_occ_grid.header = self.static_map.header
        ros_merged_occ_grid.header.stamp = rospy.Time(0)
        ros_merged_occ_grid.info = self.static_map.info
        ros_merged_occ_grid.data = list(map(int, self.multilayered_map.merged_occ_grid.flatten()))
        Utils.publish_once(self.simulated_merged_occ_grid_pub, ros_merged_occ_grid)

if __name__ == '__main__':
    rospy.init_node('map_manager')
    map_manager = MapManager(0.05, 1.0, "/map", "/map", "/simulated/global_occupancy_grid")

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

    rospy.spin()
