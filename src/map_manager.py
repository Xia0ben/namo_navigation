#! /usr/bin/env python

import rospy
import copy

from nav_msgs.msg import OccupancyGrid
from multilayered_map import MultilayeredMap
from robot_meta_data import RobotMetaData
from utils import Utils
from obstacle import Obstacle

class MapManager:
    def __init__(self, robot_radius, robot_fov_radius, map_frame, static_map_topic):
        # Get parameters
        self.static_map_topic = static_map_topic # rospy.get_param('~map_topic')
        self.map_frame = map_frame # rospy.get_param('~map_frame')

        # Declare common parameters
        self.static_map = None
        self.multilayered_map = None

        # Subscribe to appropriate topics
        self.static_map_sub = rospy.Subscriber(self.static_map_topic, OccupancyGrid, self.static_map_callback)

        # Initialize map
        while self.static_map is None:
            rospy.sleep(0.2)
        self.robot_metadata = RobotMetaData(robot_radius, robot_fov_radius, self.static_map.info.resolution)
        self.multilayered_map = MultilayeredMap(self.static_map, map_frame, self.robot_metadata)

        # DEBUG
        print(self.multilayered_map.info)
        print(self.multilayered_map.robot_metadata)
        print(self.multilayered_map.frame_id)
        print(self.multilayered_map.static_occ_grid)
        print(self.multilayered_map.static_obstacles_positions)
        print(self.multilayered_map.inflated_static_occ_grid)
        print(self.multilayered_map.obstacles)
        print(self.multilayered_map.merged_occ_grid)

    def static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map

    def get_map_copy(self):
        return copy.deepcopy(self.multilayered_map)

    def manually_add_obstacle(self, obstacle):
        self.multilayered_map.manually_add_obstacle(obstacle)

if __name__ == '__main__':
    rospy.init_node('map_manager')
    map_manager = MapManager(0.05, 1.0, "/map", "/map")

    obstacle1 = Obstacle.from_points_make_polygon(Utils.map_coords_to_real_coords(
                                                    {(8, 1), (9, 1), (8, 6), (9, 6)},
                                                    map_manager.multilayered_map.info.resolution),
                                                  map_manager.multilayered_map.info,
                                                  map_manager.map_frame,
                                                  map_manager.robot_metadata,
                                                  1,
                                                  True)

    map_manager.manually_add_obstacle(obstacle1)

    print(obstacle1.robot_inflated_obstacle["matrix"])
    print(obstacle1.obstacle_matrix["matrix"])

    rospy.spin()
