#! /usr/bin/env python

import rospy
import copy

from nav_msgs.msg import OccupancyGrid
from multilayered_map import MultilayeredMap

class MapManager:
    def __init__(self, robot_radius, robot_fov_radius, map_frame, static_map_topic):
        # Get parameters
        self.static_map_topic = static_map_topic # rospy.get_param('~map_topic')
        self.robot_radius = robot_radius # rospy.get_param('~robot_radius')
        self.robot_fov_radius = robot_fov_radius # rospy.get_param('~robot_fov_radius')
        self.map_frame = map_frame # rospy.get_param('~map_frame')

        # Declare common parameters
        static_map = None
        self.multilayered_map = None

        # Subscribe to appropriate topics
        self.static_map_sub = rospy.Subscriber(self.static_map_topic, OccupancyGrid, self.static_map_callback)

        # Initialize map
        while static_map is None:
            rospy.sleep(0.2)
        self.multilayered_map = MultilayeredMap(static_map, map_frame, robot_radius, robot_fov_radius)

    def static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map

    def get_map_copy(self):
        return copy.deepcopy(self.multilayered_map)

if __name__ == '__main__':
    rospy.init_node('map_manager')
    mapManager = MapManager()
    rospy.spin()
