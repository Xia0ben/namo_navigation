#! /usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import OccupancyGrid

from multilayered_map import MultilayeredMap
from simple_position import SimplePosition

import math
import numpy

class MapManager:
    def __init__(self):
        # Get parameters
        self.static_map_topic = rospy.get_param('~map_topic')
        self.robot_diameter = rospy.get_param('~robot_diameter')
        self.map_frame = rospy.get_param('~map_frame')
        self.robot_frame = rospy.get_param('~robot_frame')
        
        # Laserscan specific parameters
        self.scan_topic = rospy.get_param('~scan_topic') # TODO change for ~<name>/<source_name>/topic 
        self.laserscan_frame = rospy.get_param('~laserscan_frame') # TODO change for ~<name>/<source_name>/sensor_frame
        self.obstacle_range = rospy.get_param('~obstacle_range') 
        self.raytrace_range = rospy.get_param('~raytrace_range') 
        
        # TODO Implement these params for each sensor (starting with laserscan) ;
        # See http://wiki.ros.org/costmap_2d/hydro/obstacles :
        # observation_persistence
        # expected_update_rate
        # data_type
        # clearing
        # marking 
        # max_obstacle_height
        # min_obstacle_height
        # inf_is_valid

        # Declare common parameters
        self.static_map = None
        self.multilayered_map = None
        
        # Subscribe to appropriate topics
        self.tf_listener = tf.TransformListener()
        self.static_map_sub = rospy.Subscriber(self.static_map_topic, OccupancyGrid, self.static_map_callback)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        
        # Initialize map
        while self.static_map is None:
            rospy.sleep(0.2)
        self.multilayered_map = MultilayeredMap(self.static_map, self.robot_diameter)
    
    def static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map
    
    # Distance in meters, angle in radians
    @staticmethod
    def _laser_distance_to_local_coords(distance, angle):
        return numpy.array([distance * math.cos(angle),
                            distance * math.sin(angle)])
    
    def scan_callback(self, laserscan):
        time  = rospy.Time(0) # Makes sure same tf is used for calls
        
        robot_pose = None
        # Get robot pose
        try:
            (position, quaternion) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)
            robot_pose = SimplePose.from_pos_quat(position, quaternion, self.map_frame)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Couldn't get robot pose in map : transform unavailable. \
            Maybe amcl is not working and there is no transform between \
            /odom and /map ?")
            return # exit callback if error
        
        
        # Convert laserscan readings into points for the map
        pointsWithDistance = {}
        for i in range(len(laserscan.ranges)):
            distance = laserscan.ranges[i]
            
            # Simply discard values that are not in the laser's specs
            if distance >= laserscan.range_min or distance <= laserscan.range_max:
            angle = laserscan.angle_min + laserscan.angle_increment * i
            local_point = MapManager._laser_distance_to_local_coords(distance, angle)
            
            pointstamp_local = PointStamped()
            pointstamp_local.header.frame_id = self.laserscan_frame
            pointstamp_local.header.stamp =  time
            pointstamp_local.point.x = local_point[0]
            pointstamp_local.point.y = local_point[1]
            pointstamp_local.point.z = 0.0
            
            try:
                pointstamp_map = self.tf_listener.transformPoint(self.map_frame, pointstamp_local)
                
                ### Conversion to map coordinates ###
                # TODO : may need to fix the conversion if map is offsetted regarding to
                # the world coordinates (here is done simply by the use of 2DPosition class)
                pointsWithDistance[SimplePosition(pointstamp_map.point.x, pointstamp_map.point.y)] = distance
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                rospy.logerr("Couldn't add laserscan : transform unavailable. \
            Maybe amcl is not working and there is no transform between \
            /odom and /map ?")
                return # exit callback if error
        
        self.multilayered_map.update_laserscan(pointsWithDistance, robot_pose, self.obstacle_range, self.raytrace_range)
    
if __name__ == '__main__':
    rospy.init_node('map_manager')
    mapManager = MapManager()
    rospy.spin()