#! /usr/bin/env python

import rospy
import tf
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped

class LocalPlanner(object):
    ODOM_TOPIC = '/odom'
    VEL_TOPIC = '/cmd_vel'
    MAX_LINEAR_SPEED = 0.3
    MAX_ANGULAR_SPEED = 0.5
    
    UNIT_VECTOR = np.array([1, 0])
    ANGLE_TOLERANCE = 0.03 # [rad] ~= 2 [deg]
    DISTANCE_TOLERANCE = 0.05 # [m]
    
    def __init__(self):
        
        # Init odom topic sub
        self._sub = rospy.Subscriber(LocalPlanner.ODOM_TOPIC, Odometry, self.odom_callback)
        self._odomdata = Odometry()
        
        self.tf_listener = tf.TransformListener()
        
        # Init cmd_vel pub
        self._cmd_vel_pub = rospy.Publisher(LocalPlanner.VEL_TOPIC, Twist, queue_size=1)
        self._twist_object = Twist()
        
        rospy.loginfo("Local planner inited")
    
    def odom_callback(self, msg):
        self._odomdata = msg
        
    def move_robot(self, direction):
        if direction == "forward":
            self._twist_object.linear.x = LocalPlanner.MAX_LINEAR_SPEED
            self._twist_object.angular.z = 0.0
        elif direction == "right":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = -LocalPlanner.MAX_ANGULAR_SPEED
        elif direction == "left":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = LocalPlanner.MAX_ANGULAR_SPEED
        elif direction == "backward":
            self._twist_object.linear.x = -LocalPlanner.MAX_LINEAR_SPEED
            self._twist_object.angular.z = 0.0
        elif direction == "stop":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = 0.0
        else:
            pass
        
        # rospy.loginfo("move_robot function called with twist : %s", self._twist_object)
        
        """
        The following loop is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while True:
            connections = self._cmd_vel_pub.get_num_connections()
            if connections > 0:
                self._cmd_vel_pub.publish(self._twist_object)
                break
    
    def get_odom(self):
        return self._odomdata
    
    @staticmethod
    def yaw_from_geom_quat(geom_quat):
        explicit_quat = [geom_quat.x,
                         geom_quat.y,
                         geom_quat.z,
                         geom_quat.w]
        return tf.transformations.euler_from_quaternion(explicit_quat)[2]
                         
    def turn_in_place(self, target_yaw, angle_tolerance, turn_direction):
        # Start turning
        #rospy.loginfo("Start turning to target yaw : %s", target_yaw)
        self.move_robot(turn_direction)
        
        # Wait for target angle to be attained then break
        while True:
            current_robot_odometry = self._odomdata
            current_yaw = LocalPlanner.yaw_from_geom_quat(current_robot_odometry.pose.pose.orientation)
            
            # Break condition
            if np.isclose(target_yaw, current_yaw, atol = angle_tolerance):
                break
        
        # Stop turning
        self.move_robot("stop")
        #rospy.loginfo("Stop, odometry is at : %s", current_robot_odometry)
        
        return True # TODO check if rotation actually succeeds, and if not, return False
        
    def move_linearly(self, target_position, distance_tolerance, move_direction):
        # Start moving
        #rospy.loginfo("Start moving to target point : %s", target_position)
        self.move_robot(move_direction)
        
        # Wait for target position to be attained then break
        while True:
            current_robot_odometry = self._odomdata
            current_direction = target_position - np.array(
                [current_robot_odometry.pose.pose.position.x,
                 current_robot_odometry.pose.pose.position.y])
            current_direction_norm = np.linalg.norm(current_direction)
            
            # Break condition
            if np.isclose(0.0, current_direction_norm, atol = distance_tolerance):
                break
        
        # Stop turning
        self.move_robot("stop")
        #rospy.loginfo("Stop moving to target point : %s", target_position)
        
        return True # TODO check if move actually succeeds, and if not, return False

    def robot_local_point(self, point, robot_pos, theta):
        point_translated = (point[0] - robot_pos[0], point[1] - robot_pos[1])
        point_translated_rotated = (point_translated[0] * math.cos(theta) - point_translated[1] * math.sin(theta),
                                    point_translated[0] * math.sin(theta) + point_translated[1] * math.cos(theta))
        
        pointstamp = PointStamped()
        pointstamp.header.frame_id = "odom"
        pointstamp.header.stamp = rospy.Time(0)
        pointstamp.point.x = point[0]
        pointstamp.point.y = point[1]
        pointstamp.point.z = 0.0
        
        while True:
            try:
                tf_pointstamp = self.tf_listener.transformPoint("base_link", pointstamp)
                tf_point_translated_rotated = (tf_pointstamp.point.x, tf_pointstamp.point.y)
                rospy.loginfo("Tf local waypoint : %s", tf_point_translated_rotated)
                break
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass
        
        return tf_point_translated_rotated
    
    def modulo_pi(self, angle):
        if 0.0 <= angle <= math.pi or -math.pi <= angle <= 0.0:
            return angle
        elif math.pi < angle < 2.0 * math.pi:
            return -2.0 * math.pi + angle
        elif -2.0 * math.pi < angle < -math.pi:
            return 2.0 * math.pi + angle
        elif angle >=  2.0 * math.pi:
            return self.modulo_pi(angle - 2.0 * math.pi)
        elif angle <=  -2.0 * math.pi:
            return self.modulo_pi(angle + 2.0 * math.pi)
    
    def follow_path(self, path):
        rospy.loginfo("Start of path following...")
        for waypoint in path:
            rospy.loginfo("Going to waypoint : %s", waypoint)
            current_robot_odometry = self._odomdata
            current_position = (current_robot_odometry.pose.pose.position.x, current_robot_odometry.pose.pose.position.y)
            current_yaw = LocalPlanner.yaw_from_geom_quat(current_robot_odometry.pose.pose.orientation)
            
            waypoint_local = self.robot_local_point(waypoint, current_position, current_yaw)
            rospy.loginfo("Local waypoint : %s", waypoint_local)
            
            # If the waypoint is on the left side of the robot, go left, else the reverse
            if waypoint_local[1] > 0:
                turn_direction = "left"
            else:
                turn_direction = "right"
            
            local_target_direction = np.array(waypoint_local)
            local_target_direction_norm = np.linalg.norm(local_target_direction)
            
            if waypoint_local[0] < 0:
                rospy.loginfo("Correct arcos")
                target_local_yaw = math.pi - math.acos(np.dot(local_target_direction, LocalPlanner.UNIT_VECTOR) /
                    local_target_direction_norm)
            else:
                rospy.loginfo("Don't correct arcos")
                target_local_yaw = math.acos(np.dot(local_target_direction, LocalPlanner.UNIT_VECTOR) /
                    local_target_direction_norm)
                
            target_yaw = self.modulo_pi(current_yaw + target_local_yaw)
            rospy.loginfo("Target local yaw : %s [rad], %s [deg]", target_local_yaw, math.degrees(target_local_yaw))
            rospy.loginfo("Target yaw : %s [rad], %s [deg]", target_yaw, math.degrees(target_yaw))
            
            # Turn then proceed forward
            self.turn_in_place(target_yaw, LocalPlanner.ANGLE_TOLERANCE, turn_direction)
            self.move_linearly(np.array(waypoint), LocalPlanner.DISTANCE_TOLERANCE, "forward")
        rospy.loginfo("End of path following...")
        
if __name__ == "__main__":
    rospy.init_node('local_planner', log_level=rospy.INFO)
    
    local_planner = LocalPlanner()

    PATH = [(1, 0), (1, 1), (0, 1), (0, 0)]
    
    local_planner.follow_path(PATH)
    
    # rate = rospy.Rate(5)
    
    # local_planner.move_robot("left")
    
    # while not rospy.is_shutdown():
    #     odom = local_planner.get_odom()
    #     yaw = local_planner.yaw_from_geom_quat(odom.pose.pose.orientation)
    #     rospy.loginfo("Odom yaw : %s [rad], %s [deg]", yaw, math.degrees(yaw))
    #     rate.sleep()
        
    #slocal_planner.move_robot("forward")