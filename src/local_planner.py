#! /usr/bin/env python

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from utils import Utils

import actionlib

from namo_navigation.msg import FollowPathAction, FollowPathActionFeedback, FollowPathActionGoal, FollowPathActionResult, FollowPathFeedback, FollowPathGoal, FollowPathResult

class LocalPlanner(object):
    VEL_TOPIC = '/cmd_vel'
    MAX_LINEAR_SPEED = 0.3
    MAX_ANGULAR_SPEED = 0.5

    UNIT_VECTOR = np.array([1, 0])
    ANGLE_TOLERANCE = 0.03 # [rad] ~= 2 [deg]
    DISTANCE_TOLERANCE = 0.05 # [m]

    def __init__(self):
        self.map_frame = "/map" # rospy.get_param('~map_frame')
        self.robot_frame = "/base_link" # rospy.get_param('~robot_frame')

        self.tf_listener = tf.TransformListener()

        # Init cmd_vel pub
        self._cmd_vel_pub = rospy.Publisher(LocalPlanner.VEL_TOPIC, Twist, queue_size=1)
        self._twist_object = Twist()

        # Creates the action server
        self._follow_path_as = actionlib.SimpleActionServer("follow_path_as", FollowPathAction, self.goal_callback, False)
        self._follow_path_as.start()

        rospy.loginfo("Local planner inited")

    def goal_callback(self, goal):
        self.follow_path(goal.path)

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


    # TODO Merge turn_in_place and move_linearly as move_to_target and use a
    # boolean to switch between the two behaviours
    def turn_in_place(self, target_yaw, angle_tolerance, turn_direction):
        # Start turning
        #rospy.loginfo("Start turning to target yaw : %s", target_yaw)
        self.move_robot(turn_direction)

        # Wait for target angle to be attained then break
        while True:
            current_pose = Utils.get_current_pose(self.map_frame, self.robot_frame)
            current_yaw = Utils.yaw_from_geom_quat(current_pose.pose.orientation)

            if self._follow_path_as.is_preempt_requested():
              rospy.loginfo('The goal has been cancelled/preempted')
              # the following line, sets the client in preempted state (goal cancelled)
              self._follow_path_as.set_preempted()
              break

            # If current yaw is close to target yaw
            if np.isclose(target_yaw, current_yaw, atol = angle_tolerance):
                break

        # Stop turning
        self.move_robot("stop")

        return True # TODO check if rotation actually succeeds, and if not, return False

    def move_linearly(self, distance, init_pose, distance_tolerance, move_direction):
        # Start moving
        #rospy.loginfo("Start moving to target point : %s", target_position)
        self.move_robot(move_direction)

        # Wait for target position to be attained then break
        while True:
            current_pose = Utils.get_current_pose(self.map_frame, self.robot_frame)
            traveled_distance = np.linalg.norm(np.array([current_pose.pose.position.x, current_pose.pose.position.y]) -
                np.array([init_pose.pose.position.x, init_pose.pose.position.y]))

            if self._follow_path_as.is_preempt_requested():
              rospy.loginfo('The goal has been cancelled/preempted')
              # the following line, sets the client in preempted state (goal cancelled)
              self._follow_path_as.set_preempted()
              break

            # If current position is close to target position
            if np.isclose(traveled_distance, distance, atol = distance_tolerance):
                break

        # Stop turning
        self.move_robot("stop")
        #rospy.loginfo("Stop moving to target point : %s", target_position)

        return True # TODO check if move actually succeeds, and if not, return False

    def robot_local_point(self, pose_stamped):
        # TODO transform locally : may require adjustments
        # point_translated = (point[0] - robot_pos[0], point[1] - robot_pos[1])
        # point_translated_rotated = (point_translated[0] * math.cos(theta) - point_translated[1] * math.sin(theta),
        #                             point_translated[0] * math.sin(theta) + point_translated[1] * math.cos(theta))

        while True:
            try:
                local_pose = self.tf_listener.transformPose(self.robot_frame, pose_stamped)
                break
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass

        return local_pose

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
        feedback = FollowPathFeedback()

        rospy.loginfo("Start of path following...")
        for current_subgoal in path.poses:
            rospy.loginfo("Going to current_subgoal : %s", current_subgoal)
            feedback.current_subgoal = current_subgoal
            self._follow_path_as.publish_feedback(feedback)

            current_pose = Utils.get_current_pose(self.map_frame, self.robot_frame)
            current_yaw = Utils.yaw_from_geom_quat(current_pose.pose.orientation)

            current_subgoal_local = self.robot_local_point(current_subgoal)
            rospy.loginfo("Local current_subgoal : %s", current_subgoal_local)

            # If the current_subgoal is on the left side of the robot, go left, else the reverse
            if current_subgoal_local.pose.position.y > 0:
                turn_direction = "left"
            else:
                turn_direction = "right"

            local_target_direction = np.array([current_subgoal_local.pose.position.x, current_subgoal_local.pose.position.y])
            local_target_direction_norm = np.linalg.norm(local_target_direction)

            if current_subgoal_local.pose.position.x < 0:
                rospy.loginfo("Complement acos because we are outside of range 0 to 180 deg.")
                target_local_yaw = math.pi - math.acos(np.dot(local_target_direction, LocalPlanner.UNIT_VECTOR) /
                    local_target_direction_norm)
            else:
                rospy.loginfo("Don't complement acos.")
                target_local_yaw = math.acos(np.dot(local_target_direction, LocalPlanner.UNIT_VECTOR) /
                    local_target_direction_norm)

            target_yaw = self.modulo_pi(current_yaw + target_local_yaw)
            target_distance = local_target_direction_norm
            rospy.loginfo("Target local yaw : %s [rad], %s [deg]", target_local_yaw, math.degrees(target_local_yaw))
            rospy.loginfo("Target yaw : %s [rad], %s [deg]", target_yaw, math.degrees(target_yaw))

            # Turn then proceed forward
            self.turn_in_place(target_yaw, LocalPlanner.ANGLE_TOLERANCE, turn_direction)
            self.move_linearly(target_distance, current_pose, LocalPlanner.DISTANCE_TOLERANCE, "forward")
        rospy.loginfo("End of path following...")
        self._follow_path_as.set_succeeded()

if __name__ == "__main__":
    rospy.init_node('local_planner', log_level=rospy.INFO)
    local_planner = LocalPlanner()
    
    # Test code :
    # PATH = [(1, 0), (1, 1), (0, 1), (0, 0)] # Test variable for debugging
    pub = rospy.Publisher('/follow_path_as/goal', FollowPathActionGoal, queue_size=1)
    test_goal = FollowPathActionGoal()
    
    pose_1 = PoseStamped()
    pose_1.pose.position.x = 1.0
    pose_1.pose.position.y = 0.0
    pose_2 = PoseStamped()
    pose_2.pose.position.x = 1.0
    pose_2.pose.position.y = 1.0
    pose_3 = PoseStamped()
    pose_3.pose.position.x = 0.0
    pose_3.pose.position.y = 1.0
    pose_4 = PoseStamped()
    pose_4.pose.position.x = 0.0
    pose_4.pose.position.y = 0.0
    
    test_goal.goal.path.poses = [pose_1, pose_2, pose_3, pose_4]
    
    
    while True:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(test_goal)
            break
    
    rospy.spin()
