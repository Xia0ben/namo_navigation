#! /usr/bin/env python

# One reason why reprogramming the whole ROS navigation Stack :
# https://answers.ros.org/question/174067/possible-to-write-move_base-plugin-in-python/

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sortedcontainers import SortedDict
import math
import copy
import tf
import numpy

# My own Classes
from global_planner import GlobalPlanner
from local_planner import LocalPlanner
from map_manager import MapManager
from plan import Plan
from utils import Utils

class NavManager:

    def __init__(self):
        # Get parameters
        self.static_map_topic = "/map" # rospy.get_param('~map_topic')
        self.robot_radius = "0.5" # [m] rospy.get_param('~robot_radius')
        self.robot_fov_radius = "2.0" # [m] rospy.get_param('~robot_fov_radius')
        self.map_frame = "/map" # rospy.get_param('~map_frame')
        self.robot_frame = "/base_link" # rospy.get_param('~robot_frame')
        self.move_cost = 1.0 # rospy.get_param('~move_cost')
        self.push_cost = 1.0 # rospy.get_param('~push_cost')
        self.xy_goal_tolerance = 0.10 # [m] rospy.get_param('~xy_goal_tolerance')
        self.yaw_goal_tolerance = 0.03 # [rad] rospy.get_param('~yaw_goal_tolerance')
        self.one_push_distance = 0.05 # [m] rospy.get_param('~push_distance')

        # Declare common parameters
        self.map_manager = MapManager(self.robot_radius, self.robot_fov_radius, self.map_frame, self.static_map_topic)
        self.global_planner = GlobalPlanner()
        self.local_planner = LocalPlanner()

    def _static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map

    def _is_goal_app_reached(self, current_pose, goal_pose):
        distance_to_goal = Utils.distance_between_ros_poses(current_pose, goal_pose)

        if distance_to_goal < self.xy_goal_tolerance:
            return True
        return False

        # TODO May be necessary to check if goal orientation has been reached too
        # Use yaw_goal_tolerance param for that

    def make_and_execute_plan(self, goal_robot_pose):
        init_robot_pose = Utils.get_current_pose(self.map_frame, self.robot_frame)

        self._optimized(init_robot_pose, goal_robot_pose)

    def _get_cost_at_index(self, sorted_dict, index):
        try:
            return sorted_dict.peekitem(index)[1]
        except IndexError:
            # If cost not found at index in dict, it means
            return float('inf')

    def _get_obstacle_at_index(self, sorted_dict, index):
        try:
            return sorted_dict.peekitem(index)[0]
        except IndexError as e:
            raise e

    def _apply_translation_to_pose(self, pose, translation_vector):
        new_pose = copy.deepcopy(pose)
        new_pose.pose.x = new_pose.pose.x + translation_vector[0]
        new_pose.pose.y = new_pose.pose.y + translation_vector[1]
        return new_pose

    def _get_safe_swept_area(self, obstacle, translation_vector, occupancy_grid):
        manipulation_area_map_points = obstacle.get_manipulation_area_map_points()

        # If any manipulation area map point is occupied by an obstacle, then
        # return an empty set because the manipulation area is not a safe swept
        # area. Otherwise, return all the points as safe swept points.
        for point in manipulation_area_map_points:
            if occupancy_grid[point[0]][point[1]] == Utils.LETHAL_COST_ROS
                return set()
        return manipulation_area_map_points

    def _execute_plan(self, optimal_plan):
        self.local_planner.follow_path(optimal_plan)

    def _optimized(self, init_robot_pose, goal_robot_pose):
        # Current state
        current_robot_pose = init_robot_pose
        navigation_map = self.map_manager.get_map_copy()
        euclidean_cost_L = SortedDict()
        min_cost_L = SortedDict()
        optimal_plan = Plan.from_path(
            self.global_planner.make_plan(navigation_map.merged_occ_grid, init_robot_pose, goal_robot_pose),
            is_manipulation = False, resolution = navigation_map.resolution,
            move_cost = self.move_cost, push_cost = self.push_cost)
        is_moving = False

        # While check goal is reached
        while not _is_goal_app_reached(current_robot_pose, goal_robot_pose):
            # Original algorithm only checks for intersection with new obstacles
            # But actually, if we use an occupancy grid to check whether the
            # path crosses any obstacle, it doesn't change any functionnality
            # and is not any more costly. Actually, it is less of a pain to
            # implement.
            if (optimal_plan.intersects_with_obstacles(navigation_map.merged_occ_grid)):
                if navigation_map.has_free_space_been_created():
                    min_cost_L.clear()

                for current_obstacle in navigation_map.obstacles.values():
                    euclidean_cost_L[current_obstacle] = Utils.distance_between_ros_poses(
                        goal_robot_pose, current_obstacle.ros_pose))

                optimal_plan = Plan.from_path(
                    self.global_planner.make_plan(navigation_map.merged_occ_grid, current_robot_pose, goal_robot_pose),
                    is_manipulation = False, resolution = navigation_map.resolution,
                    move_cost = self.move_cost, push_cost = self.push_cost)

                index_EL, index_ML = 0, 0

                # Note: min_cost_L.peekitem(index_ML)[0] -> returns the obstacle
                # Note: min_cost_L.peekitem(index_ML)[1] -> returns the estimated cost
                # While we need to evaluate obstacles
                while (min(self._get_cost_at_index(min_cost_L, index_ML), self._get_cost_at_index(euclidean_cost_L, index_EL)) < optimal_plan.cost):
                    if (self._get_cost_at_index(min_cost_L, index_ML) < self._get_cost_at_index(euclidean_cost_L, index_EL)):
                        current_plan = self._opt_evaluate_action(self._get_obstacle_at_index(min_cost_L, index_ML), optimal_plan, navigation_map, current_robot_pose, goal_robot_pose)

                        if (current_plan is not None):
                            min_cost_L[self._get_obstacle_at_index(min_cost_L, index_ML)] = current_plan.min_cost
                            index_ML = index_ML + 1

                            if (current_plan.cost < optimal_plan.cost):
                                optimal_plan = current_plan

                    else
                        try:
                            # If the min_cost_L doesn't contain the obstacle, do except
                            min_cost_L.index(self._get_obstacle_at_index(euclidean_cost_L, index_EL))
                        except IndexError as exception:
                            current_plan = self._opt_evaluate_action(self._get_obstacle_at_index(euclidean_cost_L,
                                index_EL), optimal_plan, navigation_map, current_robot_pose, goal_robot_pose)

                            if (current_plan is not None):
                                min_cost_L[self._get_obstacle_at_index(euclidean_cost_L,
                                    index_EL)] = current_plan.min_cost
                                index_ML = index_ML + 1

                                if (current_plan.cost < optimal_plan.cost):
                                    optimal_plan = current_plan

                    index_EL = index_EL + 1
                    # Endwhile we need to evaluate obstacles

            if not is_moving:
                self._execute_plan(optimal_plan)
                is_moving = True

            # R \gets$ Next step in $optimal_plan$ FIXME Have the robot advance one step in the plan

            current_robot_pose = Utils.get_current_pose(self.map_frame, self.robot_frame)
            navigation_map = self.map_manager.get_map_copy()

            # Endwhile check goal is reached


    def _opt_evaluate_action(self, obstacle, optimal_plan, navigation_map, current_robot_pose, goal_robot_pose):
        paths_set = SortedDict()  # Use SortedDict rather than set or array for ease of use
        blocking_areas = None

        # For easier implementation, we iterate over push pose rather than
        # an abstract concept of "direction"
        for push_pose in obstacle.push_poses
            push_pose_yaw = Utils.yaw_from_geom_quat(push_pose.pose.orientation)
            push_pose_unit_direction_vector = (math.cos(push_pose_yaw), math.sin(push_pose_yaw))

            # Computation of c1 is done here (a contrario to the original algorithm)
            # because it is in fact dependent of the push point (if the push point
            # cannot be reached, we imediately know it)
            c1 = Plan.from_path(
                self.global_planner.make_plan(navigation_map.merged_occ_grid, current_robot_pose, push_pose),
                is_manipulation = False, resolution = navigation_map.resolution,
                move_cost = self.move_cost, push_cost = self.push_cost)
            # If no path to the pushing point is found, then study next point
            if (c1 is not None):
                seq = 1
                c3_est = Plan.from_poses(obstacle_sim_pose, goal_robot_pose,
                            navigation_map.resolution, isManipulation = False,
                            move_cost = self.move_cost, push_cost = self.push_cost)
                cEst = c1.cost + c3_est.cost + seq * self.push_cost
                translation_vector = (push_pose_unit_direction_vector[0] * one_push_distance * float(seq),
                    push_pose_unit_direction_vector[1] * one_push_distance * float(seq))
                obstacle_sim_pose = self._apply_translation_to_pose(push_pose, translation_vector)
                safe_swept_area = self._get_safe_swept_area(obstacle, translation_vector, navigation_map.merged_occ_grid)

                # If safe_swept_area is empty, it is evaluated to False
                while (bool(safe_swept_area) and cEst <= optimal_plan.cost):
                    if (self._check_new_opening(navigation_map, obstacle, translation_vector, blocking_areas)):
                        c2 = Plan.from_poses(push_pose, obstacle_sim_pose,
                            navigation_map.resolution, isManipulation = False,
                            move_cost = self.move_cost, push_cost = self.push_cost)
                        c3 = Plan.from_path(self.global_planner.make_plan(navigation_map.merged_occ_grid, obstacle_sim_pose, goal_robot_pose),
                            is_manipulation = False, resolution = navigation_map.resolution,
                            move_cost = self.move_cost, push_cost = self.push_cost)
                        # Costs are computed within the class
                        current_plan = Plan.from_plans(c1, c2, c3)

                    seq = seq + 1

                    c3_est = Plan.from_poses(obstacle_sim_pose, goal_robot_pose,
                                navigation_map.resolution, isManipulation = True,
                                move_cost = self.move_cost, push_cost = self.push_cost)

                    cEst = c1.cost + c3_est.cost + seq * self.push_cost
                    translation_vector = (push_pose_unit_direction_vector[0] * one_push_distance * float(seq),
                        push_pose_unit_direction_vector[1] * one_push_distance * float(seq))
                    obstacle_sim_pose = self._apply_translation_to_pose(obstacle_sim_pose, translation_vector)
                    safe_swept_area = self._get_safe_swept_area(obstacle, translation_vector, navigation_map.merged_occ_grid)

        # Return path with lowest cost : the first in the ordered dictionnary
        # If there is none, return None
        try:
            return paths_set.peekitem(0)
        except IndexError:
            return None

    def _check_new_opening(self, navigation_map, obstacle, translation_vector, blocking_areas):
        obstacle_matrix = obstacle.obstacle_matrix["matrix"]
        x_offset_before = obstacle.obstacle_matrix["top_left_corner"][0]
        y_offset_before = obstacle.obstacle_matrix["top_left_corner"][1]

        if (blocking_areas is None):
            blocking_areas = self.get_blocking_areas(
                x_offset_before, y_offset_before, obstacle_matrix, navigation_map.merged_occ_grid)

        x_offset_after, y_offset_after = self.get_new_pos(translation_vector, navigation_map.info.resolution, x_offset_before, y_offset_before)
        blocking_areas_after_action = self.get_blocking_areas(
            x_offset_after, y_offset_after, obstacle_matrix, navigation_map.merged_occ_grid)
        shifted_blocking_areas = [
            [0 for k in range(len(obstacle_matrix))] for l in range(len(obstacle_matrix[k]))]

        for i in range(len(shifted_blocking_areas)):
            for j in range(len(shifted_blocking_areas[i])):
                x = (x_offset_after - x_offset_before) + i
                y = (y_offset_after - y_offset_before) + j

                if (0 < x < len(shifted_blocking_areas) and 0 < y < len(shifted_blocking_areas[x])):
                    shifted_blocking_areas[x][y] = blocking_areas_after_action[i][j]

        Z = self.compare(blocking_areas, shifted_blocking_areas)
        if self.is_zero_matrix(Z)
            return False
        return True


    def get_blocking_areas(self, x_offset, y_offset, obstacle_matrix, grid):
        index = 1
        blocking_areas = [[0 for k in range(len(obstacle_matrix))] for l in range(len(obstacle_matrix[k]))]

        for i in range(len(blocking_areas)):
            for j in range(len(blocking_areas[i])):
                if (obstacle_matrix[x][y] != 0 and grid[x + x_offset][y + y_offset] != 0):
                    self.assign_nr(blocking_areas, x, y, index)

        return blocking_areas


    def assign_nr(self, blocking_areas, x, y, index):
        # Note : range(-1, 2) = [-1, 0, 1]
        for i in range(-1, 2):
            for j in range(-1, 2):
                if (blocking_areas[x + i][y + j] != 0):
                    blocking_areas[x][y] = blocking_areas[x + i][x + j]
                    return
        blocking_areas[x][y] = index
        index = index + 1
        return


    def compare(self, blocking_areas_before, blocking_areas_after):
        comparison_matrix = copy.deepcopy(blocking_areas_after)
        del_num = set()

        for x in range(len(comparison_matrix)):
            for y in range(len(comparison_matrix[x])):

                if (blocking_areas_before[x][y] in del_num):
                    comparison_matrix[x][y] = 0
                if (blocking_areas_before[x][y] != 0 and
                        blocking_areas_after[x][y] != 0):
                    del_num = del_num.union(comparison_matrix[x][y])
                    comparison_matrix[x][y] = 0

        return comparison_matrix

    # Helper function for the Efficient Local Opening Detection Algorithm (ELODA)
    def get_new_pos(self, translation_vector, resolution, x_offset, y_offset):
        return (x_offset + int(translation_vector[0] / resolution), y_offset + int(translation_vector[1] / resolution))

    def is_zero_matrix(self, Z):
        for i in range(len(Z)):
            for j in range(len(Z[i])):
                if Z[i][j] != 0
                    return False
        return True
