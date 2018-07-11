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
from multilayered_map import MultilayeredMap
from plan import Plan

class NavManager:

    def __init__(self):
        # Get parameters
        self.static_map_topic = "/map" # rospy.get_param('~map_topic')
        self.robot_diameter = "0.5" # [m] rospy.get_param('~robot_diameter')
        self.map_frame = "/map" # rospy.get_param('~map_frame')
        self.robot_frame = "/base_link" # rospy.get_param('~robot_frame')
        self.move_cost = 1.0 # rospy.get_param('~move_cost')
        self.push_cost = 1.0 # rospy.get_param('~push_cost')
        self.xy_goal_tolerance = 0.10 # [m] rospy.get_param('~xy_goal_tolerance')
        self.yaw_goal_tolerance = 0.03 # [rad] rospy.get_param('~yaw_goal_tolerance')

        # Subscribe to necessary topics
        self.tf_listener = tf.TransformListener()
        self.static_map_sub = rospy.Subscriber(self.static_map_topic, OccupancyGrid, self.static_map_callback)

        # Declare common parameters
        self.static_map = None
        self.navigation_map = None
        self.global_planner = GlobalPlanner()
        self.local_planner = LocalPlanner()

        # Initialize map
        while self.static_map is None:
            rospy.sleep(0.2)
        self.navigation_map = MultilayeredMap(self.static_map, self.robot_diameter)


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

    def make_and_execute_plan(self, goal_robot_poseParam):
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

    def _execute_plan(self, optimal_plan):
        self.local_planner.follow_path(optimal_plan)

    def _optimized(init_robot_pose, goal_robot_pose):
        # Current state
        current_robot_pose = init_robot_pose
        euclidean_cost_L = SortedDict()
        min_cost_L = SortedDict()
        optimal_plan = Plan.from_path(
            self.global_planner.make_plan(self.navigation_map.merged_occ_grid, init_robot_pose, goal_robot_pose),
            is_manipulation = False, resolution = self.navigation_map.resolution,
            move_cost = self.move_cost, push_cost = self.push_cost)
        is_moving = False

        # While check goal is reached
        while not _is_goal_app_reached(current_robot_pose, goal_robot_pose):
            # Original algorithm only checks for intersection with new obstacles
            # But actually, if we use an occupancy grid to check whether the
            # path crosses any obstacle, it doesn't change any functionnality
            # and is not any more costly. Actually, it is less of a pain to
            # implement.
            if (optimal_plan.intersects_with_obstacles(self.navigation_map.merged_occ_grid)):
                if self.navigation_map.has_free_space_been_created():
                    min_cost_L.clear()

                for current_obstacle in self.navigation_map.obstacles:
                    euclidean_cost_L[current_obstacle] = Utils.distance_between_ros_poses(
                        goal_robot_pose, goal_pose, current_obstacle.pose))

                optimal_plan = Plan.from_path(
                    self.global_planner.make_plan(self.navigation_map.merged_occ_grid, current_robot_pose, goal_robot_pose),
                    is_manipulation = False, resolution = self.navigation_map.resolution,
                    move_cost = self.move_cost, push_cost = self.push_cost)

                index_EL, index_ML = 0, 0

                # Note: min_cost_L.peekitem(index_ML)[0] -> returns the obstacle
                # Note: min_cost_L.peekitem(index_ML)[1] -> returns the estimated cost
                # While we need to evaluate obstacles
                while (min(self._get_cost_at_index(min_cost_L, index_ML), self._get_cost_at_index(euclidean_cost_L, index_EL)) < optimal_plan.cost):
                    if (self._get_cost_at_index(min_cost_L, index_ML) < self._get_cost_at_index(euclidean_cost_L, index_EL)):
                        current_plan = _opt_evaluate_action(self._get_obstacle_at_index(min_cost_L, index_ML), optimal_plan, map, current_robot_pose, goal_robot_pose)

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
                            current_plan = _opt_evaluate_action(self._get_obstacle_at_index(euclidean_cost_L,
                                index_EL), optimal_plan, map, current_robot_pose, goal_robot_pose)

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

            current_robot_pose = Utils.get_current_pose(self.map_frame, self.robot_frame)
            # R \gets$ Next step in $optimal_plan$ FIXME Have the robot advance one step in the plan

            # Endwhile check goal is reached


    def _opt_evaluate_action(obstacle, optimal_plan, map, current_robot_pose, goal_robot_pose):
        paths_set = SortedDict()  # Use sortedDict rather than set or array for ease of use
        blocking_areas = None

        # For easier implementation, we iterate over grasp points rather than
        # an abstract concept of "direction"
        for graspPoint in obstacle.graspPoints
            one_push_in_d = obstacle.initPose.pos - graspoint  # FIXME ADD CONVERSION FROM TUPLE TO ARRAY
            # FIXME put the right value here
            firstRobotPose = copy.deepcopy(current_robot_pose)

            # Computation of c1 is done here (a contrario to the original algorithm)
            # because it is in fact dependent of the grasp point (if the grasp point
            # cannot be reached, we imediately know it)
            c1 = Plan.from_path(
                self.global_planner.make_plan(self.navigation_map.merged_occ_grid, current_robot_pose, firstRobotPose),
                is_manipulation = False, resolution = self.navigation_map.resolution,
                move_cost = self.move_cost, push_cost = self.push_cost)
            # If no path to the grasping point is found, then study next point
            if (c1 is not None):
                obstacle.simPose = obstacle.initPose
                seq = 1
                # FIXME so that the start position is not the center of the object but the grasping point
                c3_est = Plan.from_poses(obstacle.simPose, goal_robot_pose,
                            self.navigation_map.resolution, isManipulation = False,
                            move_cost = self.move_cost, push_cost = self.push_cost)
                cEst = c1.cost + c3_est.cost + seq * self.push_cost

                while (obstacle.pushUsingGraspPointPossible(graspPoint, map) and cEst <= optimal_plan.cost):
                    if (_check_new_opening(map, obstacle, [one_push_in_d], blocking_areas)):
                        obstacle.simPose.pos = obstacle.simPose.pos + one_push_in_d
                        # FIXME both points
                        c2 = Plan.from_poses(obstacle.initPose, obstacle.simPose,
                            self.navigation_map.resolution, isManipulation = False,
                            move_cost = self.move_cost, push_cost = self.push_cost)
                        c3 = Plan.from_path(self.global_planner.make_plan(self.navigation_map.merged_occ_grid, obstacle.simPose, goal_robot_pose),
                            is_manipulation = False, resolution = self.navigation_map.resolution,
                            move_cost = self.move_cost, push_cost = self.push_cost)  # FIXME origin point
                        # Costs are computed within the class
                        current_plan = Plan.from_plans(c1, c2, c3)
                seq = seq + 1

                # FIXME so that the start position is not the center of the object but the grasping point
                c3_est = Plan.from_poses(obstacle.simPose, goal_robot_pose,
                            self.navigation_map.resolution, isManipulation = True,
                            move_cost = self.move_cost, push_cost = self.push_cost)

                cEst = c1.cost + c3_est.cost + seq * self.push_cost

        # Return path with lowest cost : the first in the ordered dictionnary
        # If there is none, return None
        try:
            return paths_set.peekitem(0)
        except IndexError:
            return None

    # It may be interesting to move the following methods into a proper class


    def _check_new_opening(map, obstacle, action, blocking_areas):
        M = getObstacleMatrix(map, obstacle)
        xOffset = obstacle.simPose.pos[0]
        yOffset = obstacle.simPose.pos[1]

        if (blocking_areas is None):
            blocking_areas = get_blocking_areas(
                xOffset, yOffset, M, map.mergedBinaryOccupationGrid)

        newPos = get_new_pos(action, obstacle)
        xOffset, yOffset = newPos[0], newPos[1]
        blocking_areasAfterAction = get_blocking_areas(
            xOffset, yOffset, M, map.mergedBinaryOccupationGrid)
        shiftedBlockingAreas = [
            [0 for k in range(len(M))] for l in range(len(M[k]))]

        for i in range(len(shiftedBlockingAreas)):
            for j in range(len(shiftedBlockingAreas[i])):
                x = (xOffset - obstacle.simPose.pos[0]) + i
                y = (yOffset - obstacle.simPose.pos[1]) + j

                if (0 < x < len(shiftedBlockingAreas) and 0 < y < len(shiftedBlockingAreas[x])):
                    shiftedBlockingAreas[x][y] = blocking_areasAfterAction[i][j]

        Z = compare(blocking_areas, shiftedBlockingAreas)
        if is_zero_matrix(Z)
            return False
        return True


    def get_blocking_areas(xOffset, yOffset, M, grid):
        index = 1
        blocking_areas = [[0 for k in range(len(M))] for l in range(len(M[k]))]

        for i in range(len(blocking_areas)):
            for j in range(len(blocking_areas[i])):
                if (M[x][y] != 0 and grid[x + xOffset][y + yOffset] != 0):
                    assign_nr(blocking_areas, x, y, index)

        return blocking_areas


    def assign_nr(blocking_areas, x, y, index):
        # Note : range(-1, 2) = [-1, 0, 1]
        for i in range(-1, 2):
            for j in range(-1, 2):
                if (blocking_areas[x + i][y + j] != 0):
                    blocking_areas[x][y] = blocking_areas[x + i][x + j]
                    return
        blocking_areas[x][y] = index
        index = index + 1
        return


    def compare(blocking_areasBefore, blocking_areasAfter):
        comparisonMatrix = copy.deepcopy(blocking_areasAfter)
        delNum = set()

        for x in range(len(comparisonMatrix)):
            for y in range(len(comparisonMatrix[x])):

                if (blocking_areasBefore[x][y] in delNum):
                    comparisonMatrix[x][y] = 0
                if (blocking_areasBefore[x][y] != 0 and
                        blocking_areasAfter[x][y] != 0):
                    delNum = delNum.union(comparisonMatrix[x][y])
                    comparisonMatrix[x][y] = 0

        return comparisonMatrix

    # Helper functions for the Efficient Local Opening Detection Algorithm (ELODA)

    # action is an array of 2D vectors (2 items-arrays)
    def get_new_pos(action, obstacle):
        action = [[20.0, 30.0]]
        newPos = [obstacle.simPose.pos[0], obstacle.simPose.pos[1]]

        for component in action:
            newPos = [newPos[0] + component[0], newPos[1] + component[1]]

        return newPos


    def is_zero_matrix(Z):
        for i in range(len(Z)):
            for j in range(len(Z[i])):
                if Z[i][j] != 0
                    return False
        return True
