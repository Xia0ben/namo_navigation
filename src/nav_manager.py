#! /usr/bin/env python

# One reason why reprogramming the whole ROS navigation Stack :
# https://answers.ros.org/question/174067/possible-to-write-move_base-plugin-in-python/

import rospy
from geometry_msgs.msg import PoseStamped
from sortedcontainers import SortedDict
import math
import copy
import numpy as np

# My own Classes
from global_planner import GlobalPlanner
from map_manager import MapManager
from path import Path
from plan import Plan
from utils import Utils

from namo_navigation.srv import LocalGoalPose, LocalGoalPoseRequest

class NavManager:

    def __init__(self, robot_radius, robot_fov_radius, static_map_topic, merged_occ_grid_topic, push_poses_topic,
                 goal_topic, simulated_pose_topic, simulated_fov_pointcloud_topic, local_goal_pose_topic):
        self.MOVE_COST = 1.0
        self.PUSH_COST = 1.0
        self.ONE_PUSH_DISTANCE = 0.05
        self.POSITION_TOLERANCE = 0.01 # [m]
        self.ANGLE_TOLERANCE = 0.02 # [rad]
        self.map_manager = MapManager(robot_radius, robot_fov_radius, static_map_topic, merged_occ_grid_topic,
                                      push_poses_topic, simulated_pose_topic, simulated_fov_pointcloud_topic)
        self.global_planner = GlobalPlanner()

        # Parameters
        self.current_robot_pose = None
        self.is_ready = False
        self.blocked_obstacles = set()
        self.debug_rate = rospy.Rate(4.0)

        self.goal_topic = goal_topic
        self.simulated_pose_topic = simulated_pose_topic
        self.local_goal_pose_topic = local_goal_pose_topic

        rospy.Subscriber(self.goal_topic, PoseStamped, self._global_goal_pose_callback)
        rospy.Subscriber(self.simulated_pose_topic, PoseStamped, self._simulated_pose_callback)
        rospy.wait_for_service(self.local_goal_pose_topic)
        self.local_goal_pose_client = rospy.ServiceProxy(self.local_goal_pose_topic, LocalGoalPose)

        self.local_goal_pose_pub = rospy.Publisher(self.local_goal_pose_topic, PoseStamped, queue_size=1)

    def _get_safe_swept_area(self, obstacle, translation_vector, occupancy_grid):
        # FIXME Don't just get manipulation area from obstacle polygon but also robot polygon, therefore, the method should move into MultilayeredMap class
        manipulation_area_map_points = obstacle.get_manipulation_area_map_points(translation_vector)

        # If any manipulation area map point is occupied by an obstacle, then
        # return None because the manipulation area is not a safe swept
        # area. Otherwise, return the set of all safe swept map points.
        for point in manipulation_area_map_points:
            if point not in obstacle.discretized_polygon and occupancy_grid[point[0]][point[1]] == Utils.ROS_COST_LETHAL:
                return None
        return manipulation_area_map_points

    def _apply_translation_to_pose(self, pose, translation_vector):
        new_pose = copy.deepcopy(pose)
        new_pose.pose.position.x = new_pose.pose.position.x + translation_vector[0]
        new_pose.pose.position.y = new_pose.pose.position.y + translation_vector[1]
        return new_pose

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

    def _is_same_pose(self, pose_a, pose_b):
        pose_a_yaw = Utils.yaw_from_geom_quat(pose_a.pose.orientation)
        pose_b_yaw = Utils.yaw_from_geom_quat(pose_b.pose.orientation)

        if (np.isclose(pose_a_yaw, pose_b_yaw, atol = self.ANGLE_TOLERANCE) and
                np.isclose(pose_a.pose.position.x, pose_b.pose.position.x, atol = self.POSITION_TOLERANCE) and
                np.isclose(pose_a.pose.position.y, pose_b.pose.position.y, atol = self.POSITION_TOLERANCE)):
            return True
        # Else
        return False

    def robot_goto(self, pose, is_manipulation):
        pose.header.stamp = rospy.Time.now()
        request = LocalGoalPoseRequest()
        request.local_pose_goal = pose
        request.is_manipulation = is_manipulation
        result = self.local_goal_pose_client(request)
        return result.real_pose

    def _simulated_pose_callback(self, pose):
        self.current_robot_pose = pose
        if not self.is_ready:
            self.is_ready = True

    def _global_goal_pose_callback(self, goal_pose):
        self.make_and_execute_plan_caller(goal_pose)

    def manually_set_goal_robot_pose_from_map_coordinates_yaw(self, coordX, coordY, yaw):
        goal_pose = Utils.ros_pose_from_map_coord_yaw(coordX, coordY, yaw, self.map_manager.multilayered_map.info.resolution, 1, self.map_manager.multilayered_map.frame_id)
        self.make_and_execute_plan_caller(goal_pose)

    def make_and_execute_plan_caller(self, r_goal):
        Utils.publish_goal_pose_for_display(r_goal)
        success = self.make_and_execute_plan(self.current_robot_pose, r_goal, self.blocked_obstacles)
        if success:
            rospy.loginfo("Goal successfully reached.")
        else:
            rospy.logerr("Could not reach goal even after considering all available options.")

    def make_and_execute_plan(self, r_init, r_goal, blocked_obstacles):
        r_cur = r_init
        is_manip_success = True
        blocked_obstacles = blocked_obstacles
        eu_cost_l, min_cost_l = SortedDict(), SortedDict()

        map_cur = self.map_manager.get_init_map()

        p_opt = [Plan([Path.from_path(
            self.global_planner.make_plan(map_cur, r_cur, r_goal), False, self.MOVE_COST)])]

        Utils.publish_plan(p_opt[0])

        while not self._is_same_pose(r_cur, r_goal):
            map_cur = self.map_manager.get_map_copy()

            if map_cur.has_free_space_been_created():
                min_cost_l.clear()

            obstacles = map_cur.obstacles

            is_push_pose_valid = True
            is_obstacle_same = True

            if p_opt[0].obstacle is not None:
                is_obstacle_same = p_opt[0].obstacle.compare(obstacles[p_opt[0].obstacle.obstacle_id])
                if not is_obstacle_same:
                    p_opt[0].obstacle = obstacles[p_opt[0].obstacle.obstacle_id]
                    p_opt[0].safe_swept_area = self._get_safe_swept_area(p_opt[0].obstacle, p_opt[0].translation_vector, map_cur.merged_occ_grid)
                    if p_opt[0].next_step_component == Plan.C1 and not p_opt[0].obstacle.is_obstacle_push_pose(p_opt[0].push_pose):
                        is_push_pose_valid = False

            is_plan_not_colliding = not p_opt[0].is_plan_colliding(map_cur)

            is_plan_valid = is_plan_not_colliding and is_push_pose_valid and is_manip_success

            if not is_plan_valid:
                p_opt = [Plan([Path.from_path(
                    self.global_planner.make_plan(map_cur, r_cur, r_goal), False, self.MOVE_COST)])]
                Utils.publish_plan(p_opt[0])
                self.make_plan(r_cur, r_goal, map_cur, obstacles, blocked_obstacles, p_opt, eu_cost_l, min_cost_l)
                Utils.debug_erase_partial_paths()
                Utils.publish_plan(p_opt[0])
                pass

            # Stop condition if no plan is found
            if p_opt[0].cost >= float("inf"):
                return False

            is_manip_success = True
            r_next = p_opt[0].get_next_step()
            c_next = p_opt[0].next_step_component
            is_manipulation = c_next == Plan.C2
            r_real = self.robot_goto(r_next, is_manipulation)
            if is_manipulation and not self._is_same_pose(r_real, r_next):
                is_manip_success = False
                blocked_obstacles.add(p_opt[0].obstacle.obstacle_id)
            r_cur = r_real
            self.debug_rate.sleep()

        # Endwhile
        return True

    def make_plan(self, r_cur, r_goal , map_cur, obstacles, blocked_obstacles, p_opt, eu_cost_l, min_cost_l):
        for obstacle in obstacles.values():
            c3_est = float("inf")
            for push_pose in obstacle.push_poses:
                c3_est = min(c3_est, Utils.euclidean_distance_ros_poses(push_pose, r_goal))
            eu_cost_l[obstacle] = c3_est

        index_EL, index_ML = 0, 0
        evaluated_obstacles = set()

        while (min(self._get_cost_at_index(min_cost_l, index_ML), self._get_cost_at_index(eu_cost_l, index_EL)) < p_opt[0].cost):
            if self._get_cost_at_index(min_cost_l, index_ML) < self._get_cost_at_index(eu_cost_l, index_EL):
                evaluated_obstacle = self._get_obstacle_at_index(min_cost_l, index_ML)
                if evaluated_obstacle not in evaluated_obstacles:
                    p = self.plan_for_obstacle(evaluated_obstacle, p_opt, map_cur, r_cur, r_goal, blocked_obstacles)
                    if p is not None:
                        min_cost_l[evaluated_obstacle] = p.min_cost
                    else:
                        min_cost_l[evaluated_obstacle] = float("inf")
                    evaluated_obstacles.add(evaluated_obstacle)
                index_ML = index_ML + 1
            else:
                evaluated_obstacle = self._get_obstacle_at_index(eu_cost_l, index_EL)
                if evaluated_obstacle not in evaluated_obstacles:
                    try:
                        # If the min_cost_L doesn't contain the obstacle, do except
                        min_cost_l.index(evaluated_obstacle)
                    except ValueError:
                        p = self.plan_for_obstacle(
                            evaluated_obstacle,
                            p_opt, map_cur, r_cur, r_goal, blocked_obstacles)
                        if p is not None:
                            min_cost_l[evaluated_obstacle] = p.min_cost
                        else:
                            min_cost_l[evaluated_obstacle] = float("inf")
                        evaluated_obstacles.add(evaluated_obstacle)
                index_EL = index_EL + 1

    def plan_for_obstacle(self, obstacle, p_opt, map_cur, r_cur, r_goal, blocked_obstacles):
        if obstacle.obstacle_id in blocked_obstacles:
            return None

        paths = SortedDict()

        for push_pose in obstacle.push_poses:
            push_pose_yaw = Utils.yaw_from_geom_quat(push_pose.pose.orientation)
            push_pose_unit_direction_vector = (math.cos(push_pose_yaw), math.sin(push_pose_yaw))

            c1 = Path.from_path(
                self.global_planner.make_plan(map_cur, r_cur, push_pose, True), False, self.MOVE_COST)

            Utils.debug_publish_path(c1, "/simulated/debug_c1")

            # If c1 does not exist
            if not c1.path.poses:
                continue

            seq = 1
            translation_vector = (push_pose_unit_direction_vector[0] * self.ONE_PUSH_DISTANCE * float(seq),
                                  push_pose_unit_direction_vector[1] * self.ONE_PUSH_DISTANCE * float(seq))
            safe_swept_area = self._get_safe_swept_area(obstacle, translation_vector, map_cur.merged_occ_grid)
            obstacle_sim_pose = self._apply_translation_to_pose(push_pose, translation_vector)
            c3_est = Path.from_poses(obstacle_sim_pose, r_goal,
                                     False, self.MOVE_COST, push_pose_unit_direction_vector, self.ONE_PUSH_DISTANCE, map_cur.frame_id)
            Utils.debug_publish_path(c3_est, "/simulated/debug_c3")
            # TODO Make PUSH_COST an attribute of the object that is initialized to 1.0 until object is identified.
            c_est = c1.cost + c3_est.cost + self.PUSH_COST * Utils.euclidean_distance(translation_vector, (0.0, 0.0))

            while c_est <= p_opt[0].cost and safe_swept_area is not None:
                c2 = Path.from_poses(push_pose, obstacle_sim_pose,
                                     True, self.PUSH_COST, push_pose_unit_direction_vector, self.ONE_PUSH_DISTANCE, map_cur.frame_id)
                Utils.debug_publish_path(c2, "/simulated/debug_c2")
                map_mod = copy.deepcopy(map_cur)
                map_mod.manually_move_obstacle(obstacle.obstacle_id, translation_vector)
                c3 = Path.from_path(
                    self.global_planner.make_plan(map_mod, obstacle_sim_pose, r_goal), False, self.MOVE_COST)
                Utils.debug_publish_path(c3, "/simulated/debug_c3")
                # If c3 exists
                if c3.path.poses:
                    p = Plan([c1, c2, c3], obstacle, translation_vector, safe_swept_area, push_pose)
                    paths[p] = p.cost
                    if p.cost < p_opt[0].cost:
                        p_opt[0] = p

                seq = seq + 1

                translation_vector = (push_pose_unit_direction_vector[0] * self.ONE_PUSH_DISTANCE * float(seq),
                                      push_pose_unit_direction_vector[1] * self.ONE_PUSH_DISTANCE * float(seq))
                safe_swept_area = self._get_safe_swept_area(obstacle, translation_vector, map_cur.merged_occ_grid)
                obstacle_sim_pose = self._apply_translation_to_pose(push_pose, translation_vector)
                c3_est = Path.from_poses(obstacle_sim_pose, r_goal,
                                         False, self.MOVE_COST, push_pose_unit_direction_vector, self.ONE_PUSH_DISTANCE, map_cur.frame_id)
                Utils.debug_publish_path(c3_est, "/simulated/debug_c3")
                c_est = c1.cost + c3_est.cost + self.PUSH_COST * Utils.euclidean_distance(translation_vector, (0.0, 0.0))

        # Return path with lowest cost : the first in the ordered dictionnary
        # If there is none, return None
        try:
            return paths.peekitem(0)[0]
        except IndexError:
            return None

if __name__ == '__main__':
    rospy.init_node('nav_manager')

    nav_manager = NavManager(robot_radius=0.05,
                             robot_fov_radius=0.4,
                             static_map_topic="/map",
                             merged_occ_grid_topic="/robot_occ_grid",
                             push_poses_topic="/simulated/all_push_poses",
                             goal_topic="/move_base_simple/goal",
                             simulated_pose_topic="/simulated/pose",
                             simulated_fov_pointcloud_topic="/simulated/fov_pointcloud",
                             local_goal_pose_topic="/local_goal_pose")

    Utils.clean_up_viz()

    # obstacle1 = Obstacle.from_points_make_polygon(Utils.map_coords_to_real_coords(
    #     {(10, 3), (10, 6), (11, 3), (11, 6)},
    #     # {(10, 3)},
    #     # {(10, 3), (10, 11)},
    #     nav_manager.map_manager.multilayered_map.info.resolution),
    #     nav_manager.map_manager.multilayered_map.info,
    #     nav_manager.map_manager.multilayered_map.frame_id,
    #     nav_manager.map_manager.robot_metadata,
    #     1,
    #     True)
    # obstacle2 = Obstacle.from_points_make_polygon(Utils.map_coords_to_real_coords(
    #     {(12, 9), (12, 11), (13, 9), (13, 11)},
    #     # {(10, 3)},
    #     # {(10, 3), (10, 11)},
    #     nav_manager.map_manager.multilayered_map.info.resolution),
    #     nav_manager.map_manager.multilayered_map.info,
    #     nav_manager.map_manager.multilayered_map.frame_id,
    #     nav_manager.map_manager.robot_metadata,
    #     2,
    #     True)
    #
    # nav_manager.map_manager.manually_add_obstacle(obstacle1)
    # nav_manager.map_manager.manually_add_obstacle(obstacle2)
    # nav_manager.map_manager.publish_ros_merged_occ_grid()
    # nav_manager.map_manager.publish_all_push_poses()
    #
    # Test of plan_for_obstacle
    # p_opt = [Plan([Path(RosPath(), False, nav_manager.MOVE_COST)])]
    # map_cur = map_manager.get_map_copy()
    # r_cur = Utils.ros_pose_from_map_coord_yaw(4, 4, 0.0, map_cur.info.resolution, 1, map_manager.multilayered_map.frame_id)
    # r_goal = Utils.ros_pose_from_map_coord_yaw(20, 10, 0.0, map_cur.info.resolution, 1, map_manager.multilayered_map.frame_id)
    # blocked_obstacles = set()
    #
    # robot_pose_pub = rospy.Publisher("/simulated/robot_pose", PoseStamped, queue_size=1)
    # Utils.publish_once(robot_pose_pub, r_cur)
    # goal_pose_pub = rospy.Publisher("/simulated/goal_pose", PoseStamped, queue_size=1)
    # Utils.publish_once(goal_pose_pub, r_goal)

    # nav_manager.plan_for_obstacle(obstacle1, p_opt, map_cur, r_cur, r_goal, blocked_obstacles)

    # obstacles = list(map_cur.obstacles.values())
    # eu_cost_l = SortedDict()
    # min_cost_l = SortedDict()
    #
    # nav_manager.make_plan(r_cur, r_goal, map_cur, obstacles, blocked_obstacles, p_opt, eu_cost_l, min_cost_l)

    rate = rospy.Rate(20)

    while not nav_manager.is_ready:
        rate.sleep()

    nav_manager.manually_set_goal_robot_pose_from_map_coordinates_yaw(20, 4, 0.0)

    rospy.spin()
