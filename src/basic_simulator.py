import rospy
from multilayered_map import MultilayeredMap, IntersectionWithObstaclesException, ObstacleOutOfBoundsException, IntersectionWithRobotException
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from utils import Utils
from robot_state import RobotState
from obstacle import Movability
from shapely.geometry import Point
import copy
from namo_navigation.srv import LocalGoalPose, LocalGoalPoseResponse
from nav_manager import NavManager

class RobotPlacementException(Exception):
    pass

class BasicSimulator:
    obstacle_id_counter = 1

    def __init__(self, robot_radius, robot_fov_radius, static_map_topic, local_goal_pose_topic,
                 initial_pose_topic, point_clicked_topic, simulated_occ_grid_topic, simulated_pose_topic,
                 simulated_fov_pointcloud_topic, simulated_robot_polygonal_footprint_topic,
                 simulated_robot_polygonal_fov_topic):
        # Parameters
        self.static_map_topic = static_map_topic
        self.local_goal_pose_topic = local_goal_pose_topic
        self.initial_pose_topic = initial_pose_topic
        self.point_clicked_topic = point_clicked_topic
        self.simulated_occ_grid_topic = simulated_occ_grid_topic
        self.simulated_pose_topic = simulated_pose_topic
        self.simulated_fov_pointcloud_topic = simulated_fov_pointcloud_topic
        self.simulated_robot_polygonal_footprint_topic = simulated_robot_polygonal_footprint_topic
        self.simulated_robot_polygonal_fov_topic = simulated_robot_polygonal_fov_topic

        # World state
        self.simulated_map = None
        self.static_map = None

        # Subscribers
        rospy.Subscriber(self.static_map_topic, OccupancyGrid, self._static_map_callback)
        rospy.Subscriber(self.local_goal_pose_topic, PoseStamped, self._local_goal_pose_callback)
        rospy.Subscriber(self.initial_pose_topic, PoseWithCovarianceStamped, self._initial_pose_callback)
        rospy.Subscriber(self.point_clicked_topic, PointStamped, self._point_for_obstacle)

        # Services
        rospy.Service(self.local_goal_pose_topic, LocalGoalPose, self._local_goal_pose_callback)

        # Publishers
        self.simulated_pose_pub = rospy.Publisher(self.simulated_pose_topic, PoseStamped, queue_size=1)
        self.simulated_robot_polygonal_footprint_pub = rospy.Publisher(self.simulated_robot_polygonal_footprint_topic, PolygonStamped, queue_size=1)
        self.simulated_robot_polygonal_fov_pub = rospy.Publisher(self.simulated_robot_polygonal_fov_topic, PolygonStamped, queue_size=1)
        self.simulated_fov_pointcloud_pub = rospy.Publisher(self.simulated_fov_pointcloud_topic, PointCloud, queue_size=1)
        self.simulated_occ_grid_pub = rospy.Publisher(self.simulated_occ_grid_topic, OccupancyGrid, queue_size=1)

        # Initialize map and robot
        while self.static_map is None:
            rospy.sleep(0.2)
        self.simulated_robot_state = RobotState(robot_radius, robot_fov_radius, self.static_map.info.resolution)
        self.simulated_map = MultilayeredMap(self.static_map, self.simulated_robot_state.metadata)
        self.simulated_fov_pointcloud = PointCloud()

    def _static_map_callback(self, static_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = static_map

    def _local_goal_pose_callback(self, request):
        local_pose_goal = request.local_pose_goal
        is_manipulation = request.is_manipulation
        response = LocalGoalPoseResponse()

        goal_map_coords = Utils.map_coord_from_ros_pose(local_pose_goal, self.simulated_map.info.resolution)
        expected_polygon_footprint = Point((local_pose_goal.pose.position.x, local_pose_goal.pose.position.y)).buffer(self.simulated_robot_state.metadata.radius)

        # Check if expected robot polygon is going tp intersect any obstacle's polygons
        obstacles_colliding_with_robot = set()
        for obstacle in self.simulated_map.obstacles.values():
            if expected_polygon_footprint.intersects(obstacle.polygon):

                # If the robot entered in collision with at least one unmovable obstacle, the pose is not updated
                if obstacle.movability == Movability.unmovable:
                    response.real_pose = self.simulated_robot_state.pose
                    return response
                obstacles_colliding_with_robot.add(obstacle)

        # Check if robot is going to enter in contact with static obstacles
        is_colliding_with_static_obstacles = (
                Utils._is_in_matrix(goal_map_coords[0], goal_map_coords[1], self.simulated_map.info.width, self.simulated_map.info.height) and
                self.simulated_map.inflated_static_occ_grid[goal_map_coords[0]][goal_map_coords[1]] >= Utils.ROS_COST_POSSIBLY_CIRCUMSCRIBED)

        # If no obstacles collided with the robot, simply update the pose
        if not (bool(obstacles_colliding_with_robot) or is_colliding_with_static_obstacles):
            self.simulated_robot_state.set_pose(local_pose_goal, self.simulated_map.info.resolution)
            self.simulated_fov_pointcloud = self.simulated_map.get_point_cloud_in_fov(self.simulated_robot_state.pose)
        else:
            # Else we must check if these obstacles collide with other when translation is applied
            translation = (local_pose_goal.pose.position.x - self.simulated_robot_state.pose.pose.position.x,
                           local_pose_goal.pose.position.y - self.simulated_robot_state.pose.pose.position.y)
            for colliding_obstacle in obstacles_colliding_with_robot:
                colliding_obstacle_with_push = copy.deepcopy(colliding_obstacle)
                colliding_obstacle_with_push.move(translation)

                if colliding_obstacle_with_push.movability == Movability.movable:
                    # Check collision between obstacles
                    for other_obstacle_id, other_obstacle in self.simulated_map.obstacles.items():
                        # If the movable obstacle we are moving enters in collision with another, the pose is not updated
                        if other_obstacle_id != colliding_obstacle_with_push.obstacle_id and colliding_obstacle_with_push.polygon.intersects(other_obstacle.polygon):
                            response.real_pose = self.simulated_robot_state.pose
                            return response
                    # Check collision between pushed obstacle and static obstacles
                    for map_point in colliding_obstacle_with_push.discretized_polygon:
                        if self.simulated_map.static_occ_grid[map_point[0]][map_point[1]] == Utils.ROS_COST_LETHAL:
                            response.real_pose = self.simulated_robot_state.pose
                            return response

            # If no obstacle bothered us while pushing a movable obstacle, apply translation to it and update robot pose
            self.simulated_robot_state.set_pose(local_pose_goal, self.simulated_map.info.resolution)
            if is_manipulation:
                for colliding_obstacle in obstacles_colliding_with_robot:
                    self.simulated_map.manually_move_obstacle(colliding_obstacle.obstacle_id, translation)
            self.simulated_fov_pointcloud = self.simulated_map.get_point_cloud_in_fov(self.simulated_robot_state.pose)

        response.real_pose = self.simulated_robot_state.pose
        return response

    def _initial_pose_callback(self, pose_with_covariance):
        pose = PoseStamped()
        pose.header = pose_with_covariance.header
        pose.pose = pose_with_covariance.pose.pose
        map_coords = Utils.map_coord_from_ros_pose(pose, self.simulated_map.info.resolution)
        try:
            self.set_initial_robot_pose(pose, map_coords[0], map_coords[1])
        except RobotPlacementException:
            rospy.logerr(
                "The given robot pose is not valid: : it is either outside of the map or intersects obstacles.")

    def _point_for_obstacle(self, point):
        pass
        # TODO BONUS allow simple obstacle creation through rviz
        # Check in polygonal representation of all obstacles if point is inside by other obstacle

    def set_initial_robot_pose(self, pose, map_coord_x, map_coord_y):
        if (Utils._is_in_matrix(map_coord_x, map_coord_y, self.simulated_map.info.width, self.simulated_map.info.height) and
                not self.simulated_map.merged_occ_grid[map_coord_x][map_coord_y] >= Utils.ROS_COST_POSSIBLY_CIRCUMSCRIBED):
            self.simulated_robot_state.set_pose(pose, self.simulated_map.info.resolution)
        else:
            raise RobotPlacementException
        self.simulated_fov_pointcloud = self.simulated_map.get_point_cloud_in_fov(self.simulated_robot_state.pose)

    def manually_set_initial_robot_pose_from_map_coordinates_yaw(self, coordX, coordY, yaw):
        pose = Utils.ros_pose_from_map_coord_yaw(coordX, coordY, yaw, self.simulated_map.info.resolution, 1, self.simulated_map.frame_id)
        try:
            self.set_initial_robot_pose(pose, coordX, coordY)
        except RobotPlacementException:
            rospy.logerr("The given robot pose is not valid: : it is either outside of the map or intersects obstacles.")

    def manually_add_obstacle_from_map_coordinates(self, map_points_set, movability):
        obstacle_id = BasicSimulator.obstacle_id_counter
        points_set = Utils.map_coords_to_real_coords(map_points_set, self.simulated_map.info.resolution)
        try:
            self.simulated_map.manually_add_obstacle(points_set, obstacle_id, movability, self.simulated_robot_state.polygonal_footprint)
            BasicSimulator.obstacle_id_counter = BasicSimulator.obstacle_id_counter + 1
        except IntersectionWithObstaclesException:
            rospy.logerr("Obstacle could not be created: it overlaps other obstacles.")
        except ObstacleOutOfBoundsException:
            rospy.logerr("Obstacle could not be created: it is outside of the map.")
        except IntersectionWithRobotException:
            rospy.logerr("Obstacle could not be created: it intersects with the robot.")
        if self.simulated_robot_state.pose is not None:
            self.simulated_fov_pointcloud = self.simulated_map.get_point_cloud_in_fov(self.simulated_robot_state.pose)

    def manually_remove_obstacle(self, obstacle_id):
        pass # FIXME implement this and obstacle update too

    def update(self, time_delta):
        ros_simulated_occ_grid = Utils.convert_matrix_to_ros_occ_grid(self.simulated_map.merged_occ_grid, self.static_map.header, self.static_map.info)
        Utils.publish_once(self.simulated_occ_grid_pub, ros_simulated_occ_grid)
        if self.simulated_robot_state.pose is not None:
            Utils.publish_once(self.simulated_pose_pub, self.simulated_robot_state.pose)
            Utils.publish_once(self.simulated_robot_polygonal_footprint_pub, Utils.convert_shapely_as_ros_polygon(self.simulated_robot_state.polygonal_footprint))
            Utils.publish_once(self.simulated_robot_polygonal_fov_pub, Utils.convert_shapely_as_ros_polygon(self.simulated_robot_state.polygonal_fov))
            Utils.publish_once(self.simulated_fov_pointcloud_pub, self.simulated_fov_pointcloud)

    # def set_simulation_robot_twist_callback(self, twist):
    #     self.simulation_robot_twist = twist

    # def update(self, time_delta):
    #     # Get position and velocity
    #     theta = Utils.yaw_from_geom_quat(self.init_robot_yaw)
    #     x = self.simulation_robot_pose.pose.position.x
    #     y = self.simulation_robot_pose.pose.position.y
    #     v = self.simulation_robot_twist.linear.x
    #     w = self.simulation_robot_twist.angular.z
    #
    #     # Compute new values
    #     if w != 0:
    #         new_x = x + -(v/w)*math.sin(theta) + (v/w)*math.sin(theta+w*time_delta)
    #         new_y = y -(v/w)*math.cos(theta) - (v/w)*math.cos(theta+w*time_delta)
    #     else:
    #         new_x = x + v * time_delta * math.cos(theta)
    #         new_y = y + v * time_delta * math.sin(theta)
    #     new_theta = theta + w * time_delta
    #
    #     new_robot_polygonal_footprint = Point(new_x, new_y).buffer(self.robot_diameter / 2.0)
    #
    #     # FIXME If new position causes the robot to enter in contact with an obstacle,
    #     # check whether the obstacle is movable or not. If it is, move it in the
    #     # right direction so that the points are not in collision anymore
    #
    #
    #     # FIXME Update robot pose according to computed values AND collision detection
    #     self.simulation_robot_pose.header.seq = self.simulation_robot_pose.header.seq + 1
    #     self.simulation_robot_pose.header.stamp = rospy.Time.now()
    #     self.simulation_robot_pose.pose.position.x = new_x
    #     self.simulation_robot_pose.pose.position.y = new_y
    #     self.simulation_robot_pose.pose.orientation = Utils.geom_quat_from_yaw(new_theta)
    #
    #     # Update fov pointcloud according to what the robot should see in the
    #     # new pose
    #     self.simulation_fov_pointcloud = self.sim_map.get_point_cloud_in_fov(self.simulation_robot_pose)
    #
    #     # CONCLUSION : Publish pose and point cloud
    #     Utils.publish_once(self.simulated_pose_pub, self.simulation_robot_pose)
    #     Utils.publish_once(self.simulated_fov_pointcloud_pub, self.simulation_fov_pointcloud)

if __name__ == "__main__":
    rospy.init_node('basic_simulator', log_level=rospy.INFO)

    basic_simulator = BasicSimulator(robot_radius=0.05,
                                     robot_fov_radius=0.4,
                                     static_map_topic="/map",
                                     local_goal_pose_topic="/local_goal_pose",
                                     initial_pose_topic="/initialpose",
                                     point_clicked_topic="/clicked_point",
                                     simulated_occ_grid_topic="/simulated/occ_grid",
                                     simulated_pose_topic="/simulated/pose",
                                     simulated_fov_pointcloud_topic="/simulated/fov_pointcloud",
                                     simulated_robot_polygonal_footprint_topic="/simulated/robot_polygonal_footprint",
                                     simulated_robot_polygonal_fov_topic="/simulated/robot_polygonal_fov")

    basic_simulator.manually_set_initial_robot_pose_from_map_coordinates_yaw(4, 4, 0.0)

    basic_simulator.manually_add_obstacle_from_map_coordinates({(10, 3), (10, 6), (11, 3), (11, 6)}, Movability.unmovable)

    basic_simulator.manually_add_obstacle_from_map_coordinates({(12, 9), (12, 11), (13, 9), (13, 11)}, Movability.movable)

    frequency = 20.0 # [Hz]
    time_delta = 1.0 / frequency # [s]
    rate = rospy.Rate(frequency)

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

    # simulated_local_goal_pose_pub = rospy.Publisher("/local_goal_pose", PoseStamped, queue_size=1)
    # counter = 0
    while not rospy.is_shutdown():
        # if counter % 5 == 0:
        #     pose = copy.deepcopy(basic_simulator.simulated_robot_state.pose)
        #     pose.pose.position.x = pose.pose.position.x + 0.05
        #     Utils.publish_once(simulated_local_goal_pose_pub, pose)
        # counter = counter + 1
        basic_simulator.update(time_delta)
        rate.sleep()
