import rospy
from multilayered_map import MultilayeredMap
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from utils import Utils
import math

from shapely.geometry import Point

class BasicSimulator:
    
    def __init__(self):
        # Parameters
        self.static_map_topic = "/map" # rospy.get_param('~map_topic')
        self.simulated_pose_topic = "/simulated/pose" # rospy.get_param('~simulated_pose_topic')
        self.simulated_cmd_vel_topic = "/simulated/cmd_vel" # rospy.get_param('~simulated_cmd_vel_topic')
        self.simulated_fov_pointcloud_topic = "/simulated_fov_pointcloud" # rospy.get_param('~simulated_fov_pointcloud_topic')
        self.map_frame = "/map" # rospy.get_param('~map_frame')
        self.robot_frame = "/base_link" # rospy.get_param('~robot_frame')
        self.robot_diameter = "0.5" # [m] rospy.get_param('~robot_diameter')
        self.robot_fov_radius = "3.0" # [m] rospy.get_param('~robot_fov_radius')
        self.init_robot_2d_position = [0.0, 0.0] # rospy.get_param('~init_robot_position')
        self.init_robot_yaw = 0.0 # rospy.get_param('~init_robot_yaw')
        self.init_robot_twist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # rospy.get_param('~init_robot_twist')
        self.robot_polygonal_footprint = Point(self.init_robot_2d_position).buffer(self.robot_diameter / 2.0)
        self.arbitrary_obstacles = {
            1: {(1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 1.0)}
        }

        # Subscribe to simulated static map and cmd_vel topics
        rospy.Subscriber(self.static_map_topic, OccupancyGrid, self.static_map_callback)
        rospy.Subscriber(self.simulated_cmd_vel_topic, Twist, self.set_simulation_robot_twist_callback)
        
        # Publish simulated pose topic
        self.simulated_pose_pub = rospy.Publisher(self.simulated_pose_topic, PoseStamped, queue_size=1)
        
        # Publish simulated fov pointcloud topic
        self.simulated_fov_pointcloud_pub = rospy.Publisher(self.simulated_fov_pointcloud_topic, PointCloud, queue_size=1)
        
        # Robot state
        self.simulation_robot_pose = PoseStamped()
        self.simulation_robot_pose.header.seq = 1
        self.simulation_robot_pose.header.stamp = rospy.Time(0)
        self.simulation_robot_pose.header.frame_id = self.map_frame
        self.simulation_robot_pose.pose.position.x = self.init_robot_2d_position[0]
        self.simulation_robot_pose.pose.position.y = self.init_robot_2d_position[1]
        self.simulation_robot_pose.pose.position.z = 0.0
        self.simulation_robot_pose.pose.orientation = Utils.geom_quat_from_yaw(self.init_robot_yaw)
        
        self.simulation_robot_twist = Twist()
        self.simulation_robot_twist.linear.x = self.init_robot_twist[0]
        self.simulation_robot_twist.linear.y = self.init_robot_twist[1]
        self.simulation_robot_twist.linear.z = self.init_robot_twist[2]
        self.simulation_robot_twist.angular.x = self.init_robot_twist[3]
        self.simulation_robot_twist.angular.y = self.init_robot_twist[4]
        self.simulation_robot_twist.angular.z = self.init_robot_twist[5]
        
        # World state
        self.sim_map = None
        self.static_map = None
        # Initialize map
        while self.static_map is None:
            rospy.sleep(0.2)
        self.sim_map = MultilayeredMap(self.static_map, self.robot_diameter, self.robot_fov_radius, self.arbitrary_obstacles)
        
        # Robot fov point cloud
        self.simulation_fov_pointcloud = self.sim_map.get_point_cloud_in_fov(self.simulation_robot_pose)

    def _static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map
            
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
    #     self.simulation_robot_pose.header.stamp = rospy.Time(0)
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
    basic_simulator = BasicSimulator()
    
    # frequency = 60.0 # [Hz]
    # time_delta = 1.0 / frequency # [s]
    # rate = rospy.Rate(frequency)
    #
    # while not rospy.is_shutdown():
    #     basic_simulator.update(time_delta)
    #     rate.sleep()

    rospy.spin()