from robot_meta_data import RobotMetaData
from shapely.geometry import Point
from geometry_msgs.msg import PoseStamped
from utils import Utils

class RobotState:
    def __init__(self, robot_radius, robot_fov_radius, map_resolution, init_robot_pose = None):
        self.metadata = RobotMetaData(robot_radius, robot_fov_radius, map_resolution)
        self.pose = init_robot_pose
        self.map_position = None
        self.yaw = None
        self.polygonal_footprint = None
        self.polygonal_fov = None
        if self.pose is not None:
            self._update_representations(map_resolution)

    def set_pose(self, pose, map_resolution):
        self.pose = pose
        self._update_representations(map_resolution)

    def _update_representations(self, map_resolution):
        self.map_position = Utils.map_coord_from_ros_pose(self.pose, map_resolution)
        self.yaw = Utils.yaw_from_geom_quat(self.pose.pose.orientation)
        self.polygonal_footprint = Point((self.pose.pose.position.x, self.pose.pose.position.y)).buffer(self.metadata.radius)
        self.polygonal_fov = Point((self.pose.pose.position.x, self.pose.pose.position.y)).buffer(self.metadata.fov_radius)