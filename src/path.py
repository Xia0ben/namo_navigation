import numpy

from bresenham import bresenham

from utils import Utils

class Path:
    def __init__(self, ros_path, is_manipulation, weigth):
        self.path = ros_path
        self.is_manipulation = is_manipulation
        self.weigth = weigth

        self.cost = self.__sum_of_euclidean_distances() * weigth

    @classmethod
    def from_path(cls, ros_path, is_manipulation, weigth):
        return cls(ros_path, is_manipulation, weigth)

    @classmethod
    def from_poses(cls, init_pose, goal_pose, is_manipulation, weigth, map_resolution, map_frame):
        ros_path = Utils.ros_path_from_poses(init_pose, goal_pose, map_frame)

        return cls(ros_path, is_manipulation, weigth)

    def __sum_of_euclidean_distances(self):
        sum = 0.0
        poses = self.path.poses
        if len(poses) == 0:
            return float("inf")
        for i in range(len(poses) - 1):
            current_pose_np = numpy.array([poses[i].pose.position.x, poses[i].pose.position.y])
            next_pose_np = numpy.array([poses[i + 1].pose.position.x, poses[i + 1].pose.position.y])
            sum = sum + numpy.linalg.norm(next_pose_np - current_pose_np)
        return sum
