import numpy
import rospy

from bresenham import bresenham

from nav_msgs.msg import Path
from utils import Utils

class Plan:
    def __init__(self, is_multi_path, path, c1, c2, c3, is_manipulation=False, move_cost=1.0, push_cost=1.0):
        self._is_multi_path = is_multi_path

        if self._is_multi_path == True:
            self._c1 = c1
            self._c2 = c2
            self._c3 = c3
            self.cost = c1.cost + c2.cost + c3.cost
            self.min_cost = c2.cost + c3.cost
        else:
            self._move_cost = move_cost
            self._push_cost = push_cost
            self.path = path
            self._is_manipulation = is_manipulation
            self.cost = self.__sum_of_euclidean_distances() * (push_cost if is_manipulation else move_cost)

    @classmethod
    def from_path(cls, path, is_manipulation, move_cost=1.0, push_cost=1.0):
        path = []
        return cls(False, path, None, None, None, is_manipulation, move_cost, push_cost)

    @classmethod
    def from_plans(cls, c1, c2, c3):
        return cls(True, None, c1, c2, c3)

    @classmethod
    def from_poses(cls, init_pose, goal_pose, resolution, is_manipulation, move_cost=1.0, push_cost=1.0):
        init_map_coords = Utils.map_coord_from_ros_pose(init_pose, resolution)
        goal_map_coords = Utils.map_coord_from_ros_pose(goal_pose, resolution)
        
        map_path = list(
                    bresenham(init_map_coords[0], # x1
                              init_map_coords[1], # y1
                              goal_map_coords[0],  # x2
                              goal_map_coords[1])) # y2
        
        path = Utils.ros_path_from_map_path(map_path, init_pose, goal_pose, resolution, frame_id = "/map")
        
        return cls(False, path, None, None, None, is_manipulation, move_cost, push_cost)

    def __sum_of_euclidean_distances(self):
        if not self._is_multi_path:
            sum = 0.0
            poses = self.path.poses
            for i in range(len(poses) - 1):
                current_pose_np = numpy.array([poses[i].pose.position.x, poses[i].pose.position.y])
                next_pose_np = numpy.array([poses[i + 1].pose.position.x, poses[i + 1].pose.position.y])
                sum = sum + numpy.linalg.norm(next_pose_np - current_pose_np)
            return sum

    def _intersectsWithObstacle(self, obstacle):
        
        return True  # FIXME do actual test here eventually using ROS capabilities
        
        # Check if multipath -> if yes, apply following lines to c1, c2 and c3 (merge les arrays de position ?)
        # -> if no, apply following lines to path
        
        # Inflate obstacle by the robot footprint (actually, just get this inflation from the obstacle's properties)
        
        # Check if all points for the path are neighbours
        # -> if not, build the list of points using bresenham algorithm because
        # all points only have 2 neighbours at once, it seems
        # -> This gives the "path line"
        # May be interesting to inflate the path ?
        # Or rather inflate the obstacle more ?
        
        
        # FOREACH point of the "path line", check that it is not part of the inflated obstacle
        # -> As soon as an intersection point is found, return True
        # return False at the end of the loop
        
        # You might need to add an "from obstacle import Obstacle"

    def intersectsWithObstacles(self, obstacles):
        for obstacle in obstacles:
            if _intersectsWithObstacle(obstacle):
                return True
        return False