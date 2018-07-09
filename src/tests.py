import unittest
import numpy

from global_planner import GlobalPlanner
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import rospy

class TestGlobalPlanner(unittest.TestCase):
            
    def test_astar(self):
        global_planner = GlobalPlanner()
        
        # Info for testing a star alone
        nmap = numpy.array([
            [ 0 ,  0 , 254, 254,  0 ,  0 ],
            [ 0 ,  0 , 100, 100,  0 ,  0 ],
            [ 0 ,  0 ,  0 ,  0 ,  0 ,  0 ],
            [ 0 ,  0 ,  50,  50,  0 ,  0 ],
            [ 0 ,  0 ,  50,  50,  0 ,  0 ]])
        
        start_coord = (0, 0)
        end_coord = (4, 5)
        
        # Info for testing make_plan
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = "/map"
        occupancy_grid.info.resolution = 0.5
        occupancy_grid.info.width = 6
        occupancy_grid.info.height = 5
        
        occupancy_grid.data = nmap.flatten()
        
        start_pose = PoseStamped()
        start_pose.pose.position.x = float(start_coord[0])
        start_pose.pose.position.y = float(start_coord[1])
        
        end_pose = PoseStamped()
        end_pose.pose.position.x = (float(end_coord[0]) + 0.5) * occupancy_grid.info.resolution # 2.25
        end_pose.pose.position.y = (float(end_coord[1]) + 0.5) * occupancy_grid.info.resolution # 2.75
        
        # Results and check
        path = global_planner.astar(nmap, start_coord, end_coord)
        ros_path = global_planner.make_plan(occupancy_grid, start_pose, end_pose)
        
        # global_planner.print_path(nmap, path) # Uncomment to ease debugging
    
        expected_path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5)]
        expected_ros_path = [(0, 0), (0.75, 0.25), (1.25, 0.25), (1.75, 0.25), (2.25, 0.25), (2.25, 0.75), (2.25, 1.25), (2.25, 1.75), (2.25, 2.25), (2.25, 2.75)]
        
        self.assertEqual(path, expected_path)
        
        for pose, expected_position in zip(ros_path.poses, expected_ros_path):
            # print(pose) # Uncomment to ease debugging
            self.assertEqual(pose.pose.position.x, expected_position[0])
            self.assertEqual(pose.pose.position.y, expected_position[1])
            
    def test_astar_no_path(self):
        global_planner = GlobalPlanner()
        
        # Info for testing a star alone
        nmap = numpy.array([
            [ 0 ,  0 , 254, 254,  0 ,  0 ],
            [ 0 ,  0 , 254, 254,  0 ,  0 ],
            [ 0 ,  0 , 254, 254,  0 ,  0 ],
            [ 0 ,  0 , 254, 254,  0 ,  0 ],
            [ 0 ,  0 , 254, 254,  0 ,  0 ]])
        
        start_coord = (0, 0)
        end_coord = (4, 5)
        
        path = global_planner.astar(nmap, start_coord, end_coord)
        expected_path = []
        
        self.assertEqual(path, expected_path)
            
        
if __name__ == '__main__':
    unittest.main()