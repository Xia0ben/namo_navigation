import unittest
import numpy

import sys
sys.path.append('../global_planner/')

from a_star import *

class TestAStar(unittest.TestCase):
            
    def test_simple(self):
        nmap = numpy.array([
            [ 0 ,  0 , 254, 254,  0 ,  0 ],
            [ 0 ,  0 , 100, 100,  0 ,  0 ],
            [ 0 ,  0 ,  0 ,  0 ,  0 ,  0 ],
            [ 0 ,  0 ,  50,  50,  0 ,  0 ],
            [ 0 ,  0 ,  50,  50,  0 ,  0 ]])
        
        path = AStar.astar(nmap, (0, 0), (4, 5))
    
        AStar.print_path(nmap, path)
    
        expected_path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5)]
        
        self.assertEqual(path, expected_path)
        
if __name__ == '__main__':
    unittest.main()