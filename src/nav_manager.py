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
from multilayered_map import MultilayeredMap
from plan import Plan

class NavManager:
    
    def __init__():
        # Get parameters
        self.static_map_topic = "/map" # rospy.get_param('~map_topic')
        self.robot_diameter = "0.5" # rospy.get_param('~robot_diameter')
        self.map_frame = "/map" # rospy.get_param('~map_frame')
        self.robot_frame = "base_footprint" # rospy.get_param('~robot_frame')
        self.move_cost = 1.0 # rospy.get_param('~move_cost')
        self.push_cost = 1.0 # rospy.get_param('~push_cost')
        
        # Subscribe to necessary topics
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(self.static_map_topic, OccupancyGrid, self.static_map_callback)
        
        # Declare common parameters
        self.static_map = None
        self.navigation_map = None
        self.global_planner = GlobalPlanner
        
        # Initialize map
        while self.static_map is None:
            rospy.sleep(0.2)
        self.navigation_map = MultilayeredMap(self.static_map, self.robot_diameter)

        
    def _static_map_callback(self, new_map):
        # For the moment, we don't want to manage new static maps for the
        # node's life duration
        if self.static_map is None:
            self.static_map = new_map
            
    def _get_current_pose():
        time  = rospy.Time(0) # Makes sure same tf is used for calls
        robot_pose = PoseStamped()
        
        try:
            (position, quaternion) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)
            
            robot_pose.header.seq = 1
            robot_pose.header.stamp = rospy.Time(0)
            robot_pose.header.frame_id = self.map_frame
            robot_pose.pose.position.x = position.x
            robot_pose.pose.position.y = position.y
            robot_pose.pose.position.z = position.z
            robot_pose.pose.orientation.x = quaternion.x
            robot_pose.pose.orientation.y = quaternion.y
            robot_pose.pose.orientation.z = quaternion.z
            robot_pose.pose.orientation.w = quaternion.w
            
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("Couldn't get robot pose in map : transform unavailable. \
            Maybe amcl is not working and there is no transform between \
            /odom and /map ?")
        
        return robot_pose


    def makeAndExecutePlan(goalRobotPoseParam):
        initRobotPose = self._get_current_pose()
    
        self._optimized(initRobotPose, goalRobotPose)
    
    def _optimized(initRobotPose, goalRobotPose):
    
        # Current state
        currentRobotPose = initRobotPose
        newObstacles = set()
        euclideanCostL = SortedDict()
        minCostL = SortedDict()
        optimalPath = GlobalPlanner.make_plan(self.navigation_map.get_merged_occupancy_grid(), initRobotPose, goalRobotPose)
    
        # While 1
        while (currentRobotPose != goalRobotPose):  # FIXME to be reformulated
            map.update()
            if {map.freeSpaceCreated()}
                minCostL.clear()
    
            newObstacles = newObstacles.union(map.newMovableObstacles())
    
            for currentObstacle in map.obstacles:
                euclideanCostL[currentObstacle] = numpy.linalg.norm(
                    goalRobotPose.pos - currentObstacle.initPose.pos) # FIXME ADD CONVERSION FROM TUPLE TO ARRAY
    
            if (optimalPath.intersectsWithObstacles(newObstacles)):
                optimalPath = GlobalPlanner.make_plan(
                    self.navigation_map.get_merged_occupancy_grid(), currentRobotPose, goalRobotPose)
    
                indexEL, indexML = 0, 0
    
                # minCostL.peekitem(indexML)[0] -> returns the obstacle
                # minCostL.peekitem(indexML)[1] -> returns the estimated cost
                # While 2
                # FIXME check for IndexError because if list is empty, everything will explode
                while (min(minCostL.peekitem(indexML)[1], euclideanCostL.peekitem(indexEL)[1]) < optimalPath.cost):
                    if (minCostL.peekitem(indexML).[1] < euclideanCostL.peekitem(indexEL)[1]):
                        currentPath = optEvaluateAction(minCostL.peekitem(indexML).[0], optimalPath, map, currentRobotPose, goalRobotPose)
    
                        if (currentPath is not None):
                            minCostL[minCostL.peekitem(indexML).[0]] = currentPath.minCost
                            indexML = indexML + 1
    
                            if (currentPath.cost < optimalPath.cost):
                                optimalPath = currentPath
    
                    else
                        try:
                            # If the minCostL doesn't contain the obstacle, do except
                            minCostL.index(euclideanCostL.peekitem(indexEL)[0])
                        except ValueError as exception:
                            currentPath = optEvaluateAction(euclideanCostL.peekitem(
                                indexEL)[0], optimalPath, map, currentRobotPose, goalRobotPose)
    
                            if (currentPath is not None):
                                minCostL[euclideanCostL.peekitem(
                                    indexEL)[0]] = currentPath.minCost
                                indexML = indexML + 1
    
                                if (currentPath.cost < optimalPath.cost):
                                    optimalPath = currentPath
    
                    indexEL = indexEL + 1
                    # Endwhile 2
    
                newObstacles.clear()
    
            # R \gets$ Next step in $optimalPath$ FIXME Have the robot advance one step in the plan
            # Endwhile 1


    def optEvaluateAction(obstacle, optimalPath, map, currentRobotPose, goalRobotPose):
        pathsSet = SortedDict()  # Use sortedDict rather than set or array for ease of use
        blockingAreas = None
    
        # For easier implementation, we iterate over grasp points rather than
        # an abstract concept of "direction"
        for graspPoint in obstacle.graspPoints
            one_push_in_d = obstacle.initPose.pos - graspoint  # FIXME ADD CONVERSION FROM TUPLE TO ARRAY
            # FIXME put the right value here
            firstRobotPose = copy.deepcopy(currentRobotPose)
    
            # Computation of c1 is done here (a contrario to the original algorithm)
            # because it is in fact dependent of the grasp point (if the grasp point
            # cannot be reached, we imediately know it)
            c1 = GlobalPlanner.make_plan(self.navigation_map.get_merged_occupancy_grid(),
                currentRobotPose, firstRobotPose)
            # If no path to the grasping point is found, then study next point
            if (c1 is not None):
                obstacle.simPose = obstacle.initPose
                seq = 1
                # FIXME so that the start position is not the center of the object but the grasping point
                c3_est = Plan.from_poses(obstacle.simPose, goalRobotPose,
                            self.navigation_map.resolution, isManipulation = False,
                            move_cost = self.move_cost, push_cost = self.push_cost)
                cEst = c1.cost + c3_est.cost + seq * self.push_cost
    
                while (obstacle.pushUsingGraspPointPossible(graspPoint, map) and cEst <= optimalPath.cost):
                    if (checkNewOpening(map, obstacle, [one_push_in_d], blockingAreas)):
                        obstacle.simPose.pos = obstacle.simPose.pos + one_push_in_d
                        # FIXME both points
                        c2 = Plan.from_poses(obstacle.initPose, obstacle.simPose,
                            self.navigation_map.resolution, isManipulation = False,
                            move_cost = self.move_cost, push_cost = self.push_cost)
                        c3 = Plan.from_path(GlobalPlanner.make_plan(self.navigation_map.get_merged_occupancy_grid(), obstacle.simPose, goalRobotPose))  # FIXME origin point
                        # Costs are computed within the class
                        currentPath = Plan.from_plans(c1, c2, c3)
                seq = seq + 1
    
                # FIXME so that the start position is not the center of the object but the grasping point
                c3_est = Plan.from_poses(obstacle.simPose, goalRobotPose,
                            self.navigation_map.resolution, isManipulation = True,
                            move_cost = self.move_cost, push_cost = self.push_cost)
    
                cEst = c1.cost + c3_est.cost + seq * self.push_cost
    
        # Return path with lowest cost : the first in the ordered dictionnary
        # If there is none, return None
        try:
            return peekitem(0)
        except IndexError as noPathFound:
            return None
    
    # It may be interesting to move the following methods into a proper class
    
    
    def checkNewOpening(map, obstacle, action, blockingAreas):
        M = getObstacleMatrix(map, obstacle)
        xOffset = obstacle.simPose.pos[0]
        yOffset = obstacle.simPose.pos[1]
    
        if (blockingAreas is None):
            blockingAreas = getBlockingAreas(
                xOffset, yOffset, M, map.mergedBinaryOccupationGrid)
    
        newPos = getNewPos(action, obstacle)
        xOffset, yOffset = newPos[0], newPos[1]
        blockingAreasAfterAction = getBlockingAreas(
            xOffset, yOffset, M, map.mergedBinaryOccupationGrid)
        shiftedBlockingAreas = [
            [0 for k in range(len(M))] for l in range(len(M[k]))]
    
        for i in range(len(shiftedBlockingAreas)):
            for j in range(len(shiftedBlockingAreas[i])):
                x = (xOffset - obstacle.simPose.pos[0]) + i
                y = (yOffset - obstacle.simPose.pos[1]) + j
    
                if (0 < x < len(shiftedBlockingAreas) and 0 < y < len(shiftedBlockingAreas[x])):
                    shiftedBlockingAreas[x][y] = blockingAreasAfterAction[i][j]
    
        Z = compare(blockingAreas, shiftedBlockingAreas)
        if isZeroMatrix(Z)
            return False
        return True
    
    
    def getBlockingAreas(xOffset, yOffset, M, grid):
        index = 1
        blockingAreas = [[0 for k in range(len(M))] for l in range(len(M[k]))]
    
        for i in range(len(blockingAreas)):
            for j in range(len(blockingAreas[i])):
                if (M[x][y] != 0 and grid[x + xOffset][y + yOffset] != 0):
                    assignNr(blockingAreas, x, y, index)
    
        return blockingAreas
    
    
    def assignNr(blockingAreas, x, y, index):
        # Note : range(-1, 2) = [-1, 0, 1]
        for i in range(-1, 2):
            for j in range(-1, 2):
                if (blockingAreas[x + i][y + j] != 0):
                    blockingAreas[x][y] = blockingAreas[x + i][x + j]
                    return
        blockingAreas[x][y] = index
        index = index + 1
        return
    
    
    def compare(blockingAreasBefore, blockingAreasAfter):
        comparisonMatrix = copy.deepcopy(blockingAreasAfter)
        delNum = set()
    
        for x in range(len(comparisonMatrix)):
            for y in range(len(comparisonMatrix[x])):
    
                if (blockingAreasBefore[x][y] in delNum):
                    comparisonMatrix[x][y] = 0
                if (blockingAreasBefore[x][y] != 0 and
                        blockingAreasAfter[x][y] != 0):
                    delNum = delNum.union(comparisonMatrix[x][y])
                    comparisonMatrix[x][y] = 0
    
        return comparisonMatrix
    
    # Helper functions for the Efficient Local Opening Detection Algorithm (ELODA)
    
    # action is an array of 2D vectors (2 items-arrays)
    def getNewPos(action, obstacle):
        action = [[20.0, 30.0]]
        newPos = [obstacle.simPose.pos[0], obstacle.simPose.pos[1]]
    
        for component in action:
            newPos = [newPos[0] + component[0], newPos[1] + component[1]]
    
        return newPos
    
    
    def isZeroMatrix(Z):
        for i in range(len(Z)):
            for j in range(len(Z[i])):
                if Z[i][j] != 0
                    return False
        return True