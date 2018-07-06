class Path:
    def __init__(self, multiPath, path, rosPath, c1, c2, c3, isManipulation=False, moveCost=1.0, pushCost=1.0):
        self.multiPath = multiPath

        if self.multiPath == True:
            self.c1 = c1
            self.c2 = c2
            self.c3 = c3
            self.cost = c1.cost + c2.cost + c3.cost
            self.minCost = c2.cost + c3.cost
        else:
            self.path = path
            self.rosPath = rosPath
            self.isManipulation = isManipulation
            self.cost = __sumOfEuclideanDistances(path) * (pushCost if isManipulation else moveCost)

    @classmethod
    def from_rosPath(cls, rosPath, isManipulation, moveCost=1.0, pushCost=1.0):
        # rosPathToSimplePath(rosPathP) # FIXME Do conversion from ROS path here
        path = []
        return cls(False, path, rosPath, None, None, None, isManipulation, moveCost, pushCost)

    @classmethod
    def from_paths(cls, c1, c2, c3):
        return cls(True, None, None, c1, c2, c3)

    @classmethod
    def from_poses(cls, initPose, goalPose, isManipulation, moveCost=1.0, pushCost=1.0):
        path = [initPose, goalPose]
        # simplePathToRosPath(path) # FIXME Do conversion to ROS path here
        rosPath = []
        return cls(False, path, rosPath, None, None, None, isManipulation, moveCost, pushCost)

    def __sumOfEuclideanDistances(path):
        sum = 0.0
        for i in range(len(path) - 1):
            sum = sum + numpy.linalg.norm(path[i + 1] - path[i].pos)
        return sum

    def intersectsWithObstacle(self, obstacle):
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
            if intersectsWithObstacle(obstacle):
                return True
        return False