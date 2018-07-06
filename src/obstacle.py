from simple_pose import SimplePose
from simple_position import SimplePosition

import copy

class Obstacle:
    idCounter = 1

    def __init__(self, position, points):
        # All following coordinates are in world frame
        self.id = Obstacle.idCounter
        Obstacle.idCounter = Obstacle.idCounter + 1

        self.isMovable = True

        self.initPose = SimplePose.from_position(position)
        self.simPose = copy.deepcopy(self.initPose)
        self.points = points
        # FIXME Compute grasp Points. ONLY FOR OBJECTS THAT ARE CIRCLES ?
        self.graspPoints = graspPoints
        # FIXME Compute Bounding Box
        self.bbTopLeft = SimplePosition(0, 0)
        self.bbBottomRight = SimplePosition(0, 0)

        self.obstacleMatrix = setObstacleMatrix(map)
        self.robotInflatedObstacle = setInflatedObstacle(map)

    def __hash__(self):
        # Hash depends only on the unique and immutable id of the obstacle
        return self.id