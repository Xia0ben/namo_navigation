class Plan:
    def __init__(self, components, obstacle = None, translation_vector = None, safe_swept_area = None):

        self.components = components
        self.obstacle = obstacle
        self.translation_vector = translation_vector
        self.safe_swept_area = safe_swept_area

        # If this is a plan with only one path component (avoid all obstacles)
        self.cost = components[0].cost
        self.min_cost = None

        # If this is a plan with more three components (manipulate an obstacle)
        if len(components) == 3:
            self.cost = components[0].cost + components[1].cost + components[2].cost
            self.min_cost = components[1].cost + components[2].cost

    def set_obstacle(self, obstacle):
        self.obstacle = obstacle

    def intersects_with_obstacles(self, occupancy_grid):


        """
        Use the following four lines if you would rather test intersection
        with a list of obstacles.
        """
        # for obstacle in obstacles:
        #     if _intersectsWithObstacle(obstacle):
        #         return True
        # return False

    def _intersectsWithObstacle(self, obstacle):

        # Test to use if we want to check whether the path enters in collision with a specific obstacle or not.

        return True  # TODO do actual test here eventually using ROS capabilities

        # Check if multipath -> if yes, apply following lines to c1, c2 and c3 (merge les arrays de position ?)
        # -> if no, apply following lines to path

        # Inflate obstacle by the robot footprint (actually, just get this inflation from the obstacle's properties)

        # Check if all points for the path are neighbours
        # -> if not, build the list of points using bresenham algorithm because
        # all points only have 2 neighbours at once, it seems
        # -> This gives the "pa obstacle more ?


        # FOREACH point of the "path line", check that it is not part of the inflated obstacle
        # -> As soon as an intersection point is found, return True
        # return False at the end of the loop

        # You might need to add an "from obstacle import Obstacle"th line"
        # May be interesting to inflate the path ?
        # Or rather inflate the obstacle more ?


        # FOREACH point of the "path line", check that it is not part of the inflated obstacle
        # -> As soon as an intersection point is found, return True
        # return False at the end of the loop

        # You might need to add an "from obstacle import Obstacle"
