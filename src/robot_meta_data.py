import numpy as np
import math

from utils import Utils

class RobotMetaData:
    SQRT_2 = math.sqrt(2.0)

    def __init__(self, radius = 0.5, fov_radius = 1.0, map_resolution = 0.05):
        self.radius = float(radius)
        self.radius_div_by_sqrt_2 = self.radius / RobotMetaData.SQRT_2
        self.diameter = self.radius * 2.0
        self.fov_radius = fov_radius
        self.footprint = RobotMetaData.get_odd_circular_footprint(self.radius, map_resolution)
        self.footprint_2X = RobotMetaData.get_odd_circular_footprint(self.radius * 2.0, map_resolution)

    @staticmethod
    def get_odd_circular_footprint(radius, resolution):
        diameter = float(radius) * 2.0
        pixel_radius = round(radius / resolution)
        footprint_length = round(diameter / resolution) + 2
        footprint_length = footprint_length if footprint_length % 2 != 0 else footprint_length + 1

        footprint = np.zeros((footprint_length, footprint_length), dtype=int)

        circle_center = float(footprint_length) / 2.0

        for x in range(footprint_length):
            for y in range(footprint_length):
                for corner in Utils.get_corners_float_coords(x, y):
                    if math.sqrt((corner[0] - circle_center) ** 2.0 + (corner[1] - circle_center) ** 2.0) <= pixel_radius:
                        footprint[x][y] = Utils.ROS_COST_POSSIBLY_CIRCUMSCRIBED
        return footprint
