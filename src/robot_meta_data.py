import numpy as np
import math

from utils import Utils

class RobotMetaData:
    def __init__(self, radius = 0.5, fov_radius = 1.0, map_resolution = 0.05):
        self.radius = float(radius)
        self.radius_div_by_sqrt_2 = self.radius / Utils.SQRT_2
        self.diameter = self.radius * 2.0
        self.fov_radius = fov_radius
        self.footprint = RobotMetaData.get_odd_circular_footprint(self.radius, map_resolution)
        self.footprint_2X = RobotMetaData.get_odd_circular_footprint(self.radius * 2.0, map_resolution)

    @staticmethod
    def get_odd_circular_footprint(radius, resolution):
        diameter = float(radius) * 2.0
        pixel_radius = round(radius / resolution)
        footprint_length = int(round(diameter / resolution))
        footprint_length = footprint_length if footprint_length % 2 != 0 else footprint_length + 1

        footprint = np.full((footprint_length, footprint_length), Utils.ROS_COST_FREE_SPACE, dtype=int)

        circle_center = float(footprint_length) / 2.0

        for x in range(footprint_length):
            for y in range(footprint_length):
                for corner in Utils.get_corners_float_coords(x, y):
                    if math.sqrt((corner[0] - circle_center) ** 2.0 + (corner[1] - circle_center) ** 2.0) <= pixel_radius:
                        footprint[x][y] = Utils.ROS_COST_POSSIBLY_CIRCUMSCRIBED
        return footprint

if __name__ == '__main__':
    # Basic test
    rmd = RobotMetaData(0.025, 1.0, 0.05)
    print(rmd.radius)
    print(rmd.radius_div_by_sqrt_2)
    print(rmd.diameter)
    print(rmd.fov_radius)
    print(rmd.footprint)
    print(rmd.footprint_2X)
