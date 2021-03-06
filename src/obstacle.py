from geometry_msgs.msg import Point32, PoseStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from bresenham import bresenham
from shapely.geometry import Point, MultiPoint, LineString
from shapely import affinity
from utils import Utils
from enum import Enum
import rospy
import copy
import math

Movability = Enum('Movability', 'maybe_movable movable unmovable')

class Obstacle:

    def __init__(self, points_set, map_metadata, frame_id, robot_metadata, obstacle_id, movability, axis_rect_hypothesis = True):
        # Copy obstacle representations from parameters
        self.points_set = points_set
        self.map_metadata = map_metadata
        self.frame_id = frame_id
        self.robot_metadata = robot_metadata
        self.obstacle_id = obstacle_id
        self.axis_rect_hypothesis = axis_rect_hypothesis

        # Obstacle semantic
        self.movability = movability

        # Create subsequent properties
        self._shapely_multipoint = None
        self.polygon = None
        self.envelope = None
        self.map_points_set = None
        self.bb_top_left_corner = None
        self.bb_bottom_right_corner = None
        self.discretized_polygon = None
        self.ros_point_cloud = None
        self.ros_pose = None
        self.push_poses = None
        self.obstacle_matrix = None
        self.robot_inflated_obstacle = None

        # Update subsequent properties
        self._update_alternate_representations()

    # CLASS METHOD TO CREATE POLYGON SET OF POINTS TO GIVE TO CONSTRUCTOR
    @classmethod
    def from_points_make_polygon(cls, polygon_vertices_set, map_metadata, frame_id, robot_metadata, obstacle_id, movability, axis_rect_hypothesis = True):
        shapely_multipoint = MultiPoint(list(polygon_vertices_set))
        discretized_polygon = Obstacle._make_discretized_polygon(shapely_multipoint.convex_hull, shapely_multipoint.convex_hull, map_metadata.resolution)
        points_set = Utils.map_coords_to_real_coords(discretized_polygon, map_metadata.resolution)
        return cls(points_set, map_metadata, frame_id, robot_metadata, obstacle_id, movability, axis_rect_hypothesis)

    # We assume that the robot goes in a straight line when moving the object,
    # which allows us to use a convex hull to determine the safe swept area,
    # rather than a concave hull which is computationaly more challenging.
    # See : http://blog.thehumangeo.com/2014/05/12/drawing-boundaries-in-python/
    def _get_manipulation_polygon(self, translation_vector):
        current_polygon = copy.deepcopy(self.polygon)
        new_polygon = affinity.translate(current_polygon, translation_vector[0], translation_vector[1])
        manipulation_polygon = current_polygon.union(new_polygon).convex_hull
        return manipulation_polygon

    def get_manipulation_area_map_points(self, translation_vector):
        manipulation_area_map_points = set()
        manipulation_polygon = self._get_manipulation_polygon(translation_vector)

        if type(manipulation_polygon) is LineString:
            coords = list(manipulation_polygon.coords)

            # Check if manipulation causes to be out of map bounds
            # If yes, return empty set
            if (not Utils._is_in_matrix(coords[0][0], coords[0][1], self.map_metadata.width, self.map_metadata.height) or
                not Utils._is_in_matrix(coords[1][0], coords[1][1], self.map_metadata.width, self.map_metadata.height)):
                return set()

            manipulation_area_map_points = set(
                bresenham(int(coords[0][0] / self.map_metadata.resolution), # x1
                          int(coords[0][1] / self.map_metadata.resolution), # y1
                          int(coords[1][0] / self.map_metadata.resolution),  # x2
                          int(coords[1][1] / self.map_metadata.resolution))) # y2
        else:
            # Utils.debug_publish_shapely_as_ros_polygon(manipulation_polygon.envelope, "/simulated/debug_polygon")
            x, y = manipulation_polygon.envelope.exterior.xy
            bb_top_left_corner = (int(min(x) / self.map_metadata.resolution), int(min(y) / self.map_metadata.resolution))
            bb_bottom_right_corner = (int(max(x) / self.map_metadata.resolution), int(max(y) / self.map_metadata.resolution))
            # Utils.debug_publish_map_coords({bb_top_left_corner, bb_bottom_right_corner}, "/simulated/debug_map_coords")

            # Check if manipulation causes to be out of map bounds
            # If yes, return empty set
            if (not Utils._is_in_matrix(bb_top_left_corner[0], bb_top_left_corner[1], self.map_metadata.width, self.map_metadata.height) or
                not Utils._is_in_matrix(bb_bottom_right_corner[0], bb_bottom_right_corner[1], self.map_metadata.width, self.map_metadata.height)):
                return set()

            for i in range(bb_top_left_corner[0], bb_bottom_right_corner[0] + 1):
                for j in range(bb_top_left_corner[1], bb_bottom_right_corner[1] + 1):
                    for corner in Utils.get_corners_world_coords(i, j, self.map_metadata.resolution):
                        if manipulation_polygon.contains(Point(corner)):
                            manipulation_area_map_points.add((i, j))

        # Utils.debug_publish_map_coords(manipulation_area_map_points, "/simulated/debug_map_coords")

        return manipulation_area_map_points

    def compare(self, obstacle):
        if obstacle.obstacle_id == self.obstacle_id and obstacle.points_set == self.points_set:
            return True
        return False

    def is_obstacle_push_pose(self, push_pose_to_check):
        for push_pose in self.push_poses:
            if (push_pose.pose.position.x == push_pose_to_check.pose.position.x and
                    push_pose.pose.position.y == push_pose_to_check.pose.position.y and
                    push_pose.pose.position.z == push_pose_to_check.pose.position.z and
                    push_pose.pose.orientation.x == push_pose_to_check.pose.orientation.x and
                    push_pose.pose.orientation.y == push_pose_to_check.pose.orientation.y and
                    push_pose.pose.orientation.z == push_pose_to_check.pose.orientation.z and
                    push_pose.pose.orientation.w == push_pose_to_check.pose.orientation.w):
                return True
        return False

    def move(self, translation):
        points_set_after = set()
        for point in self.points_set:
            points_set_after.add((point[0] + translation[0], point[1] + translation[1]))

        self.points_set = points_set_after

        # Update all other alternative representations
        self._update_alternate_representations()

    def add_points(self, to_add_points_set):
        # Update points_set
        self.points_set = self.points_set | to_add_points_set

        # Update all other alternative representations
        self._update_alternate_representations()

    def remove_points(self, to_remove_points_set):
        # Update points_set
        self.points_set = self.points_set - to_remove_points_set

        # Update all other alternative representations
        self._update_alternate_representations()

    def _update_alternate_representations(self):
        # TODO ADD CHECK THAT ALL POINTS FROM self.points_set ARE INDEED WITHIN THE MAP !!!
        # IF NOT, SEND EXCEPTION (WILL HAVE TO CATCH THIS EXCEPTION ANYWHERE AN OBSTACLE IS CREATED OR UPDATED)

        # Shapely multipoint for determining envelope, convex hull, centroid
        self._shapely_multipoint = MultiPoint(list(self.points_set))

        # Envelope/Bounding box of the obstacle and convex hull
        self.envelope = self._shapely_multipoint.envelope
        self.convex_hull = self._shapely_multipoint.convex_hull

        # Polygonal reprentation of the obstacle
        self.polygon = self.envelope if self.axis_rect_hypothesis else self.convex_hull

        # Points in integer map coordinates
        self.map_points_set = set()
        for point in self.points_set:
            self.map_points_set.add((int(point[0] / self.map_metadata.resolution),
                int(point[1] / self.map_metadata.resolution)))

        # Envelope corners coordinates
        if type(self.envelope) is Point:
            x, y = self.envelope.xy
        else:
            x, y = self.envelope.exterior.xy
        self.bb_top_left_corner = (int(min(x) / self.map_metadata.resolution), int(min(y) / self.map_metadata.resolution))
        self.bb_bottom_right_corner = (int(max(x) / self.map_metadata.resolution), int(max(y) / self.map_metadata.resolution))

        # Discretized polygon as points in integer map coordinates
        self.discretized_polygon = Obstacle._make_discretized_polygon(self.convex_hull, self.polygon, self.map_metadata.resolution)

        ## Dependency on previously set attributes is specified as function call parameter

        # Points as ROS point cloud
        self.ros_point_cloud = self._make_ros_point_cloud()

        # Create pose from centroid of shapely polygon
        self.ros_pose = self._make_ros_pose(self.polygon)

        # Compute push poses, which are perpendicularly positioned at a
        # robot_radius distance from the middle point of each side of the
        # obstacle polygon representation, and directed toward the same middle
        # point.
        self.push_poses = self._make_push_poses(self.convex_hull, self.polygon, self.map_metadata.resolution)

        # Compute obstacle matrix and inflated obstacle
        self.obstacle_matrix = self._make_inflated_obstacle_grid(self.bb_top_left_corner, self.bb_bottom_right_corner, self.discretized_polygon,  self.robot_metadata.footprint_2X, removeObstaclePoints = True)
        self.robot_inflated_obstacle = self._make_inflated_obstacle_grid(self.bb_top_left_corner, self.bb_bottom_right_corner, self.discretized_polygon, self.robot_metadata.footprint, removeObstaclePoints = False)

    @staticmethod
    def _make_discretized_polygon(convex_hull, polygon, resolution):
        discretized_polygon = set()
        if type(convex_hull) is Point:
            discretized_polygon = {(int(convex_hull.x / resolution),
                int(convex_hull.y / resolution))}
            return discretized_polygon
        elif type(convex_hull) is LineString:
            polygon_vertices = list(convex_hull.coords)
        else:
            polygon_vertices = list(polygon.exterior.coords)

        for i in range(len(polygon_vertices) - 1):
            discretized_polygon = discretized_polygon | set(
                bresenham(int(polygon_vertices[i][0] / resolution), # x1
                          int(polygon_vertices[i][1] / resolution), # y1
                          int(polygon_vertices[i + 1][0] / resolution),  # x2
                          int(polygon_vertices[i + 1][1] / resolution))) # y2
        return discretized_polygon

    def _make_ros_point_cloud(self):
        ros_point_cloud = PointCloud()
        ros_point_cloud.header.seq = 1
        ros_point_cloud.header.frame_id = self.frame_id
        ros_point_cloud.header.stamp = rospy.Time.now()
        ros_point_cloud.points = []
        ros_point_cloud.channels = [ChannelFloat32()]
        ros_point_cloud.channels[0].name = "obstacle_id"
        ros_point_cloud.channels[0].values = []
        for point in self.points_set:
            new_point32 = Point32()
            new_point32.x = point[0]
            new_point32.y = point[1]
            ros_point_cloud.points.append(new_point32)
            ros_point_cloud.channels[0].values.append(self.obstacle_id)
        return ros_point_cloud

    def _make_ros_pose(self, polygon):
        ros_pose = PoseStamped()
        ros_pose.header.seq = 1
        ros_pose.header.stamp = rospy.Time.now()
        ros_pose.header.frame_id = self.frame_id
        ros_pose.pose.position.x = polygon.centroid.x
        ros_pose.pose.position.y = polygon.centroid.y
        return ros_pose

    def _make_push_poses(self, convex_hull, polygon, resolution):
        # FIXME Add verification that the push poses is not occupied in the merged occupation grid

        push_poses = []

        distance_from_border = self.robot_metadata.radius + resolution / 10.0 # WORKAROUND

        if type(convex_hull) is Point:
            center = convex_hull
            polygon_sides_centroids = [(center.x, center.y),
                                       (center.x, center.y),
                                       (center.x, center.y),
                                       (center.x, center.y)]

            beveled_polygon_sides_centroids = [(center.x - distance_from_border, center.y),
                                               (center.x, center.y + distance_from_border),
                                               (center.x + distance_from_border, center.y),
                                               (center.x, center.y - distance_from_border)]
        elif type(convex_hull) is LineString:
            center = convex_hull.centroid
            polygon_sides_centroids = [(center.x, center.y),
                                       (center.x, center.y)]

            beveled_polygon = convex_hull.buffer(distance_from_border, join_style = 3, cap_style = 2)

            beveled_polygon_sides_centroids = []
            for i in range(1, len(beveled_polygon.exterior.coords) - 1, 2):
                current_side = LineString([beveled_polygon.exterior.coords[i], beveled_polygon.exterior.coords[i+1]])
                beveled_polygon_sides_centroids.append((current_side.centroid.x, current_side.centroid.y))
        else:
            beveled_polygon = polygon.buffer(distance_from_border, join_style = 3, cap_style = 3)

            polygon_sides_centroids = []
            for i in range(len(polygon.exterior.coords) - 1):
                current_side = LineString([polygon.exterior.coords[i], polygon.exterior.coords[i+1]])
                polygon_sides_centroids = [(current_side.centroid.x, current_side.centroid.y)] + polygon_sides_centroids

            beveled_polygon_sides_centroids = []
            for i in range(0, len(beveled_polygon.exterior.coords) - 1, 2):
                current_side = LineString([beveled_polygon.exterior.coords[i], beveled_polygon.exterior.coords[i+1]])
                beveled_polygon_sides_centroids.append((current_side.centroid.x, current_side.centroid.y))

        for p_side_centroid, b_side_centroid in zip(polygon_sides_centroids, beveled_polygon_sides_centroids):
            new_push_pose = PoseStamped()
            new_push_pose.header.seq = 1
            new_push_pose.header.stamp = rospy.Time.now()
            new_push_pose.header.frame_id = self.frame_id
            new_push_pose.pose.position.x = b_side_centroid[0]
            new_push_pose.pose.position.y = b_side_centroid[1]

            observation_direction = (p_side_centroid[0] - b_side_centroid[0],
                                     p_side_centroid[1] - b_side_centroid[1])

            if observation_direction[1] < 0:
                yaw = 2 * math.pi - math.acos(observation_direction[0]/math.sqrt(observation_direction[0] ** 2 + observation_direction[1] ** 2))
            else:
                yaw = math.acos(observation_direction[0]/math.sqrt(observation_direction[0] ** 2 + observation_direction[1] ** 2))

            new_push_pose.pose.orientation = Utils.geom_quat_from_yaw(yaw)

            push_poses.append(new_push_pose)

        return push_poses

    def _make_inflated_obstacle_grid(self, bb_top_left_corner, bb_bottom_right_corner, discretized_polygon, footprint, removeObstaclePoints = False):
        inflation_width = len(footprint) / 2
        inflation_heigth = len(footprint[0]) / 2

        matrixTopLeftX = ((bb_top_left_corner[0] - inflation_width)
            if (bb_top_left_corner[0] - inflation_width) >= 0 else 0)
        matrixTopLeftY = ((bb_top_left_corner[1] - inflation_heigth)
            if (bb_top_left_corner[1] - inflation_heigth) >= 0 else 0)
        matrixBottomRightX = ((bb_bottom_right_corner[0] + inflation_width + 1)
            if (bb_bottom_right_corner[0] + inflation_width + 1) < self.map_metadata.width else self.map_metadata.width)
        matrixBottomRightY = ((bb_bottom_right_corner[1] + inflation_heigth + 1)
            if (bb_bottom_right_corner[1] + inflation_heigth + 1) < self.map_metadata.height else self.map_metadata.height)

        width = matrixBottomRightX - matrixTopLeftX
        height = matrixBottomRightY - matrixTopLeftY

        return {"top_left_corner": (matrixTopLeftX, matrixTopLeftY),
                "bottom_right_corner": (matrixBottomRightX, matrixBottomRightY),
                "matrix": Utils._get_inflastamped_matrix(discretized_polygon,
                                                         footprint,
                                                         width,
                                                         height,
                                                         matrixTopLeftX,
                                                         matrixTopLeftY,
                                                         removeObstaclePoints)}

    def __hash__(self):
        # Hash depends only on the unique and immutable id of the obstacle
        return self.obstacle_id
