from geometry_msgs.msg import Point32, PoseStamped, Polygon as RosPolygon
from sensor_msgs import PointCloud 
from bresenham import bresenham
from shapely.geometry import MultiPoint, Polygon as ShapelyPolygon
from utils import Utils

import copy
import rospy

class Obstacle:
    idCounter = 1
    
    def __init__(self, frame_id, ros_point_cloud, ros_polygon, shapely_multipoint, map_points):
        # Obstacle unique id
        self.unique_id = Obstacle.idCounter
        Obstacle.idCounter = Obstacle.idCounter + 1
        
        # Copy obstacle representations from parameters
        self.frame_id = frame_id
        self.ros_point_cloud = ros_point_cloud
        self.ros_polygon = ros_polygon
        self.shapely_multipoint = shapely_multipoint
        self._point_set = 
        self.map_points = map_points
        
        # Create pose from centroid of shapely polygon
        self.pose = PoseStamped()
        self.pose.header.seq = self.unique_id
        self.pose.header.stamp = rospy.Time(0)
        self.pose.header.frame_id = frame_id
        self.pose.pose.position.x = shapely_multipoint.convex_hull.centroid.x
        self.pose.pose.position.y = shapely_multipoint.convex_hull.centroid.y
        
        # FIXME Compute grasp Points.
        self.graspPoints = None
        
        # FIXME Compute Bounding Box (rectangle with sides orthogonal to map axes)
        self.bbTopLeft = None
        self.bbBottomRight = None

        # FIXME Compute obstacle matrix and inflated obstacle
        self.obstacleMatrix = None
        self.robotInflatedObstacle = None
        
        # Obstacle semantic
        self.isMovable = True
        
    @classmethod
    # We assume that the ros_polygon has more than or has three points
    def from_ros_polygon(cls, ros_polygon, resolution, frame_id):
        # Create map points from the polygon coordinates
        map_points = set()
        for i in range(len(ros_polygon.points) - 1):
            line_points = list(
                bresenham(int(ros_polygon.points[i].x / resolution), # x1
                          int(ros_polygon.points[i].y / resolution), # y1
                          int(ros_polygon.points[i + 1].x / resolution),  # x2
                          int(ros_polygon.points[i + 1].y / resolution))) # y2
            for point in line_points:
                map_points.add(point)
        # Add the line between last point of the polygon and first point
        last_line = list(
                bresenham(int(ros_polygon.points[len(ros_polygon.points) - 1].x / resolution), # x1
                          int(ros_polygon.points[len(ros_polygon.points) - 1].y / resolution), # y1
                          int(ros_polygon.points[0].x / resolution),  # x2
                          int(ros_polygon.points[0].y / resolution))) # y2
        for point in last_line:
            map_points.add(point)
        
        # Create point cloud from map_points
        ros_point_cloud = PointCloud()
        ros_point_cloud.header.seq = 1
        ros_point_cloud.header.frame_id = frame_id
        ros_point_cloud.header.stamp = rospy.Time(0)
        
        points32 = []
        for point in map_points:
            new_point32 = Point32()
            new_point32.x = point[0] * resolution
            new_point32.y = point[1] * resolution
            points32.append(new_point32)
        ros_point_cloud.points = points32
        
        # Create shapely multipoint from point cloud
        shapely_polygon = Obstacle.ros_polygon_to_shapely_multipoint(ros_polygon)
        
        return cls(frame_id, ros_point_cloud, ros_polygon, shapely_multipoint, map_points)
        
    @classmethod
    def from_ros_point_cloud(cls, ros_point_cloud, resolution, frame_id, is_axis_rectangle = False):
        # Create map_points from the ros_point_cloud
        # and prepare points as acceptable format to create shapely polygon
        map_points = set()
        tuple_points_set = set() # We use this set to make sure there are no doubles
        for point32 in ros_point_cloud.points:
            map_point = (int(point32.x / resolution), int(point32.y / resolution))
            map_points.add(map_point)
            
            tuple_point = (point32.x, point32.y)
            tuple_points_set.add(tuple_point)
        
        # Create shapely multipoint
        shapely_multipoint = MultiPoint(list(tuple_points_set))
        
        # Create ros_polygon
        ros_polygon = Obstacle.shapely_multipoint_to_ros_polygon(shapely_multipoint, is_axis_rectangle)
        
        return cls(frame_id, ros_point_cloud, ros_polygon, shapely_multipoint, map_points)

    def add_points(self, points):
        
    def remove_points(self, points):
        
    def 

    @staticmethod
    def ros_polygon_to_shapely_multipoint(ros_polygon):
        points = []
        for point in ros_polygon.points:
            points.append((point.x, point.y))
        points.append((ros_polygon.points[0].x, ros_polygon.points[0].y))
        return ShapelyMultiPoint(points)
        
    @staticmethod
    def shapely_multipoint_to_ros_polygon(shapely_multipoint, is_axis_rectangle = False):
        shapely_polygon = shapely_multipoint.convex_hull if not is_axis_rectangle else shapely_multipoint.envelope
        ros_polygon = RosPolygon()
        points = set()
        for point in shapely_polygon.points:
            new_point = Point32()
            new_point.x = point.x
            new_point.y = point.y
            points.add(new_point)
        ros_polygon.points = list(points)
        return ros_polygon

    def __hash__(self):
        # Hash depends only on the unique and immutable id of the obstacle
        return self.id