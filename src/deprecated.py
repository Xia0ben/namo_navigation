###########################################
# DEPRECATED LASER MANAGEMENT CODE
###########################################
# Distance in meters, angle in radians
# @staticmethod
# def _laser_distance_to_local_coords(distance, angle):
#     return numpy.array([distance * math.cos(angle),
#                         distance * math.sin(angle)])
#
# def scan_callback(self, laserscan):
#     time  = rospy.Time(0) # Makes sure same tf is used for calls
#
#     robot_pose = None
#     # Get robot pose
#     try:
#         (position, quaternion) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)
#         robot_pose = SimplePose.from_pos_quat(position, quaternion, self.map_frame)
#     except (tf.Exception, tf.LookupException, tf.ConnectivityException):
#         rospy.logerr("Couldn't get robot pose in map : transform unavailable. \
#         Maybe amcl is not working and there is no transform between \
#         /odom and /map ?")
#         return # exit callback if error
#
#
#     # Convert laserscan readings into points for the map
#     pointsWithDistance = {}
#     for i in range(len(laserscan.ranges)):
#         distance = laserscan.ranges[i]
#
#         # Simply discard values that are not in the laser's specs
#         if distance >= laserscan.range_min or distance <= laserscan.range_max:
#         angle = laserscan.angle_min + laserscan.angle_increment * i
#         local_point = MapManager._laser_distance_to_local_coords(distance, angle)
#
#         pointstamp_local = PointStamped()
#         pointstamp_local.header.frame_id = self.laserscan_frame
#         pointstamp_local.header.stamp =  time
#         pointstamp_local.point.x = local_point[0]
#         pointstamp_local.point.y = local_point[1]
#         pointstamp_local.point.z = 0.0
#
#         try:
#             pointstamp_map = self.tf_listener.transformPoint(self.map_frame, pointstamp_local)
#
#             ### Conversion to map coordinates ###
#             # TODO : may need to fix the conversion if map is offsetted regarding to
#             # the world coordinates (here is done simply by the use of 2DPosition class)
#             pointsWithDistance[SimplePosition(pointstamp_map.point.x, pointstamp_map.point.y)] = distance
#         except (tf.LookupException, tf.ConnectivityException,
#                 tf.ExtrapolationException):
#             rospy.logerr("Couldn't add laserscan : transform unavailable. \
#         Maybe amcl is not working and there is no transform between \
#         /odom and /map ?")
#             return # exit callback if error
#
#     self.multilayered_map.update_laserscan(pointsWithDistance, robot_pose, self.obstacle_range, self.raytrace_range)
# def update_laserscan(self, pointsWithDistance, robot_pose, obstacle_range, raytrace_range):
#     updated_obstacle_occ_grid = copy.deepcopy(self.obstacle_occ_grid)
#     updated_obstacles = copy.deepcopy(self.obstacles)

#     ### Clearing ###
#     ## Raytracing method ##
#     # Use bresenham algorithm to trace lines in 2D:
#     # (https://en.wikipedia.org/wiki/Bresenham's_line_algorithm)
#     # list(bresenham) returns an array of tuples [(x,y), ...]
#     pointsToClear = set()
#     for point, distance in pointsWithDistance.items():
#         if distance <= raytrace_range:
#             line_points = list(
#                 bresenham(robot_pose.pos.coords[0], # x1
#                           robot_pose.pos.coords[1], # y1
#                           point.coords[0],  # x2
#                           point.coords[1])) # y2
#             for point in line_points:
#                 pointsToClear.add(point)

#     # pointsToClear is a set of tuples set([(x,y), ...])
#     new_cleared_points = []
#     for point in pointsToClear:
#         # TODO Maybe use a function that degrades over time, starting from
#         # 127 (free space) going to 0, over 20 seconds ?
#         # Would need to update inflation function to take this kind of case
#         # into account.
#         current_obstacle_id = self.updated_obstacle_occ_grid[point[0]][point[1]]
#         if current_obstacle_id > 0:
#             self.updated_obstacle_occ_grid[point[0]][point[1]] = MultilayeredMap.FREESPACE_COST_ROS
#             updated_obstacles[current_obstacle_id].removePoin # FIXME FINISH THIS LINE
#             new_cleared_points.append(SimplePosition(point[0], point[1]))

#     ## "Conetracing" method ##
#     # Draw polygons of successive clearable points of the laser_points list
#     # And remove all registered points within them
#     # TODO Implement this idea if appropriate

#     ### Marking and Static Map Overlap Checking ###
#     # Add points only if within marking distance, and that they are not in
#     # a pseudo inflated static occupation spot
#     for point, distance in pointsWithDistance.items():
#         isInRange = (distance <= obstacle_range)
#         isInMatrix = _is_in_matrix(point.coords[0], point.coords[1],
#                                   self.info.width, self.info.height)
#         isNotInPseudo = (self.pseudo_inflated_static_occ_grid[point.coords[0]][point.coords[1]] == 0)
#         if (isInRange and isInMatrix and isNotInPseudo):
#             self.updated_obstacle_occ_grid = MultilayeredMap.LETHAL_COST_ROS


#     # Clusterize points into separate obstacles
#     db = DBSCAN(eps=MultilayeredMap.DBSCAN_EPSILON,
#         min_samples=MultilayeredMap.DBSCAN_MIN_SAMPLES,
#         algorithm=MultilayeredMap.DBSCAN_ALGORITHM ,
#         metric=MultilayeredMap.DBSCAN_METRIC,
#         metric_params=MultilayeredMap.DBSCAN_METRIC_PARAMS,
#         leaf_size=MultilayeredMap.DBSCAN_LEAF_SIZE,
#         p=MultilayeredMap.DBSCAN_P,
#         n_jobs=MultilayeredMap.N_JOBS).fit(npObstaclePoints)

#     cluster_labels = db.labels_
#     n_clusters = len(set(cluster_labels)) - (1 if -1 in cluster_labels else 0)

#     # get the clusters
#     # cluster_labels = -1 means outliers, we ignore it
#     clusters = []
#     for cluster_label in range(n_clusters):
#         clusters.append([])

#     for i in range(len(cluster_labels)):
#         clusters[cluster_labels[i]].append(npObstaclePoints[i])

###########################################
# DEPRECATED OBSTACLE MANAGEMENT CODE
###########################################
from geometry_msgs.msg import Point32, PoseStamped, Polygon as RosPolygon
from sensor_msgs import PointCloud
from bresenham import bresenham
from shapely.geometry import MultiPoint, Polygon as ShapelyPolygon
from utils import Utils

import copy
import rospy

class Obstacle:
    idCounter = 1

    def __init__(self, points_set, resolution, obstacle_id):
        # Obstacle unique id
        self.unique_id = obstacle_id

        # Copy obstacle representations from parameters
        self.points_set = points_set

        # Create alternate representations
        self.shapely_multipoint = MultiPoint(list(points_set))

        self.map_points_set = map_points_set = set()
        for point in points_set:
            map_points_set.add((int(point[0] / resolution), int(point[1]/ resolution)))

        self.ros_point_cloud = PointCloud()
        self.ros_point_cloud.header.seq = 1
        self.ros_point_cloud.header.frame_id = frame_id
        self.ros_point_cloud.header.stamp = rospy.Time(0)
        self.ros_point_cloud.points = []
        for point in points_set:
            new_point32 = Point32()
            new_point32.x = point[0]
            new_point32.y = point[1]
            self.ros_point_cloud.points.append(new_point32)
        ros_point_cloud.points = points32

        # Create pose from centroid of shapely polygon
        self.pose = PoseStamped()
        self.pose.header.seq = 1
        self.pose.header.stamp = rospy.Time(0)
        self.pose.header.frame_id = frame_id
        self.pose.pose.position.x = shapely_multipoint.convex_hull.centroid.x
        self.pose.pose.position.y = shapely_multipoint.convex_hull.centroid.y

        # Compute grasp Points, which are the centers of each side of the convex hull
        self.graspPoints = None

        # FIXME Compute Bounding Box (rectangle with sides orthogonal to map axes)
        self.bbTopLeft = None
        self.bbBottomRight = None

        # FIXME Compute obstacle matrix and inflated obstacle
        self.obstacleMatrix = None
        self.robotInflatedObstacle = None

        # Obstacle semantic
        self.isMovable = True


    # @classmethod
    # # We assume that the ros_polygon has more than or has three points
    # def from_ros_polygon(cls, ros_polygon, resolution, frame_id):
    #     # Create map points from the polygon coordinates
    #     map_points = set()
    #     for i in range(len(ros_polygon.points) - 1):
    #         line_points = list(
    #             bresenham(int(ros_polygon.points[i].x / resolution), # x1
    #                       int(ros_polygon.points[i].y / resolution), # y1
    #                       int(ros_polygon.points[i + 1].x / resolution),  # x2
    #                       int(ros_polygon.points[i + 1].y / resolution))) # y2
    #         for point in line_points:
    #             map_points.add(point)
    #     # Add the line between last point of the polygon and first point
    #     last_line = list(
    #             bresenham(int(ros_polygon.points[len(ros_polygon.points) - 1].x / resolution), # x1
    #                       int(ros_polygon.points[len(ros_polygon.points) - 1].y / resolution), # y1
    #                       int(ros_polygon.points[0].x / resolution),  # x2
    #                       int(ros_polygon.points[0].y / resolution))) # y2
    #     for point in last_line:
    #         map_points.add(point)

    #     # Create point cloud from map_points
    #     ros_point_cloud = PointCloud()
    #     ros_point_cloud.header.seq = 1
    #     ros_point_cloud.header.frame_id = frame_id
    #     ros_point_cloud.header.stamp = rospy.Time(0)

    #     points32 = []
    #     for point in map_points:
    #         new_point32 = Point32()
    #         new_point32.x = point[0] * resolution
    #         new_point32.y = point[1] * resolution
    #         points32.append(new_point32)
    #     ros_point_cloud.points = points32

    #     # Create shapely multipoint from point cloud
    #     shapely_polygon = Obstacle.ros_polygon_to_shapely_multipoint(ros_polygon)

    #     return cls(frame_id, ros_point_cloud, ros_polygon, shapely_multipoint, map_points)

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

    def add_point(self, point):

    def remove_points(self, points):

    def has_point(self, point):


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
