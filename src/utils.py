import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Polygon as RosPolygon, Point32, Point, PolygonStamped
from nav_msgs.msg import Path as RosPath, GridCells

class Utils:
    TF_LISTENER = tf.TransformListener()

    # UINT_TO_INT = 128
    # ROS_COST_LETHAL = 254 - UINT_TO_INT
    # ROS_COST_INSCRIBED = 253 - UINT_TO_INT
    # ROS_COST_POSSIBLY_CIRCUMSCRIBED = 128 - UINT_TO_INT
    # ROS_COST_POSSIBLY_NONFREE = 1 - UINT_TO_INT
    # ROS_COST_FREE_SPACE = 0 - UINT_TO_INT

    ROS_COST_LETHAL = 100
    ROS_COST_INSCRIBED = 99
    ROS_COST_POSSIBLY_CIRCUMSCRIBED = 51
    ROS_COST_POSSIBLY_NONFREE = 1
    ROS_COST_FREE_SPACE = 0

    """
    The following loop is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems there is no big deal but in systems that publish only
    once it IS very important.
    """
    @staticmethod
    def publish_once(publisher, payload):
        while True:
            connections = publisher.get_num_connections()
            if connections > 0:
                publisher.publish(payload)
                break

    @staticmethod
    def get_current_pose(map_frame, robot_frame):

        time  = rospy.Time(0) # Makes sure same tf is used for calls
        robot_pose = PoseStamped()

        while True:
            try:
                (position, quaternion) = Utils.TF_LISTENER.lookupTransform(map_frame, robot_frame, time)

                robot_pose.header.seq = 1
                robot_pose.header.stamp = rospy.Time(0)
                robot_pose.header.frame_id = map_frame
                robot_pose.pose.position.x = position[0]
                robot_pose.pose.position.y = position[1]
                robot_pose.pose.position.z = position[2]
                robot_pose.pose.orientation.x = quaternion[0]
                robot_pose.pose.orientation.y = quaternion[1]
                robot_pose.pose.orientation.z = quaternion[2]
                robot_pose.pose.orientation.w = quaternion[3]

                break

            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                rospy.loginfo("Couldn't get robot pose in map : transform unavailable. \
                Maybe amcl is not working and there is no transform between \
                /odom and /map ?")

        return robot_pose

    @staticmethod
    def yaw_from_geom_quat(geom_quat):
        explicit_quat = [geom_quat.x,
                         geom_quat.y,
                         geom_quat.z,
                         geom_quat.w]
        return tf.transformations.euler_from_quaternion(explicit_quat)[2]

    @staticmethod
    def geom_quat_from_yaw(yaw):
        explicit_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        geom_quat = Quaternion()
        geom_quat.x = explicit_quat[0]
        geom_quat.y = explicit_quat[1]
        geom_quat.z = explicit_quat[2]
        geom_quat.w = explicit_quat[3]
        return geom_quat

    @staticmethod
    def ros_path_from_poses(start_pose, end_pose, frame_id):
        ros_path = RosPath()
        ros_path.header.seq = 1
        ros_path.header.stamp = rospy.Time(0)
        ros_path.header.frame_id = frame_id
        ros_path.poses = [start_pose, end_pose]
        return ros_path

    @staticmethod
    def ros_path_from_map_path(map_path, start_pose, end_pose, resolution, frame_id):
        ros_path = RosPath()
        ros_path.header.seq = 1
        ros_path.header.stamp = rospy.Time(0)
        ros_path.header.frame_id = frame_id

        poses = []

        # If path is not empty, add poses, otherwise ros path stays empty
        if map_path:
            poses = [start_pose]

            id_counter = 1
            for point in map_path[1:len(map_path) - 1]:
                new_pose = Utils.ros_pose_from_map_coord(point[0], point[1], resolution, id_counter, frame_id)
                id_counter = id_counter + 1
                poses.append(new_pose)

            poses.append(end_pose)

        ros_path.poses = poses

        return ros_path

    @staticmethod
    def map_path_from_ros_path(ros_path, resolution):
        map_path = []

        for pose in ros_path.poses:
            map_path.append(Utils.map_coord_from_ros_pose(pose, resolution))

        return map_path


    ##################### TODO MAKE USE OF THESE METHODS EVERYWHERE
    @staticmethod
    def real_to_map(realX, realY, resolution):
        return int(realX / resolution), int(realY / resolution)

    @staticmethod
    def map_to_real(mapX, mapY, resolution):
        return resolution * (float(mapX) + 0.5), resolution * (float(mapY) + 0.5)

    @staticmethod
    def real_coords_to_map_coords(real_coords_set, resolution):
        map_coords_set = set()
        for real_coords in real_coords_set:
            map_coords_set.add(Utils.real_to_map(real_coords[0], real_coords[1], resolution))
        return map_coords_set

    @staticmethod
    def map_coords_to_real_coords(map_coords_set, resolution):
        real_coords_set = set()
        for map_coords in map_coords_set:
            real_coords_set.add(Utils.map_to_real(map_coords[0], map_coords[1], resolution))
        return real_coords_set

    ##################### ENDOFTODO MAKE USE OF THESE METHODS EVERYWHERE

    @staticmethod
    def ros_pose_from_map_coord(coordX, coordY, resolution, point_id, frame_id):
        new_pose = PoseStamped()
        new_pose.header.seq = point_id
        new_pose.header.stamp = rospy.Time(0)
        new_pose.header.frame_id = frame_id
        new_pose.pose.position.x = resolution * (float(coordX) + 0.5)
        new_pose.pose.position.y = resolution * (float(coordY) + 0.5)
        new_pose.pose.orientation.x = 0.0
        new_pose.pose.orientation.y = 0.0
        new_pose.pose.orientation.z = 0.0
        new_pose.pose.orientation.w = 0.0

        return new_pose

    @staticmethod
    def map_coord_from_ros_pose(ros_pose, resolution):
        coordX = int(ros_pose.pose.position.x / resolution)
        coordY = int(ros_pose.pose.position.y / resolution)
        return coordX, coordY

    @staticmethod
    def euclidean_distance_ros_poses(init_pose, goal_pose):
        return Utils.euclidean_distance(
            (init_pose.pose.position.x, init_pose.pose.position.y),
            (goal_pose.pose.position.x, goal_pose.pose.position.y))

    @staticmethod
    def euclidean_distance(point_tuple_a, point_tuple_b):
        return math.sqrt((point_tuple_b[0] - point_tuple_a[0]) ** 2 + (point_tuple_b[1] - point_tuple_a[1]) ** 2)

    # Works best if footprint has odd width and height
    # Basically, you just stamp the inflation footprint on each obstacle point
    @staticmethod
    def _get_inflastamped_matrix(points, footprint, width, height, offsetX = 0, offsetY = 0, remove_original_points = False):
        # TODO a funny way to implement even sized footprints would be to
        # add a column and/or line in the middle by copying one of the
        # columns/lines of the middle of the footprint

        matrix = np.full((width, height), Utils.ROS_COST_FREE_SPACE, dtype=int)

        for point in points:
            xInM = point[0] - offsetX
            yInM = point[1] - offsetY
            matrix[xInM][yInM] = Utils.ROS_COST_LETHAL

        fWidthLeft = len(footprint) / 2 if len(footprint) % 2 == 0 else (len(footprint) - 1) / 2
        fHeightLeft = len(footprint[0]) / 2 if len(footprint[0]) % 2 == 0 else (len(footprint[0]) - 1) / 2
        fWidthRight = len(footprint) / 2
        fHeightRight = len(footprint[0]) / 2

        # Inflate the obstacle <=> apply footprint at every obstacle point
        for point in points:
            xInM = point[0] - offsetX
            yInM = point[1] - offsetY

            # Apply the footprint around the point
            xF, yF = 0, 0 # Current coordinates in footprint
            for xM in range(xInM - fWidthLeft, xInM + fWidthRight + 1):
                for yM in range(yInM - fHeightLeft, yInM + fHeightRight + 1):
                    if Utils._is_in_matrix(xM, yM, width, height) and footprint[xF][yF] > matrix[xM][yM]:
                        matrix[xM][yM] = footprint[xF][yF]
                    yF = yF + 1
                xF = xF + 1
                yF = 0
            xF = 0

        # Remove points that correspond to the obstacle itself if asked for it
        # Do not combine with above loop : there would be overwriting
        if remove_original_points:
            for point in points:
                xInM = point[0] - offsetX
                yInM = point[1] - offsetY
                matrix[xInM][yInM] = 0

        return matrix

    @staticmethod
    def _is_in_matrix(x, y, width, height) :
        return True if (x >= 0 and x < width and y >= 0 and y < height) else False

    @staticmethod
    def get_corners_float_coords(int_coord_x, int_coord_y):
        return [(float(int_coord_x), float(int_coord_y)), (float(int_coord_x) + 1.0, float(int_coord_y)), (float(int_coord_x) + 1.0, float(int_coord_y) + 1.0), (float(int_coord_x), float(int_coord_y) + 1.0)]

    @staticmethod
    def get_corners_world_coords(int_coord_x, int_coord_y, resolution):
        float_coord_x = resolution * float(int_coord_x)
        float_coord_y = resolution * float(int_coord_y)
        return {(float_coord_x, float_coord_y),
                (float_coord_x + resolution, float_coord_y),
                (float_coord_x + resolution, float_coord_y + resolution),
                (float_coord_x, float_coord_y + resolution)}

    @staticmethod
    def debug_publish_path(path, topic):
        Utils.publish_path(path, topic)

    @staticmethod
    def publish_plan(plan):
        if len(plan.components) == 1:
            p_opt_pub = rospy.Publisher("/simulated/p_opt_no", RosPath, queue_size=1)
            Utils.publish_once(p_opt_pub, plan.components[0].path)
        elif len(plan.components) == 3:
            c1_pub = rospy.Publisher("/simulated/p_opt_c1", RosPath, queue_size=1)
            c2_pub = rospy.Publisher("/simulated/p_opt_c2", RosPath, queue_size=1)
            c3_pub = rospy.Publisher("/simulated/p_opt_c3", RosPath, queue_size=1)
            Utils.publish_once(c1_pub, plan.components[0].path)
            Utils.publish_once(c2_pub, plan.components[1].path)
            Utils.publish_once(c3_pub, plan.components[2].path)

    @staticmethod
    def publish_path(path, topic):
        pub = rospy.Publisher(topic, RosPath, queue_size=1)
        Utils.publish_once(pub, path.path)

    @staticmethod
    def debug_publish_real_coords(real_coords, topic):
        grid_cells = GridCells()
        grid_cells.header.seq = 1
        grid_cells.header.stamp = rospy.Time(0)
        grid_cells.header.frame_id = "/map"
        grid_cells.cell_width = 0.01
        grid_cells.cell_height = grid_cells.cell_width
        for real_coord in real_coords:
            point = Point()
            point.x = real_coord[0]
            point.y = real_coord[1]
            grid_cells.cells.append(point)
        pub = rospy.Publisher(topic, GridCells, queue_size=1)
        Utils.publish_once(pub, grid_cells)

    @staticmethod
    def debug_publish_map_coords(map_coords, topic):
        grid_cells = GridCells()
        grid_cells.header.seq = 1
        grid_cells.header.stamp = rospy.Time(0)
        grid_cells.header.frame_id = "/map"
        grid_cells.cell_width = 0.05
        grid_cells.cell_height = grid_cells.cell_width
        real_coords = Utils.map_coords_to_real_coords(map_coords, grid_cells.cell_width)
        for real_coord in real_coords:
            point = Point()
            point.x = real_coord[0]
            point.y = real_coord[1]
            grid_cells.cells.append(point)
        pub = rospy.Publisher(topic, GridCells, queue_size=1)
        Utils.publish_once(pub, grid_cells)

    @staticmethod
    def debug_publish_shapely_as_ros_polygon(shapely_polygon, topic):
        ros_polygon = Utils.convert_shapely_as_ros_polygon(shapely_polygon)
        pub = rospy.Publisher(topic, PolygonStamped, queue_size=1)
        Utils.publish_once(pub, ros_polygon)

    @staticmethod
    def convert_shapely_as_ros_polygon(shapely_polygon):
        polygon_stamped = PolygonStamped()
        polygon_stamped.header.seq = 1
        polygon_stamped.header.stamp = rospy.Time(0)
        polygon_stamped.header.frame_id = "/map"
        ros_polygon = RosPolygon()
        coords = list(shapely_polygon.exterior.coords)
        for coord in coords:
            point = Point32()
            point.x = coord[0]
            point.y = coord[1]
            ros_polygon.points.append(point)
        polygon_stamped.polygon = ros_polygon
        return polygon_stamped

