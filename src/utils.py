import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path

class Utils:
    TF_LISTENER = tf.TransformListener()
    
    
    """
    The following loop is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems there is no big deal but in systems that publish only
    once it IS very important.
    """
    @staticmethod
    def publish_once(publisher, payload):
        while True:
            connections = self.publisher.get_num_connections()
            if connections > 0:
                self.publisher.publish(payload)
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
    def ros_path_from_map_path(map_path, start_pose, end_pose, resolution, frame_id):
        ros_path = Path()
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
    def distance_between_ros_poses(init_pose, goal_pose):
        return numpy.linalg.norm(
            numpy.array([goal_pose.pose.position.x, goal_pose.pose.position.y]) -
            numpy.array([init_pose.pose.position.x, init_pose.pose.position.y])
        )
