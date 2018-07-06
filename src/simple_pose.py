from geometry_msgs.msg import PoseWithCovarianceStamped
from simple_position import SimplePosition

class SimplePose:
    seqCounter = 1
    DEFAULT_FRAME_ID = "/map"

    def __init__(self, x, y, rot=0, frame_id=SimplePose.DEFAULT_FRAME_ID):
        self.pos = SimplePosition(x, y)
        self.rot = float(rot) # Yaw
        self.frame_id = frame_id

    @classmethod
    def from_pos_quat(cls, position, quaternion, frame_id):
        x = position.x
        y = position.y
        rot = tf.transformations.euler_from_quaternion(
                quaternion)[2]
        return cls(x, y, rot, frame_id)

    @classmethod
    def from_ros_pose(cls, pose):
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        rot = tf.transformations.euler_from_quaternion(
                pose.pose.pose.orientation)[2]
        frame_id = pose.header.frame_id
        return cls(x, y, rot, frame_id)

    @classmethod
    def from_array(cls, array):
        return cls(pose[0], pose[1]))
        
    @classmethod
    def from_position(cls, position):
        return cls(position.coords[0], position.coords[1]))

    def getCorrespondingRosPose():
        rosPose = PoseWithCovarianceStamped()
        rosPose.header.seq = seqCounter
        Pose.seqCounter = Pose.seqCounter + 1
        rosPose.header.frame_id = self.frame_id

        rosPose.pose.pose.position.x = float(self.pos.coords[0])
        rosPose.pose.pose.position.y = float(self.pos.coords[1])
        rosPose.pose.pose.position.z = 0.0
        rosPose.pose.pose.orientation = tf.transformations.quaternion_from_euler(
            0, 0, self.rot)

        rosPose.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        return rosPose