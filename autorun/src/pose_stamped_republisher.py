import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

PALN_B = True

class Republisher:
    def __init__(self):
        rospy.init_node('pose_republisher')
        self.covariance = rospy.get_param("pose_stamped_covariance")

        self.pub = rospy.Publisher('/pose_stamped_covariance', PoseWithCovarianceStamped, queue_size=5)
        rospy.Subscriber('/pose_stamped', PoseStamped, self.callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.imu_orientation = Quaternion(0, 0, 0, 1)

    def callback(self, data):
        msg = PoseWithCovarianceStamped()
        msg.header = data.header
        msg.pose.pose.position = data.pose.position
        if PALN_B:
            msg.pose.pose.orientation = self.imu_orientation
        else:
            msg.pose.pose.orientation = data.pose.orientation
        msg.pose.covariance = self.covariance

        self.pub.publish(msg)

    def imu_callback(self, data):
        self.imu_orientation = data.orientation


if __name__ == '__main__':
    node = Republisher()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    