import rospy
from aruco_opencv_msgs.msg import ArucoDetection
from aruco_opencv_msgs.srv import MarkerSize, MarkerSizeResponse
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import numpy as np
import tf.transformations as tr
import tf, tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class MapPublisher():
    def __init__(self):
        rospy.init_node('robot_pose_publisher')

        self.robot_id        = rospy.get_param("robot_marker_id", 1)
        self.pose_covariance = rospy.get_param("/camera_pose_covariance")

        rospy.loginfo(f"Robot marker id is: {self.robot_id}")

        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.marker_size = rospy.get_param("/robot_marker_size", 0.07)

        rospy.loginfo("Waiting for map callibration")
        rospy.wait_for_service('map_is_callibrated')
        rospy.loginfo("Map is callibrated, start robot pose detection")

        rospy.wait_for_service('set_marker_size')
        try:
            set_marker_size = rospy.ServiceProxy('set_marker_size', MarkerSize)
            resp1 = set_marker_size(self.marker_size)
            rospy.loginfo(resp1.response)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

        self.robot_sub      = rospy.Subscriber("aruco_detections", ArucoDetection, self.arucoCallback)
        self.robot_pose_pub = rospy.Publisher("robot_pose_aruco", PoseWithCovarianceStamped, queue_size=5)
            

    def arucoCallback(self, data):
        markers = data.markers

        robot_marker = None
        for marker in markers:
            if marker.marker_id == self.robot_id:
                self.calcRobotTopic(marker)

    def calcRobotTopic(self, marker):
        #TODO look up for transform!
        try:
            trans = self.tfBuffer.lookup_transform('map', f'marker_{self.robot_id}', rospy.Duration(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Cannot get transform to robot marker!!!")
            return
        
        x,y,z,w = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
        # rospy.loginfo(x)
        r,p,y = euler_from_quaternion([x,y,z,w])
        q = quaternion_from_euler(0,0,y)

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        pose.pose.pose.position = Point(trans.transform.translation.x, trans.transform.translation.y, 0)
        pose.pose.covariance = self.pose_covariance
        self.robot_pose_pub.publish(pose)

if __name__ == '__main__':
    try:
        mp = MapPublisher()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass