#!/usr/bin/env python3

from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3Stamped,Vector3

class odom():
    def __init__(self):
        self.x=0
        self.y=0
        self.th=0

        self.curentVx=0
        self.curentVy=0
        self.curentW=0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.sub = rospy.Subscriber('/robot_vel', Vector3Stamped, self.read_vel)

        self.odom_broadcaster = tf.TransformBroadcaster()
        self.odom_quat = 0

        self.pose_covariance = rospy.get_param("/pose_covariance")
        self.twist_covariance = rospy.get_param("/twist_covariance")


    def transform(self):
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                self.odom_quat,
                self.current_time,
                "base_link",
                "odom"
                                    )
        self.sendMsg()
        
    def sendMsg(self):
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.curentVx, self.curentVy, 0), Vector3(0, 0, self.curentW))
        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance
        self.odom_pub.publish(odom)
        
    def integrate(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        delta_x = (self.curentVx * cos(self.th) - self.curentVy * sin(self.th)) * dt
        delta_y = (self.curentVx * sin(self.th) + self.curentVy * cos(self.th)) * dt
        #delta_x = self.curentVx*dt
        #delta_y = self.curentVy*dt
        delta_th = self.curentW * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.last_time = self.current_time
        #rospy.loginfo('Current x|y|th: ' + str(self.x) + '|' + str(self.y)+ '|' + str(self.th))
        self.transform()

    def read_vel(self, data):
        W1 = data.vector.x
        W2 = data.vector.y
        W3 = data.vector.z
        self.curentVx = round(4*1.4*(0.0167*W1  +  0.0000*W2  - 0.0167*W3), 2)
        self.curentVy = round(4*1.4*(-0.0097*W1 +  0.0193*W2  - 0.0097*W3),2)
        self.curentW = round((-0.0491*W1 -  0.0491*W2  - 0.0491*W3)*2*pi, 2)
        #rospy.loginfo('RealVel: '+ str(self.curentVx) +' '+ str(self.curentVy) +' '+ str(self.curentW))
        self.integrate()



if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    odom()
    rospy.spin()
    







