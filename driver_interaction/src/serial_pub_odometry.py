#!/usr/bin/env python3

from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial

class Odom():
    def __init__(self):
        self.iterator = 0

        self.x  = 0
        self.y  = 0
        self.th = 0

        self.curentVx = 0
        self.curentVy = 0
        self.curentW  = 0

        self.current_time = rospy.Time.now()
        self.last_time    = rospy.Time.now()

        self.odom_pub   = rospy.Publisher("/odom", Odometry, queue_size = 1)
        self.start_pub  = rospy.Publisher("/start_engine", Bool, queue_size = 1)
        self.ser        = serial.Serial('/dev/ttyACM0', 28800, timeout = 1.0)

        self.start_flag = Bool()

        self.odom_broadcaster = tf.TransformBroadcaster()

        self.pose_covariance = rospy.get_param("/pose_covariance")
        self.twist_covariance = rospy.get_param("/twist_covariance")
        
    def sendMsg(self):
        odom = Odometry()

        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
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
        self.sendMsg()

    def serial_read(self, event=None):
        b = self.ser.readline().decode(('utf-8'))
        # rospy.loginfo("Incoming message11: {}".format(b))
        b = b.replace('b', '')
        b = b.replace("'", '')
        b = b.replace("n", '')
        b = b.replace("r", '')
        b = b.replace("\\", '')
        b = b.replace(":", ',')
        b = b.replace(" ", '')
        b = b.split(',')
        b = [val for val in b if val != '']

        if(len(b) != 4):
            rospy.loginfo("String is too short: {}".format(b))
            return
        
        # rospy.loginfo("Incoming message: {}".format(_list))
        W1 = float(b[0]) # Velocity of first motor
        W2 = float(b[1]) # Velocity of second motor
        W3 = float(b[2]) # Velocity of third motor

        self.curentVx = round(4 * 1.4 * (0.0167  * W1 + 0.0000 * W2 - 0.0167 * W3), 2)
        self.curentVy = round(4 * 1.4 * (-0.0097 * W1 + 0.0193 * W2 - 0.0097 * W3), 2)
        self.curentW  = round((-0.0491 * W1 - 0.0491 * W2 - 0.0491 * W3) * 2 * pi, 2)

        self.integrate()

        flag = int(b[3])
        if flag == 0:
            self.start_flag.data = True
        else:
            self.start_flag.data = False

        self.start_pub.publish(self.start_flag)


if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    odom = Odom()
    while not rospy.is_shutdown():
        odom.serial_read()
        rospy.sleep(rospy.Duration(1.0 / 30.0))
    







