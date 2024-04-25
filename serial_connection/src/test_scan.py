#!/usr/bin/env python3
import rospy
import serial

from geometry_msgs.msg import Vector3Stamped

def func():
    pub = rospy.Publisher('chatter', Vector3Stamped, queue_size=0)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20)
    ser = serial.Serial('/dev/ttyACM0', 9600)
    while not rospy.is_shutdown():
        string  = ser.readline()
        rospy.loginfo(string)
        rate.sleep()

if __name__ == "__main__":
    try:
        func()
    except rospy.ROSInterruptException:
        pass
        
