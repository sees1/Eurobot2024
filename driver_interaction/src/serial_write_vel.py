#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial
class CmdToDriver():

    def __init__(self):
        self.cmd_sub  = rospy.Subscriber("/cmd_vel", Twist, self.send_to_driver)
        self.ser      = serial.Serial('/dev/ttyACM0', 9600, timeout = 1.0)
        #self.liftSer = serial.Serial(rospy.get_param("Lift/port"), 9600, timeout = 1.0)
    
    def send_to_driver(self, data):
        x = data.linear.x
        y = data.linear.y
        w = data.angular.z
        String  = "m;v:" + str(x) + ',' + str(y) + ',' + str(w) + '\n'
        #rospy.loginfo("Current data: {} ".format(String))
        self.ser.write(String.encode())

if __name__ == '__main__':
    rospy.init_node("serial_connection",anonymous=True)
    CmdToDriver()
    rospy.spin()