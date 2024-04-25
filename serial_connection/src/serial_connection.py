#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Bool
import serial
class Serial_connection():

    def __init__(self):
        self._robot = Vector3Stamped()      # стандартное сообщение для отправки текущих скоростей 
        self._statFlag = Bool()
        self._robot.header.frame_id = "Base velocity"
        self._robot_pub = rospy.Publisher('/robot_vel', Vector3Stamped, queue_size=1)
        self._getData = rospy.Subscriber("/cmd_vel", Twist, self.send)
        self._startFlag = rospy.Publisher('/startFlag',Bool,queue_size=1)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1.0)
        #self.liftSer = serial.Serial(rospy.get_param("Lift/port"), 9600, timeout = 1.0)
    
    def send(self,data):
        x = data.linear.x
        y = data.linear.y
        w = data.angular.z
        String  = "m;v:" + str(x) + ',' + str(y) + ',' + str(w) + '\n'
        #rospy.loginfo("Current data: {} ".format(String))
        self.ser.write(String.encode())
        
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
        
        self.pub(b)

        # Для проверки приходящего массива
        

    def pub(self, _list):
        iterator = 0
        # rospy.loginfo("Incoming message: {}".format(_list))
        self._robot.header.seq = iterator
        self._robot.header.stamp.nsecs = rospy.get_rostime().nsecs
        self._robot.vector.x = float(_list[0]) # Velocity of first motor
        self._robot.vector.y = float(_list[1]) # Velocity of second motor
        self._robot.vector.z = float(_list[2]) # Velocity of third motor
        flag = int(_list[3])
        if flag == 0:
            self._statFlag.data = True
        else:
            self._statFlag.data = False
        self._robot_pub.publish(self._robot)
        self._startFlag.publish(self._statFlag)


if __name__ == '__main__':
    rospy.init_node("serial_connection",anonymous=True)
    interface = Serial_connection()

    # Вызов функции для считывания Serial с частотой 20Hz
    rospy.Timer(rospy.Duration(1.0/20.0), interface.serial_read)

    rospy.spin()