import rospy
import serial

g=9.80665
PI = 3.14159
timeout=0.2

DEBUG=True

def send(ser, command : str, delay=timeout):
    ser.write(bytes.fromhex(command)) #Accelerometer Calibration
    rospy.sleep(delay)

class wt901:
    def __init__(self):
        rospy.init_node('wt901_calibration_node')
        self.device = rospy.get_param("wt901/port")
        self.ser = serial.Serial(self.device, 115200, timeout=None)
        if self.ser:
            rospy.loginfo(f"Successfully connected to {self.device}. Waiting for {timeout} sec")
        else:
            rospy.logerr("Can not connect to IMU!")
            exit(1)

        rospy.sleep(timeout)
        rospy.loginfo(f"Accelerometer Calibration. Waiting for 5 sec...")
        send(self.ser, 'FF AA 01 01 00', 5) #Accelerometer Calibration, 5sec for calibration
        send(self.ser, 'FF AA 01 00 00') #Quit Calibration
        send(self.ser, 'FF AA 03 00 00') #Save
        rospy.loginfo(f"Calibration finished")
        send(self.ser, 'FF AA 03 08 00') #Set to 50 HZ (It is max)
        rospy.loginfo(f"Hardware rate has been set up for 50 HZ")
        self.ser.close()


if __name__ == '__main__':
    node = wt901()
    rospy.loginfo(f"Calibration finished. Exiting...")

    