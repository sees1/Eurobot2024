import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
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
        rospy.init_node('wt901_node')
        self.device = rospy.get_param("wt901/port")
        self.ser = serial.Serial(self.device, 115200, timeout=None)
        rospy.loginfo(f"Successfully connected to {self.device}. Waiting for {timeout} sec")
        rospy.sleep(timeout)
        rospy.loginfo(f"Accelerometer Calibration. Waiting for 5 sec...")
        send(self.ser, 'FF AA 01 01 00', 5) #Accelerometer Calibration, 5sec for calibration
        send(self.ser, 'FF AA 01 00 00') #Quit Calibration
        send(self.ser, 'FF AA 03 00 00') #Save
        rospy.loginfo(f"Calibration finished")
        send(self.ser, 'FF AA 03 08 00') #Set to 50 HZ (It is max)
        rospy.loginfo(f"Rate has been set up for 50 HZ")
        rospy.loginfo(f"Publishing to /imu/data")
        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=1)

    def update(self):
        imu = Imu()

        imu.angular_velocity_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
        imu.linear_acceleration_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
        imu.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

        while True:
            send(self.ser, 'FF AA 27 51 00', 0)
            data = self.ser.read(2)
            if data[1] == bytes.fromhex('61')[0]:
                data = self.ser.read(18)

                if DEBUG:
                    rospy.loginfo(f"Received {len(data)} bytes")
                    rospy.loginfo(''.join('{:02X} '.format(a) for a in data))

                axL,axH,ayL,ayH,azL,azH,wxL,wxH,wyL,wyH,wzL,wzH,RollL,RollH,PitchL,PitchH,YawL,YawH = data[:]

                ax=int.from_bytes([axH, axL], byteorder="big", signed=True)/32768*16*g
                ay=int.from_bytes([ayH, ayL], byteorder="big", signed=True)/32768*16*g
                az=int.from_bytes([azH, azL], byteorder="big", signed=True)/32768*16*g

                wx=int.from_bytes([wxH, wxL], byteorder="big", signed=True)/32768*2000/180*PI
                wy=int.from_bytes([wyH, wyL], byteorder="big", signed=True)/32768*2000/180*PI
                wz=int.from_bytes([wzH, wzL], byteorder="big", signed=True)/32768*2000/180*PI

                Roll=int.from_bytes([RollH, RollL], byteorder="big", signed=True)/32768*PI
                Pitch=int.from_bytes([PitchH, PitchL], byteorder="big", signed=True)/32768*PI
                Yaw=int.from_bytes([YawH, YawL], byteorder="big", signed=True)/32768*PI

                if DEBUG:
                    rospy.loginfo("Acceleration is: {:.2f} {:.2f} {:.2f}".format(ax, ay, az))
                    rospy.loginfo("Angular is: {:.2f} {:.2f} {:.2f}".format(wx, wy, wz))
                    rospy.loginfo("Angle is: {:.2f} {:.2f} {:.2f}".format(Roll, Pitch, Yaw))

                imu.angular_velocity = Vector3(wx, wy, wz)

                imu.angular_velocity_covariance = [
                    0.0663,0,0,
                    0, 0.1453, 0,
                    0, 0, 0.0378
                ]

                imu.linear_acceleration = Vector3(ax, ay, az)
                imu.linear_acceleration_covariance = [
                    0.0364, 0, 0,
                    0, 0.0048, 0,
                    0, 0, 0.0796
                ]
                
            elif data[1] == bytes.fromhex('71')[0]:
                data = self.ser.read(10)

                if DEBUG:
                    rospy.loginfo(f"Received {len(data)} bytes")
                    rospy.loginfo(''.join('{:02X} '.format(a) for a in data))
                    
                Q0L, Q0H, Q1L, Q1H, Q2L, Q2H, Q3L, Q3H = data[2:]

                Q0=int.from_bytes([Q0H, Q0L], byteorder="big", signed=True)/32768
                Q1=int.from_bytes([Q1H, Q1L], byteorder="big", signed=True)/32768
                Q2=int.from_bytes([Q2H, Q2L], byteorder="big", signed=True)/32768
                Q3=int.from_bytes([Q3H, Q3L], byteorder="big", signed=True)/32768

                if DEBUG:
                    rospy.loginfo("Quaternion is: {:.2f} {:.2f} {:.2f} {:.2f}".format(Q0, Q1, Q2, Q3))

                imu.orientation = Quaternion(Q0, Q1, Q2, Q3)
                imu.orientation_covariance = [
                    0.0479, 0, 0,
                    0, 0.0207, 0,
                    0, 0, 0.0041
                ]
            else:
                continue

            if imu.angular_velocity_covariance[0] == -1 or imu.orientation_covariance[0] == -1:
                continue

            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "wt901"
            
            self.pub_imu.publish(imu)
            return


if __name__ == '__main__':
    node = wt901()
    try:
        while not rospy.is_shutdown():
            node.update()
    except rospy.ROSInterruptException:
        pass
    