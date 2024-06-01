#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64
import time

start_time = None

def callback(data):
    global start_time
    if data.data == False and start_time is None:
        start_time = time.time()

def timer_publisher():
    rospy.init_node('time_publisher', anonymous=True)
    pub = rospy.Publisher('/timer_check', Float64, queue_size=10)
    rospy.Subscriber('/start_engine', Bool, callback)

    while not rospy.is_shutdown():
        if start_time is not None:
            elapsed_time = time.time() - start_time
            pub.publish(Float64(elapsed_time))  # Modified this line
    rospy.spin()

if __name__ == '__main__':
    try:
        timer_publisher()
    except rospy.ROSInterruptException:
        pass

