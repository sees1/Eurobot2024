#!/usr/bin/env python3
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
        rospy.init_node('static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()

        #Map
        # tf_map = geometry_msgs.msg.TransformStamped()

        # tf_map.header.stamp = rospy.Time.now()
        # tf_map.header.frame_id = "world"
        # tf_map.child_frame_id = "map"

        # tf_map.transform.translation.x = 0
        # tf_map.transform.translation.y = 0
        # tf_map.transform.translation.z = 0

        # tf_map.transform.rotation.x = 0
        # tf_map.transform.rotation.y = 0
        # tf_map.transform.rotation.z = 0
        # tf_map.transform.rotation.w = 1

        # broadcaster.sendTransform(tf_map)
        
        #Laser
        tf_laser = geometry_msgs.msg.TransformStamped()

        tf_laser.header.stamp = rospy.Time.now()
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"

        tf_laser.transform.translation.x = 0
        tf_laser.transform.translation.y = 0
        tf_laser.transform.translation.z = 0.6

        tf_laser.transform.rotation.x = 0
        tf_laser.transform.rotation.y = 0
        tf_laser.transform.rotation.z = 0
        tf_laser.transform.rotation.w = 1

        broadcaster.sendTransform(tf_laser)

        rospy.spin()