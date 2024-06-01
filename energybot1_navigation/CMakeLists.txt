cmake_minimum_required(VERSION 3.0.2)
project(energybot1_navigation)


find_package(catkin REQUIRED COMPONENTS
  rospy
  usb_cam
  aruco_opencv_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco_robot
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/map_publisher.py
  src/robot_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)