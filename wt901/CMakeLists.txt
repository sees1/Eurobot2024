cmake_minimum_required(VERSION 3.0.2)
project(wt901)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wt901
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  src/wt901_node.py
  src/wt901_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
