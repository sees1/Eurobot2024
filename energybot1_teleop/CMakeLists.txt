cmake_minimum_required(VERSION 3.0.2)
project(energybot1_teleop)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teleop
#  CATKIN_DEPENDS geometry_msgs rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)