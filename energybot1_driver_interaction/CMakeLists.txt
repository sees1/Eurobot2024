cmake_minimum_required(VERSION 3.0.2)
project(energybot1_driver_interaction)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES timer_check
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)