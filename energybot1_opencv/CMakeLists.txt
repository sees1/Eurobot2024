cmake_minimum_required(VERSION 3.0.2)
project(energybot1_opencv)


find_package(catkin REQUIRED COMPONENTS
  aruco_opencv
  aruco_opencv_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES setup
#  CATKIN_DEPENDS tf tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
