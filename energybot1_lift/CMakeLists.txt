cmake_minimum_required(VERSION 3.0.2)
project(energybot1_lift)


find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib
  actionlib_msgs
  message_generation
  rospy
  std_msgs
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   Interface.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs   
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES liftAction
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)