cmake_minimum_required(VERSION 3.0.2)
project(energybot1_strategy)


find_package(catkin REQUIRED COMPONENTS
  behavior_tree_core
  behavior_tree_leaves
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  geometry_msgs
  pcl_ros
  tf2_ros
  nav_msgs
  energybot1_lift
)

catkin_package(
  DEPENDS actionlib actionlib_msgs
  INCLUDE_DIRS include
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(energybot1_strategy_blue         src/BTloop/main_blue.cpp)
add_executable(energybot1_strategy_yellow       src/BTloop/main_yellow.cpp)
add_executable(energybot1_strategy_gomo         src/BTloop/main_gomo.cpp)
add_executable(energybot1_strategy_ne_robot_yel src/BTloop/main_ne_robot_yel.cpp)
add_executable(energybot1_strategy_reset_blue   src/BTloop/main_reset_blue.cpp)
add_executable(energybot1_strategy_reset_yellow src/BTloop/main_reset_yellow.cpp)

add_executable(button_check      src/node/button_check.cpp)
add_executable(push_plan         src/node/push_plan.cpp)
add_executable(safety_control    src/node/safety_control.cpp)
add_executable(gripper_move      src/node/gripper_move.cpp)
add_executable(timer_stop        src/node/timer_stop.cpp)

add_dependencies(energybot1_strategy_blue         ${energybot1_strategy_blue_EXPORTED_TARGETS}         ${catkin_EXPORTED_TARGETS})
add_dependencies(energybot1_strategy_yellow       ${energybot1_strategy_yellow_EXPORTED_TARGETS}       ${catkin_EXPORTED_TARGETS})
add_dependencies(energybot1_strategy_gomo         ${energybot1_strategy_gomo_EXPORTED_TARGETS}         ${catkin_EXPORTED_TARGETS})
add_dependencies(energybot1_strategy_ne_robot_yel ${energybot1_strategy_ne_robot_yel_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(energybot1_strategy_reset_blue   ${energybot1_strategy_reset_blue_EXPORTED_TARGETS}   ${catkin_EXPORTED_TARGETS})
add_dependencies(energybot1_strategy_reset_yellow ${energybot1_strategy_reset_yellow_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(button_check      ${button_check_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(push_plan         ${push_plan_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(safety_control    ${safety_control_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gripper_move      ${gripper_move_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(timer_stop        ${timer_stop_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(energybot1_strategy_blue
  ${catkin_LIBRARIES}
)
target_link_libraries(energybot1_strategy_yellow
  ${catkin_LIBRARIES}
)
target_link_libraries(energybot1_strategy_gomo
  ${catkin_LIBRARIES}
)
target_link_libraries(energybot1_strategy_ne_robot_yel
  ${catkin_LIBRARIES}
)
target_link_libraries(energybot1_strategy_reset_blue
  ${catkin_LIBRARIES}
)
target_link_libraries(energybot1_strategy_reset_yellow
  ${catkin_LIBRARIES}
)


target_link_libraries(button_check
  ${catkin_LIBRARIES}
)
target_link_libraries(push_plan
  ${catkin_LIBRARIES}
)
target_link_libraries(safety_control
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_move
  ${catkin_LIBRARIES}
)
target_link_libraries(timer_stop
  ${catkin_LIBRARIES}
)