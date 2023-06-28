cmake_minimum_required(VERSION 2.8.3)
project(turn_on_wheeltec_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)
add_message_files(
  FILES
  Speed.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
)

# ulimit -c unlimited
# ssh firefly@192.168.1.76
add_compile_options(-g -std=c++11)    #-g -> core dumped

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS})
# include_directories(include/${PROJECT_NAME})

# add_executable(wheeltec_robot_node src/wheeltec_robot.cpp src/Quaternion_Solution.cpp)
# target_link_libraries(wheeltec_robot_node ${catkin_LIBRARIES})

add_executable(udp_robot src/udp_robot.cpp)
target_link_libraries(udp_robot ${catkin_LIBRARIES})

add_executable(udp_action src/udp_action.cpp src/udp_action_test.cpp)
target_link_libraries(udp_action ${catkin_LIBRARIES})

add_executable(test_udp src/test_udp.cpp)
target_link_libraries(test_udp ${catkin_LIBRARIES})

add_executable(udp_log src/udp_log.cpp)
target_link_libraries(udp_log ${catkin_LIBRARIES})

add_executable(keyboard_controller src/keyboard_controller.cpp)
target_link_libraries(keyboard_controller ${catkin_LIBRARIES})          