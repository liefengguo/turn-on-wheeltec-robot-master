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
  a22_data.msg
  VRTK.msg
  Speed.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   geometry_msgs
)
find_package(Eigen3 REQUIRED)

# ulimit -c unlimited
# ssh firefly@192.168.1.76
add_compile_options(-g -std=c++11)    #-g -> core dumped

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
)
catkin_package(
        DEPENDS EIGEN3
)
# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS})# include_directories(include/${PROJECT_NAME})

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
add_dependencies(udp_log ${PROJECT_NAME}_generate_messages_cpp)

add_executable(record_points src/record_points/record_points.cpp src/tools/gnss_coordinate_convert.cpp)
target_link_libraries(record_points ${catkin_LIBRARIES})

add_executable(purepursuit src/purepursuit/pure_pursuit_node.cpp src/purepursuit/pure_pursuit.cpp src/tools/gnss_coordinate_convert.cpp)
target_link_libraries(purepursuit ${catkin_LIBRARIES})

add_executable(referenceline_points src/referenceline_points/referenceline_points.cpp)
target_link_libraries(referenceline_points ${catkin_LIBRARIES})


add_executable(six_a22 src/serial_connect/six_a22_node.cpp src/serial_connect/add_CRC.cpp)
target_link_libraries(six_a22
  ${catkin_LIBRARIES}
)
add_dependencies(six_a22 ${PROJECT_NAME}_generate_messages_cpp)

add_executable(serial_write src/serial_connect/serial_write.cpp src/serial_connect/add_CRC.cpp)
target_link_libraries(serial_write
  ${catkin_LIBRARIES}
)
add_dependencies(serial_write ${PROJECT_NAME}_generate_messages_cpp)


add_executable(six_decition_radar_node src/serial_connect/six_decition_radar_node 
      src/serial_connect/six_decition_radar.cpp src/serial_connect/adaptive_filter.cpp)
target_link_libraries(six_decition_radar_node
  ${catkin_LIBRARIES}
)

add_executable(radar src/serial_connect/radar.cpp) 
add_dependencies(radar ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(radar
  ${catkin_LIBRARIES}
)
add_executable(RadarController src/radar_controller/RadarController.cpp src/radar_controller/RadarController_node.cpp
        src/bizhang/BezierCurve.cpp src/bizhang/DWA.cpp src/radar_controller/VRFKReader.cpp src/serial_connect/six_decition_radar.cpp src/serial_connect/adaptive_filter.cpp)
add_dependencies(RadarController ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RadarController
  ${catkin_LIBRARIES}
)


add_executable(main_node src/main_node.cpp src/purepursuit/pure_pursuit.cpp   src/radar_controller/RadarController.cpp 
 src/tools/gnss_coordinate_convert.cpp src/bizhang/BezierCurve.cpp src/bizhang/DWA.cpp
src/radar_controller/VRFKReader.cpp src/serial_connect/six_decition_radar.cpp src/serial_connect/adaptive_filter.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES})
add_dependencies(main_node ${EXTRA_LIBS} ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(udp_log src/udp_log.cpp src/udp_car_info.cpp)
# target_link_libraries(udp_log ${catkin_LIBRARIES})

# add_executable(keyboard_controller src/keyboard_controller.cpp)
# target_link_libraries(keyboard_controller ${catkin_LIBRARIES})          