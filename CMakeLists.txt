cmake_minimum_required(VERSION 3.0.2)
project(offboard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
  LIBRARIES offboard
CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_runtime
)

include_directories(
  #include
 ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_control   src/offboard_control.cpp)


target_link_libraries(
  offboard_control
    ${catkin_LIBRARIES}
)

