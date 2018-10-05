cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files (
  FILES
  Uwbdis.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kalman_filter
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(filter
  src/KalmanFilter.cpp
  src/KFRos.cpp
)
add_executable(kf_main 
  src/kf_main.cpp
  src/KalmanFilter.cpp
  src/KFRos.cpp
)
target_link_libraries(kf_main
  filter
  ${catkin_LIBRARIES}
)

