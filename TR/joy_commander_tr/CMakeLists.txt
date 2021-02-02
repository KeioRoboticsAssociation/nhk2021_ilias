cmake_minimum_required(VERSION 2.8.3)
project(joy_commander_tr)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_commander_tr
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/joy_commander_tr
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_commander_tr src/joy_commander_tr.cpp)
target_link_libraries(joy_commander_tr ${catkin_LIBRARIES})