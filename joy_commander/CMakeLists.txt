cmake_minimum_required(VERSION 2.8.3)
project(joy_commander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_commander
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_commander src/joy_commander.cpp)
target_link_libraries(joy_commander ${catkin_LIBRARIES})