cmake_minimum_required(VERSION 3.0.2)
project(wheelctrl_4ws)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wheelctrl_4ws
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/converter)
add_subdirectory(src/processings)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} converter processings)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})