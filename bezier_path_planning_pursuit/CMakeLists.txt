cmake_minimum_required(VERSION 3.0.2)
project(bezier_path_planning_pursuit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  message_generation
  actionlib
  actionlib_msgs
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES PursuitPath.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES bezier_path_planning_pursuit
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs message_runtime
  DEPENDS Boost
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory(src/Matrix)
add_subdirectory(src/path_planning)

add_executable(bezier_path_planning_pursuit src/main.cpp)
target_link_libraries(bezier_path_planning_pursuit ${catkin_LIBRARIES} path_planning Matrix)
add_dependencies(bezier_path_planning_pursuit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})