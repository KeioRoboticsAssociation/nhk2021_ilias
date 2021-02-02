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
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bezier_path_planning_pursuit
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/Matrix)
add_subdirectory(src/path_planning)

add_executable(path_planning_pursuit src/main.cpp)
target_link_libraries(path_planning_pursuit ${catkin_LIBRARIES} path_planning Matrix)
add_dependencies(path_planning_pursuit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})