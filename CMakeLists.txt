cmake_minimum_required(VERSION 3.0.2)
project(vgraph_planner)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf
  tf2_ros
)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS 
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf
  tf2_ros
)

# Install Python executables
install(PROGRAMS scripts/vgraph_planner.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
