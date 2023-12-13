cmake_minimum_required(VERSION 3.0.2)
project(vgraph_planner)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(catkin REQUIRED COMPONENTS rospy)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS rospy)

# Install Python executables
install(PROGRAMS scripts/vgraph_planner.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
