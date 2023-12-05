cmake_minimum_required(VERSION 3.0.2)
project(template)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(catkin REQUIRED COMPONENTS rospy)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS rospy)

# Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Install Python executables
install(PROGRAMS scripts/py_template.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
