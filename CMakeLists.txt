cmake_minimum_required(VERSION 3.0.2)
project(template)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy)

# Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Create Cpp executable
add_executable(cpp_template src/cpp_template.cpp)
target_link_libraries(cpp_template ${catkin_LIBRARIES})

# Install Cpp executable
install(TARGETS cpp_template RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install Python executables
install(PROGRAMS scripts/py_template.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
