cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS}/gazebo/math) # Add this line
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(circle_motion_plugin SHARED CircleMotionPlugin.cpp)
target_link_libraries(circle_motion_plugin ${GAZEBO_LIBRARIES})