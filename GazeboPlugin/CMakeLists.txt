cmake_minimum_required(VERSION 3.10)
project(GazeboPlugin)

# Find packages
find_package(gazebo REQUIRED)

add_library(GazeboPlugin SHARED GazeboPlugin.cpp)
include_directories(${GAZEBO_INCLUDE_DIRS})
target_link_libraries(GazeboPlugin ${GAZEBO_LIBRARIES})