cmake_minimum_required(VERSION 3.0)
project(PosTest)

# Find ROS
find_package(ros REQUIRED)
include_directories(${ROS_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Generate executable
add_executable(PosTest src/Postest.cpp)
target_link_libraries(PosTest ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Install
install(TARGETS my_gazebo_reader DESTINATION bin)
