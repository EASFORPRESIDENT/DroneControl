cmake_minimum_required(VERSION 3.10)
project(MAVSDKTest)

# Assuming MAVSDK is properly configured in your environment
find_package(MAVSDK REQUIRED)

# Find Python3 and NumPy
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# Include directories for Python and NumPy
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# Add matplotlib-cpp directory to include directories
include_directories(/home/user/matplotlib-cpp)  # Update this path to the actual path of matplotlib-cpp

# Add executable
add_executable(MainTest MainTest.cpp)

# Link libraries
target_link_libraries(MainTest MAVSDK::mavsdk ${Python3_LIBRARIES})


