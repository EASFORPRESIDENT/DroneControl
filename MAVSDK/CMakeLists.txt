cmake_minimum_required(VERSION 3.10)
project(MAVSDKTest)

# Assuming MAVSDK is properly configured in your environment
find_package(MAVSDK REQUIRED)

add_executable(MainTest MainTest.cpp)
target_link_libraries(MainTest MAVSDK::mavsdk)

