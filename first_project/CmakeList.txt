cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp tf nav_msgs geometry_msgs angles)

catkin_package()

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(odometer src/odometer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odometer ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(gps_odometer src/gps_odometer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gps_odometer ${catkin_LIBRARIES})

