cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs nav_msgs sensor_msgs geometry_msgs angles message_generation rosbag)

## Declare the message files to be built
add_message_files(
  FILES
  secotor_times.msg  # Note: the filename is kept as "secotor_times.msg" per teacher specification
)

## Generate added messages with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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

## Declare a C++ executable
add_executable(sector_times src/sector_times.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sector_times ${catkin_LIBRARIES})

## Find catkin and required ROS components
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  std_msgs
#  geometry_msgs
#  message_generation
#  rosbag
#)



## Declare catkin package
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
#)

## Add dependencies to ensure message generation happens before compiling
add_dependencies(sector_times ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})