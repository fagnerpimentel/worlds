cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(social_worlds)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

################################################################################

# Build our plugin
add_library(actor_control SHARED plugins/actor_control.cc)
target_link_libraries(actor_control ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
