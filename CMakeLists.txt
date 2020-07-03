cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(social_worlds)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  gazebo_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)
find_package(gazebo REQUIRED)

################################################################################

#add_action_files(
#  DIRECTORY action
#  FILES
#  Actor.action
#)

add_service_files(
  FILES
  Start.srv
  ActorTrajectory.srv
  Regions.srv
)

#add_message_files(
#  FILES
#  ActorControl.msg
#)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_runtime
)

################################################################################

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${actionlib_INCLUDE_DIRS})
include_directories(${message_generation_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${actionlib_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


################################################################################

# Build libraries

add_library(actor_control SHARED plugins/actor_control.cc)
target_link_libraries(actor_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_dependencies(actor_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(proxemics SHARED plugins/proxemics.cc)
target_link_libraries(proxemics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_dependencies(proxemics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(sliding_door_control SHARED plugins/sliding_door_control.cc)
target_link_libraries(sliding_door_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_dependencies(sliding_door_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(receptionist SHARED plugins/task_control__receptionist.cc)
target_link_libraries(receptionist ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_dependencies(receptionist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(regions SHARED plugins/regions.cc)
target_link_libraries(regions ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_dependencies(regions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Build executable

add_executable(check_region_node src/check_region_node.cpp)
target_link_libraries(check_region_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(real_time_factor_publisher_node src/real_time_factor_publisher_node.cpp)
target_link_libraries(real_time_factor_publisher_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
