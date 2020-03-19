cmake_minimum_required(VERSION 2.8.3)

project(social_worlds)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_library(auto_sliding_door SHARED plugins/auto_sliding_door.cc)
target_link_libraries(auto_sliding_door ${GAZEBO_LIBRARIES})
