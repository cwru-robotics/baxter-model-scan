cmake_minimum_required(VERSION 2.8.3)
project(model_processing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package (PCL 1.7 REQUIRED)

catkin_package(
	INCLUDE_DIRS
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	LIBRARIES
	model_processing
	)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(model_processing src/model_processing.cpp)
target_link_libraries(model_processing ${catkin_LIBRARIES} ${PCL_LIBRARIES})
