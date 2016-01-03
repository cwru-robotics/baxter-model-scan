cmake_minimum_required(VERSION 2.8.3)
project(model_recognition)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
  DEPENDS system_lib eigen
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_NAME}/include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_executable(cluster_models src/cluster_models.cpp
                              src/model_pattern.cpp
                              src/model_pattern_set.cpp
                              src/model_cluster.cpp)

add_dependencies(cluster_models ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cluster_models
  ${catkin_LIBRARIES}
)