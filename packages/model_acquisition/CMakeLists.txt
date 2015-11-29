cmake_minimum_required(VERSION 2.8.3)
project(model_acquisition)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(actionlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  baxter_core_msgs
  angles
  baxter_traj_streamer
  pcl_ros
  pcl_conversions
  tf
  tf2
  message_generation
  cwru_pcl_utils
)

link_directories(${PCL_LIBRARY_DIRS})

add_service_files(
  FILES
  scan_pose.srv
  acquire.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES model_acquisition
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs baxter_core_msgs trajectory_msgs cwru_joint_space_planner baxter_kinematics cwru_srv actionlib_msgs cwru_pcl_utils
  DEPENDS eigen system_lib actionlib
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

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Declare a C++ executable

add_executable(model_acquisition src/model_acquisition.cpp
  src/baxter_interface.cpp
  src/kinect2_interface.cpp)

add_dependencies(model_acquisition baxter_core_msgs baxter_traj_streamer)

target_link_libraries(model_acquisition
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(planar_pointcloud src/planar_pointcloud.cpp)

add_dependencies(planar_pointcloud cwru_pcl_utils)

target_link_libraries(planar_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)