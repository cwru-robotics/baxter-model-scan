cmake_minimum_required(VERSION 2.8.3)
project(pcd_watcher)

find_package(   catkin REQUIRED COMPONENTS
                message_generation
                roslint
                roscpp
                actionlib 
		model_processing)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

add_action_files(
    FILES
    new_pcd.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES inotify-cxx 
CATKIN_DEPENDS roscpp actionlib 
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

add_library(inotify-cxx src/inotify-cxx.cpp)

add_executable(pcd_watcher_server src/pcd_watcher_server.cpp)
add_executable(pcd_watcher_client src/pcd_watcher_client.cpp)

target_link_libraries(pcd_watcher_server model_processing ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcd_watcher_client inotify-cxx ${catkin_LIBRARIES})

roslint_cpp()
