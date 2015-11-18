cmake_minimum_required(VERSION 2.8.3)
project(pcd_watcher)

find_package(   catkin REQUIRED COMPONENTS
                message_generation
                roslint
                roscpp
                actionlib )

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

add_library(inotify-cxx src/inotify-cxx.cpp)

add_executable(pcd_watcher_server src/pcd_watcher_server.cpp)
add_executable(pcd_watcher_client src/pcd_watcher_client.cpp)

target_link_libraries(pcd_watcher_server ${catkin_LIBRARIES})
target_link_libraries(pcd_watcher_client inotify-cxx ${catkin_LIBRARIES})

roslint_cpp()
