cmake_minimum_required(VERSION 2.8.3)
project(model_hmi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  model_acquisition
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES model_hmi
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(scan_hmi src/scan_hmi.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(scan_hmi model_acquisition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(scan_hmi
  ${catkin_LIBRARIES}
)
