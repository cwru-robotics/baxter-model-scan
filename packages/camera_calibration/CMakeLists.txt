cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
            COMPONENTS
            roscpp
            model_acquisition
            model_processing)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL 1.7)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    CATKIN_DEPENDS
        roscpp
        model_acquisition
    DEPENDS
        cmake_modules
        Eigen3
        Ceres
        PCL 1.7)

add_library(calibration_pcl_utils src/calibration_pcl_utils.cpp)

add_executable(calibration_solver src/calibration_solver.cpp)
add_executable(calibrate src/calibrate.cpp)

target_link_libraries(calibration_solver ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN_LIBRARIES})
target_link_libraries(calibrate calibration_pcl_utils ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
target_link_libraries(calibration_pcl_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN_LIBRARIES})
