cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(catkin REQUIRED
            COMPONENTS
            roscpp)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
    CATKIN_DEPENDS
        roscpp
    DEPENDS
        cmake_modules
        Eigen3
        Ceres)

add_executable(calibration_solver src/calibration_solver.cpp)

target_link_libraries(calibration_solver ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN_LIBRARIES})
