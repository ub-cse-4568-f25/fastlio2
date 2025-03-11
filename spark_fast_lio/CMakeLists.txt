cmake_minimum_required(VERSION 3.4)
project(spark_fast_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")
set(CMAKE_CXX_FLAGS "-O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -fexceptions")

message(STATUS "Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message(STATUS "core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message(STATUS "core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
set(PCL_FIND_QUIETLY ON)
find_package(PCL 1.8 REQUIRED)

message(STATUS Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  third_party/ikd-Tree
  third_party/IKFoM_toolkit)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

#### for ikd-Tree
add_library(ikd_tree third_party/ikd-Tree/ikd_Tree.cpp)

#### main
add_executable(spark_lio_mapping src/lio_mapping.cpp src/preprocess.cpp)
target_link_libraries(spark_lio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ikd_tree)
# NOTE(nathan) ideally we would fix unitialized variables, but the lidar preprocessing is a mess
target_compile_options(spark_lio_mapping PRIVATE -Wall -Wno-maybe-uninitialized)

find_package(livox_ros_driver QUIET)
if (livox_ros_driver_FOUND)
  target_link_libraries(spark_lio_mapping ${livox_ros_driver_LIBRARIES})
  target_include_directories(spark_lio_mapping ${livox_ros_driver_INCLUDE_DIRS})
  add_definitions(-DLIVOX_ROS_DRIVER_FOUND=${livox_ros_driver_FOUND})
else()
  message(STATUS "Missing livox driver! AVIA Lidar will be disabled!")
endif()
