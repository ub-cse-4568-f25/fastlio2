cmake_minimum_required(VERSION 3.4)
project(drone_flight)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  mavros
  mavros_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#### main
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries( ${PROJECT_NAME}_node ${catkin_LIBRARIES})