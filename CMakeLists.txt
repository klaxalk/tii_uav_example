cmake_minimum_required(VERSION 3.1.2)
project(tii_uav_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  nav_msgs
  mrs_lib
  mrs_msgs
  )

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES TiiUavExample
  CATKIN_DEPENDS roscpp nav_msgs mrs_lib mrs_msgs
  )

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  )

# The following 3 blocks define the "launch unit"

add_library(TiiUavExample
  src/tii_uav_example.cpp)

add_dependencies(TiiUavExample
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(TiiUavExample
  ${catkin_LIBRARIES}
  )
