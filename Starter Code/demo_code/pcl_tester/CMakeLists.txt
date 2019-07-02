cmake_minimum_required(VERSION 2.8.3)
project(pcl_tester)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf2_ros
)

find_package(gazebo REQUIRED)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS roscpp gazebo_ros
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
