cmake_minimum_required(VERSION 2.8.3)
project(minimal_pcl)

find_package(PCL REQUIRED) # This includes all modules
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs
  pcl_conversions pcl_ros
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs pcl_conversions pcl_ros)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(minimal_pub_sub src/minimal_pub_sub.cpp)
target_link_libraries(minimal_pub_sub ${catkin_LIBRARIES} ${PCL_LIBRARIES})
