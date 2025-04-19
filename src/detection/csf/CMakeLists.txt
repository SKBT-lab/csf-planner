cmake_minimum_required(VERSION 3.0.2)
project(csf)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(YAML-CPP REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  odom_fusion
  minimumsnap
  pcl_conversions
  pcl_ros
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Cloth.msg
  Waypoints.msg
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs odom_fusion minimumsnap pcl_conversions pcl_ros sensor_msgs message_runtime
  LIBRARIES CSF
)


include_directories(
  SYSTEM  
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(CSF
  src/CSF.cpp)
  
add_executable(csf_node
  src/csf_node.cpp)


target_link_libraries(CSF 
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  yaml-cpp
  )

target_link_libraries(csf_node 
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  CSF
  )

