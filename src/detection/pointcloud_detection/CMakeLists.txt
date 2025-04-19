cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_detection)



SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  livox_ros_driver2
  odom_fusion
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_detection
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs odom_fusion
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pointcloud_detection
  src/CSF_Lidar.cpp)
  
add_executable(CSF_Lidar_Test
  src/Testv1.cpp)

add_executable(save_pointcloud
src/save_pointcloud.cpp)


target_link_libraries(pointcloud_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  ${OpenCV_LIBRARIES}
  )

target_link_libraries(CSF_Lidar_Test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  pointcloud_detection
  ${OpenCV_LIBRARIES}
  
  )

  target_link_libraries(save_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )


