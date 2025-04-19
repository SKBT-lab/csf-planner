cmake_minimum_required(VERSION 3.0.2)
project(pc_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  odom_fusion
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs pcl_ros pcl_conversions odom_fusion odom_fusion
  DEPENDS Eigen3 PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(point_cloud_mapper src/point_cloud_mapper_fromDepth.cpp)
add_executable(pcd_vis src/vis_demo.cpp)
add_executable(pcd_combine src/combine_pcd.cpp)

target_link_libraries(point_cloud_mapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(pcd_vis
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(pcd_combine
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)