cmake_minimum_required(VERSION 3.0.2)
project(traj_record)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  odom_fusion
)

# 寻找 Eigen 库
find_package(Eigen3 REQUIRED)

# 查找OpenCV库
find_package(OpenCV REQUIRED)


set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES odometry_processor
 CATKIN_DEPENDS nav_msgs roscpp std_msgs geometry_msgs odom_fusion
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}  # 包含 Eigen 的头文件路径
  ${OpenCV_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(traj_record_node src/record.cpp)

# 链接库
target_link_libraries(traj_record_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
