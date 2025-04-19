cmake_minimum_required(VERSION 3.0.2)
project(odom_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  serial
  xv_sdk
  message_generation
)

add_message_files(
  FILES
  OdomStamp.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS xv_sdk message_runtime std_msgs geometry_msgs nav_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(odom_fusion
    src/odom_fusion.cpp
)
add_executable(odom_fusion_node
  src/odom_fusion_node.cpp
)

add_executable(TF_Test
  src/TF_Test.cpp
)

target_link_libraries(odom_fusion_node
  ${catkin_LIBRARIES}
  odom_fusion
)

target_link_libraries(TF_Test
  ${catkin_LIBRARIES}
  odom_fusion
)
