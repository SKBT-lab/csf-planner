cmake_minimum_required(VERSION 3.0.2)
project(visual_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  INCLUDE_DIRS include
  LIBRARIES visual_utils
#  CATKIN_DEPENDS nav_msgs quadrotor_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})

add_library(visual_utils
  src/planning_visualization.cpp) 


target_link_libraries(visual_utils
  ${catkin_LIBRARIES} 
  )  
