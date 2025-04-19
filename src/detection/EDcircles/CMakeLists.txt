cmake_minimum_required(VERSION 3.0.2)
project(edcircles)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS OpenCV
 LIBRARIES edcircles
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


add_library(edcircles
    src/EDCircles.cpp
    include/edcircles/NFA.cpp
    include/edcircles/EDPF.cpp
    include/edcircles/EDLines.cpp
    include/edcircles/EDColor.cpp
    include/edcircles/ED.cpp
)

target_link_libraries(edcircles
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )  



