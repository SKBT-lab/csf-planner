cmake_minimum_required(VERSION 3.0.2)
project(usb_camera)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include

)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS} 
)


add_executable(apriltag_node
  src/apriltag.cpp 
  )

target_link_libraries(apriltag_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )