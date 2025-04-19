cmake_minimum_required(VERSION 3.0.2)
project(minimumsnap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(YAML-CPP REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  visual_utils
  tf
  interactive_markers
  visualization_msgs
)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  visual_utils tf message_runtime
  interactive_markers visualization_msgs
  LIBRARIES minimumsnap
)


include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


# add_executable(MiniTest 
# 	src/Test_Qt.cpp
# )

add_library(minimumsnap 
	src/Minimumsnap.cpp
)

target_link_libraries(minimumsnap
   ${catkin_LIBRARIES}
   yaml-cpp
)

# target_link_libraries(MiniTest
#    ${catkin_LIBRARIES}
#    minimumsnap
# )
