cmake_minimum_required(VERSION 3.0.2)
project(rtk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  mavros
)

find_package(Ceres REQUIRED)
find_package(Eigen3)

catkin_package(

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)





add_executable(location_node src/location_node.cpp
                             src/locationFuntions_def.cpp)


target_link_libraries(location_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)


