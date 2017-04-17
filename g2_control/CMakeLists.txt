cmake_minimum_required(VERSION 2.8.3)
project(g2_control)
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  tf
)

find_package(PCL 1.7 REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tf
  DEPENDS PCL
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(g2_control src/g2_control.cpp)
target_link_libraries(g2_control ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GAZEBO_LIBRARIES} )

add_executable(g2_control_node src/g2_control_node.cpp)
target_link_libraries(g2_control_node g2_control )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
