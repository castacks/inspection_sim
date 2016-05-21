cmake_minimum_required(VERSION 2.8.3)
project(g2_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  tf
)

find_package(PCL 1.7 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tf
  DEPENDS PCL
)


add_library(g2_control src/g2_control.cpp)
target_link_libraries(g2_control ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(g2_control_node src/g2_control_node.cpp)
target_link_libraries(g2_control_node g2_control  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
