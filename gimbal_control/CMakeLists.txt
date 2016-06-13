cmake_minimum_required(VERSION 2.8.3)
project(gimbal_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gimbal_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS tf
)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(gimbal_control
  src/gimbal_control.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(gimbal_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(gimbal_control_node src/gimbal_control_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(gimbal_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gimbal_control_node ${catkin_LIBRARIES} gimbal_control)
