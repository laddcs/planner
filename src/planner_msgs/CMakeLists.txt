cmake_minimum_required(VERSION 3.0.2)
project(planner_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  CommandState.msg
)

generate_messages(DEPENDENCIES geometry_msgs mavros_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
