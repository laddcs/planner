cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner
  CATKIN_DEPENDS dynamic_reconfigure roscpp std_msgs tf
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
