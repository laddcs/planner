cmake_minimum_required(VERSION 3.0.2)
project(commander)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  mavros
  mavros_msgs
  geometry_msgs
  std_msgs
  planner_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES commander
  CATKIN_DEPENDS dynamic_reconfigure mavros mavros_msgs geometry_msgs roscpp std_msgs planner_msgs tf
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/commander.cpp
)

add_dependencies(commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(commander_node
  src/commander_node.cpp
)

add_dependencies(commander_node commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(commander_node commander ${PROJECT_NAME} ${catkin_LIBRARIES})
