cmake_minimum_required(VERSION 3.0.2)
project(controller)

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
  LIBRARIES controller
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
  src/controller.cpp
)

add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(controller_node
  src/controller_node.cpp
)

add_dependencies(controller_node controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(controller_node controller ${PROJECT_NAME} ${catkin_LIBRARIES})
