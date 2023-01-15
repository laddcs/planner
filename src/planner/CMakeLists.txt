cmake_minimum_required(VERSION 3.0.2)
project(planner)

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
  LIBRARIES planner
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
  src/commander/commander.cpp
  src/controller/controller.cpp
  src/planner/planner.cpp
)

add_dependencies(planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(commander_node
  src/commander/commander_node.cpp
)
add_executable(controller_node
  src/controller/controller_node.cpp
)
add_executable(planner_node
  src/planner/planner_node.cpp
)

add_dependencies(commander_node planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_node planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(planner_node planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(commander_node planner ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(controller_node planner ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(planner_node planner ${PROJECT_NAME} ${catkin_LIBRARIES})