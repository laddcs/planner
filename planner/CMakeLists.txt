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
  src/planner.cpp
  src/hybrid_astar.cpp
)

add_dependencies(planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(planner_node
  src/planner_node.cpp
)

add_dependencies(planner_node planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(planner_node planner ${PROJECT_NAME} ${catkin_LIBRARIES})

############
# TESTING #
############
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test
      test/main.cpp
      test/hybrid_astar-test.cpp)

    if(TARGET ${PROJECT_NAME}-test) 
      target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES})
    endif()
endif()