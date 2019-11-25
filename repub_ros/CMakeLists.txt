cmake_minimum_required(VERSION 2.8.3)
project(repub_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)


set( hearders
  src/lcmtormc_tram_mission_t.hpp
  src/lcmtoros_dw_lane_t.hpp
  src/lcmtoros_tram_sub_path.hpp
  src/vehicle_model_viz.hpp.hpp
)

set( sources
  src/repub_ros_node.cpp
)


find_package(lcm REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${LCM_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${sources} ${headers})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   lcm
)


