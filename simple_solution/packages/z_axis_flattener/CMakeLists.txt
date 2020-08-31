cmake_minimum_required(VERSION 2.8.3)
project(z_axis_flattener)

## default to C++11
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    tf2
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(z_axis_flattener
  src/z_axis_flattener_node.cc
  src/z_axis_flattener.cc
)

target_link_libraries(
  z_axis_flattener
  ${catkin_LIBRARIES}
)

install(TARGETS z_axis_flattener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
