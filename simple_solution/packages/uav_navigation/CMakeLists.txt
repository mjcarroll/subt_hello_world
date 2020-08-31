cmake_minimum_required(VERSION 2.8.3)
project(uav_navigation)

## default to C++11
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  geometry_msgs
  sensor_msgs
  uav_nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  hover_controller
  src/hoverUav.cc
  src/FlightController.cc
)

target_link_libraries(
  hover_controller
  ${catkin_LIBRARIES}
)

install(TARGETS hover_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
  cmd_vel_filter
  src/processCmdVels.cc
  src/CmdVelFilter.cc
)

target_link_libraries(
  cmd_vel_filter
  ${catkin_LIBRARIES}
)

install(TARGETS cmd_vel_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
