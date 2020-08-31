cmake_minimum_required(VERSION 2.8.3)
project(blob_detection)

## default to C++11
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  roscpp
  darknet_ros_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(
  blob_detection
  src/blob_detection.cc
  src/blob_detection_node.cc
)

target_link_libraries(
  blob_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS blob_detection
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
