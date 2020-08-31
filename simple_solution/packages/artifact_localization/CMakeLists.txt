cmake_minimum_required(VERSION 2.8.3)
project(artifact_localization)

## default to C++11
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  darknet_ros_msgs
  tf2
  tf2_ros
  subt_msgs
  geometry_msgs
  tf2_geometry_msgs
  subt_ign
  std_msgs
  artifact_localization_msgs
  visualization_msgs
)

set(IGN_MATH_VER 6)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)
set(IGN_MSGS_VER 4)
find_package(ignition-msgs${IGN_MSGS_VER} REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  locate_artifacts
  src/findAndReportArtifacts.cc
  src/Artifact.cc
  src/ArtifactLocalizer.cc
  src/ArtifactDatabase.cc
  src/ArtifactOriginFinder.cc
  src/ArtifactReporter.cc
  src/Detection.cc
  src/TransformGenerator.cc
  src/ExternalPointRadar.cc
)

target_link_libraries(
  locate_artifacts
  ${catkin_LIBRARIES}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)

install(TARGETS locate_artifacts
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
