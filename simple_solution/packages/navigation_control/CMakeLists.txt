cmake_minimum_required(VERSION 3.0.2)
project(navigation_control)

## default to C++11
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  base_local_planner
  costmap_2d
  geometry_msgs
  move_base_msgs
  roscpp
  visualization_msgs
)
find_package(Protobuf REQUIRED)

set(IGN_MSGS_VER 4)
find_package(ignition-msgs${IGN_MSGS_VER} REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
)

# Create a static library with the Protobuf messages used internally.
set(protobuf_lib_name NavProtobuf)
add_subdirectory(src/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)
add_library(${protobuf_lib_name} SHARED ${PROTO_SOURCES})
add_dependencies(${protobuf_lib_name} protobuf_compilation)
set_target_properties(${protobuf_lib_name}
  PROPERTIES POSITION_INDEPENDENT_CODE ON COMPILE_FLAGS "-Wno-unused-parameter" )
target_link_libraries(${protobuf_lib_name}
  PUBLIC
  ${PROTOBUF_LIBRARIES}
)
install(TARGETS ${protobuf_lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/protobuf/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/protobuf
  FILES_MATCHING PATTERN "*.h")

add_library(${PROJECT_NAME}_lib
  src/controller.cc
  src/graph.cc
  src/motion_controller.cc
  src/frontier_finder.cc
  src/costmap_client.cc
  src/serializer.cc
)
target_link_libraries(${PROJECT_NAME}_lib
  ${protobuf_lib_name}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ${catkin_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  # must use global_include instead of package_include because
  # subfolders under include don't match the package name
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
