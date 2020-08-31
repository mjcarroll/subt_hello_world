cmake_minimum_required(VERSION 2.8.3)
project(subt_solution_comms)

## C++14 is needed for std::make_unique
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  subt_ign
  std_msgs
  subt_communication_broker
)

set(IGN_COMMON_VER 3)
find_package(ignition-common${IGN_COMMON_VER} REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS subt_communication_broker
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/comms_manager.cc
  src/local_communicator.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_executable(${PROJECT_NAME}_node
  src/comms_node.cc
  src/comms_manager.cc
  src/local_communicator.cc
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
