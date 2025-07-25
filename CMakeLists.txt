cmake_minimum_required(VERSION 3.0.2)
project(anymal_wbc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
)

add_message_files( FILES
  WbcReferenceMsg.msg
)

generate_messages( DEPENDENCIES
  std_msgs
)

find_package(iDynTree REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES anymal_wbc
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  lib/qpOASES/include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(lib/qpOASES)
add_subdirectory(lib/idyntree)

add_library(WholeBodyController src/WholeBodyController.cpp)
target_link_libraries(WholeBodyController PRIVATE iDynTree::idyntree-high-level qpOASES ${catkin_LIBRARIES})

add_executable(whole_body_controller_node src/WholeBodyControllerNode.cpp)
target_link_libraries(whole_body_controller_node WholeBodyController ${catkin_LIBRARIES})