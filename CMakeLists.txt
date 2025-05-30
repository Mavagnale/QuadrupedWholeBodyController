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
  lib/towr/towr/include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(lib/qpOASES)
add_subdirectory(lib/idyntree)
add_subdirectory(lib/ifopt)
add_subdirectory(lib/towr/towr)

add_library(WholeBodyController src/WholeBodyController.cpp)
target_link_libraries(WholeBodyController PRIVATE iDynTree::idyntree-high-level qpOASES ${catkin_LIBRARIES})

add_executable(whole_body_controller_node src/WholeBodyControllerNode.cpp)
target_link_libraries(whole_body_controller_node WholeBodyController ${catkin_LIBRARIES})

add_executable(motion_planner_node src/MotionPlanner.cpp)
target_link_libraries(motion_planner_node towr ifopt_ipopt ${catkin_LIBRARIES})