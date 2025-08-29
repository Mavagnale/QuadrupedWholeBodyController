cmake_minimum_required(VERSION 3.0.2)
project(anymal_wbc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  tf
)

## Find Qt5 (Widgets is the module needed for GUI components)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

## Qt's MOC (meta-object compiler) support
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

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
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_subdirectory(lib/qpOASES)
add_subdirectory(lib/idyntree)

add_library(WholeBodyController src/whole_body_controller.cpp)
target_link_libraries(WholeBodyController PRIVATE iDynTree::idyntree-high-level qpOASES ${catkin_LIBRARIES})

add_executable(whole_body_controller_node src/whole_body_controller_node.cpp)
target_link_libraries(whole_body_controller_node WholeBodyController ${catkin_LIBRARIES})

add_executable(teleop_gui_node src/teleop_gui_node.cpp)
target_link_libraries(teleop_gui_node ${catkin_LIBRARIES} Qt5::Widgets)

add_library(motion_planner src/motion_planner.cpp)
target_link_libraries(motion_planner ${catkin_LIBRARIES})

add_executable(motion_planner_node src/motion_planner_node.cpp)
target_link_libraries(motion_planner_node motion_planner ${catkin_LIBRARIES})

add_executable(marker_publisher src/marker_publisher.cpp)
target_link_libraries(marker_publisher ${catkin_LIBRARIES})