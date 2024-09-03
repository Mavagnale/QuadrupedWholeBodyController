cmake_minimum_required(VERSION 3.0.2)
project(anymal_wbc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)

find_package(iDynTree REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES anymal_wbc
#  CATKIN_DEPENDS roscpp std_msgs
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
target_link_libraries(WholeBodyController PRIVATE iDynTree::idyntree-high-level iDynTree::idyntree-estimation qpOASES towr ${catkin_LIBRARIES})

add_executable(effortPublisher src/effortPublisher.cpp)
target_link_libraries(effortPublisher ${catkin_LIBRARIES})

add_executable(wholeBodyControllerNode src/wholeBodyControllerNode.cpp)
target_link_libraries(wholeBodyControllerNode WholeBodyController ${catkin_LIBRARIES})
