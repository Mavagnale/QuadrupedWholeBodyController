cmake_minimum_required(VERSION 3.0.2)
project(anymal_wbc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)

find_package(iDynTree REQUIRED)
find_package(Qt5Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES anymal_wbc
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(lib/qpOASES)
add_subdirectory(lib/idyntree)

add_library(WholeBodyController src/WholeBodyController.cpp)
target_link_libraries(WholeBodyController PRIVATE iDynTree::idyntree-high-level iDynTree::idyntree-estimation qpOASES ${catkin_LIBRARIES})

add_executable(jointCmdPublisher src/jointCmdPublisher.cpp)
target_link_libraries(jointCmdPublisher ${catkin_LIBRARIES})

add_executable(sliderPublisher src/sliderPublisher.cpp src/sliderGui.cpp)

target_link_libraries(sliderPublisher
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

add_executable(main src/main.cpp)
target_link_libraries(main WholeBodyController ${catkin_LIBRARIES})
