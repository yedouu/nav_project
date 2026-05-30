cmake_minimum_required(VERSION 3.0)
project(scan_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_corrector_node src/scan_corrector_node.cpp)
target_link_libraries(scan_corrector_node ${catkin_LIBRARIES})
