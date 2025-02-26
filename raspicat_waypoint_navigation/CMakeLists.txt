cmake_minimum_required(VERSION 2.8.3)
project(raspicat_waypoint_navigation)

add_compile_options(-std=c++14)

link_libraries(yaml-cpp)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs
  raspicat_navigation_msgs
  tf
  pluginlib
  roslib
  ddynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    raspicat_navigation_msgs
    ddynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# WaypointNav
add_library(WaypointNav
  src/WaypointNav.cpp
)
add_executable(WaypointNav_node
  src/WaypointNav_node.cpp
)
target_link_libraries(
  WaypointNav_node
  WaypointNav
  ${catkin_LIBRARIES}
)

# WaypointView
add_executable(WaypointView_node
  src/tools/waypoint_view.cpp
)
target_link_libraries(
  WaypointView_node
  WaypointNav
  ${catkin_LIBRARIES}
)

# waypoint_rviz_set
add_executable(waypoint_rviz_set src/tools/waypoint_set.cpp)
target_link_libraries(waypoint_rviz_set ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  src/WaypointServer.cpp
  src/WaypointRviz.cpp
  src/ParamChange.cpp
  src/SlopeObstacleAvoidance.cpp
  src/ClearCostMap.cpp
  src/WaitingLine.cpp
)

install(DIRECTORY launch src config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB python_scripts scripts/*.py)
catkin_install_python(
  PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)