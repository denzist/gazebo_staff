cmake_minimum_required(VERSION 2.8.3)
project(tilting_unit_control_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  nav_msgs
  tf
  roscpp
  sensor_msgs
)
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(tilting_unit_gazebo_plugin src/tilting_unit_gazebo_plugin.cpp)
target_link_libraries(tilting_unit_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})