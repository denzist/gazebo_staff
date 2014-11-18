cmake_minimum_required(VERSION 2.8.3)
project(ground_truth_publisher_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  nav_msgs
  roscpp
  sensor_msgs
)
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(ground_truth_publisher_plugin src/ground_truth_publisher_plugin.cpp)
target_link_libraries(ground_truth_publisher_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
