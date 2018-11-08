cmake_minimum_required(VERSION 2.8.3)
project(web_gcs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS rospy
)
