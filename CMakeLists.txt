cmake_minimum_required(VERSION 2.8.3)
project(ardrone_follow_tag)
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  geometry_msgs
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
