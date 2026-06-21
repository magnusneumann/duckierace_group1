cmake_minimum_required(VERSION 3.0.2)
project(avoid_ducks)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  duckietown_msgs
)
