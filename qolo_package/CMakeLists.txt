cmake_minimum_required(VERSION 2.8.3)
project(qolo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rds_network_ros
)


catkin_package()