cmake_minimum_required(VERSION 3.0.2)
project(metacontrol_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  lifecycle_msgs
)

catkin_python_setup()

catkin_package()
