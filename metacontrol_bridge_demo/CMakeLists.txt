cmake_minimum_required(VERSION 3.0.2)
project(metacontrol_bridge_demo)

find_package(catkin REQUIRED COMPONENTS
  metacontrol_bridge
  rospy
)

catkin_package(
  CATKIN_DEPENDS metacontrol_bridge rospy lifecycle_msgs
)

catkin_install_python(PROGRAMS
  scripts/demo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
