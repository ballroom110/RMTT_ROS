cmake_minimum_required(VERSION 3.0.2)
project(tello_control_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  tello_driver
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nav_msgs std_msgs tello_driver
)

install(PROGRAMS
  scripts/keyboard_control_node.py
  scripts/tello_control_mux_node.py
  scripts/tello_odom_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
