cmake_minimum_required(VERSION 3.0.2)
project(shape_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/shape_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
