cmake_minimum_required(VERSION 3.0.2)
project(lane_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  src/lane_follower_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
