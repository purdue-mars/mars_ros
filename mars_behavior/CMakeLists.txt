cmake_minimum_required(VERSION 3.0.2)
project(mars_behavior)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  actionlib_msgs
  geometry_msgs
)

add_action_files(DIRECTORY action FILES DoVSRigidPickup.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
