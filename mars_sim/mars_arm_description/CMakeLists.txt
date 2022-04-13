cmake_minimum_required(VERSION 3.0.2)
project(mars_arm_description)

find_package(catkin REQUIRED COMPONENTS
  franka_description
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
