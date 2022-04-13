cmake_minimum_required(VERSION 3.0.2)
project(mars_launch)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
endif()