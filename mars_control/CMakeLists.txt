cmake_minimum_required(VERSION 3.4)
project(mars_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  pluginlib
  roscpp
  genmsg
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  actionlib_msgs
  controller_interface
  mars_msgs
  franka_hw
  franka_gripper
)

find_package(Franka 0.9.0 QUIET)

if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  hardware_interface
  roscpp
  controller_interface
  franka_hw
  franka_gripper
  mars_msgs
  pluginlib

  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  tf
  tf_conversions
  message_runtime
  realtime_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ##########
# # Build ##
# ##########

# Specify header include paths
add_executable(${PROJECT_NAME}_planning_server
  src/planning_server.cpp src/kinematics.cpp
)
set_target_properties(${PROJECT_NAME}_planning_server PROPERTIES
  OUTPUT_NAME planning_server
  PREFIX ""
)
add_dependencies(${PROJECT_NAME}_planning_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_planning_server
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${Boost_LIBRARIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/cartesian_pose_controller.cpp
  src/cable_follower.cpp
  src/cable_data_collector.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${Franka_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)