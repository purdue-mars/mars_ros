cmake_minimum_required(VERSION 3.0.2)
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
  mars_msgs
)

find_package(Eigen3 REQUIRED)


# Declare a catkin package
catkin_package(
    CATKIN_DEPENDS 
      hardware_interface
      roscpp
      controller_interface
      mars_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########

# Specify header include paths
add_executable(${PROJECT_NAME}_planning_server
  src/planning_server.cpp
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
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

# Install headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
# 
# # Install targets
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# 
# # Install plugins
# install(FILES mars_control_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )