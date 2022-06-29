cmake_minimum_required(VERSION 3.0.2)
project(mars_behavior)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  actionlib_msgs
  geometry_msgs
  actionlib
  mars_msgs
  tf
)

catkin_python_setup()

add_action_files(DIRECTORY action FILES DoVSRigidPickup.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS 
      roscpp
      actionlib_msgs
      actionlib
      mars_msgs
      tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_pick nodes/pick_node.cpp)
# set_target_properties(${PROJECT_NAME}_pick PROPERTIES OUTPUT_NAME pick PREFIX "")
# add_dependencies(${PROJECT_NAME}_pick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_pick
#   ${catkin_LIBRARIES}
# )