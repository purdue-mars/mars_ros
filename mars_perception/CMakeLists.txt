cmake_minimum_required(VERSION 3.0.2)
project(mars_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  realsense2_camera
  sensor_msgs 
  pcl_conversions
  pcl_ros
  mars_msgs
)

find_package(PCL REQUIRED) # This includes all modules
find_package(Eigen3 REQUIRED)


find_package(realsense2 2.50.0)

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mars_perception
#  CATKIN_DEPENDS 
  CATKIN_DEPENDS
    roscpp 
    realsense2_camera
    sensor_msgs 
    pcl_conversions
    pcl_ros
    mars_msgs

  DEPENDS EIGEN3 

#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_reg nodes/pc_registration_node.cpp src/registration.cpp)
set_target_properties(${PROJECT_NAME}_reg PROPERTIES OUTPUT_NAME pc_registration PREFIX "")
add_dependencies(${PROJECT_NAME}_reg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_reg
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_icp_server nodes/icp_server.cpp src/icp.cpp src/mesh_sampling.cpp)
set_target_properties(${PROJECT_NAME}_icp_server PROPERTIES OUTPUT_NAME icp_server PREFIX "")
add_dependencies(${PROJECT_NAME}_icp_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_icp_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(${PROJECT_NAME}_icp_client nodes/icp_client_test.cpp)
set_target_properties(${PROJECT_NAME}_icp_client PROPERTIES OUTPUT_NAME icp_client PREFIX "")
add_dependencies(${PROJECT_NAME}_icp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_icp_client
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
  #find_package(rostest REQUIRED)
  #catkin_add_nosetests(test)
endif()