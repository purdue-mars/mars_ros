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
  dynamic_reconfigure
  mars_config
)

find_package(PCL REQUIRED) # This includes all modules
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
find_package(Ceres REQUIRED)


find_package(realsense2 2.50.0)

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mars_perception
#  CATKIN_DEPENDS 
  CATKIN_DEPENDS
    roscpp 
    sensor_msgs 
    pcl_conversions
    pcl_ros
    mars_msgs
    mars_config
    dynamic_reconfigure

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
  ${Open3D_INCLUDE_DIRS} 
  ${CERES_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS} ${Open3D_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_processor src/pc_processor_node.cpp)
set_target_properties(${PROJECT_NAME}_processor PROPERTIES OUTPUT_NAME pc_processor PREFIX "")
add_dependencies(${PROJECT_NAME}_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_processor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(${PROJECT_NAME}_registration src/registration_server.cpp src/registration/icp.cpp src/mesh_sampling.cpp src/common.cpp src/registration/constrained_icp.cpp)
set_target_properties(${PROJECT_NAME}_registration PROPERTIES OUTPUT_NAME registration_server PREFIX "")
add_dependencies(${PROJECT_NAME}_registration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_registration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Open3D_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(${PROJECT_NAME}_registration_test src/registration_test.cpp)
set_target_properties(${PROJECT_NAME}_registration_test PROPERTIES OUTPUT_NAME registration_test PREFIX "")
add_dependencies(${PROJECT_NAME}_registration_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_registration_test
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
  #find_package(rostest REQUIRED)
  #catkin_add_nosetests(test)
endif()