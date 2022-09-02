# mars_ros


### mars_behavior
This package includes a simple python interface to:
- Run/switch between ROS controllers using the `TaskInterface`
- Command the end-effector by defining cartesian pose goals for the end-effector with the `ArmInterface`
- Control the WSG 32 gripper and Panda gripper with the `GripperInterface`
- Simplify coordinate frame conversions for a robot arm using the `TFInterface` 
- Run object registration algorithms using the `PerceptionInterface`

Includes high-level behaviors that implement the above interfaces

### mars_config
Houses all yaml config files for `perception`, `gelsight`, `gripper`, `detection` components of the system

### mars_control
Includes custom ROS controller definitions for specific tasks and a planning interface with MoveIt

### mars_launch
Houses all the main launch files such as `robot.launch`, `robots.launch` for a bring-up of the system

### mars_msgs
Houses all the custom ROS msgs, srv, action definitions

### mars_panda
Includes all the forks of the `franka_description`, `panda_moveit_config`, `franka_control`, and `franka_hw` for the lab's single arm and dual arm setups 

### mars_perception
- `src/registration_server.cpp`: Point set registration for objects and scenes using ICP and extensibility to other algorithms
- `src/mars_perception`: Python package with utility functions, ML training scripts, 
- `launch/cameras.launch`: Realsense2 camera bringup launch file
- `launch/detection.launch`: Object segmentation inference launch file 
- `scripts/depth_mask_node`: Convenience node to mask color/depth images and project to point cloud
- Point cloud processing node

### mars_sim
- Sim itself is depreciated
- `nist_board_gazebo`: Includes NIST task board stl assets used for model-based point set registration

### third_party
- `detectron2_ros`: Runs detectron2 inference model and sub/pubs in ROS
- `gelsight-ros`: Gelsight ROS interface
- `moveit`
- `wsg50-ros-pkg`: custom fork of the wsg gripper ros interface
