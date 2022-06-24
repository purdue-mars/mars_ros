// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <mars_control/cable_follower.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <franka_hw/franka_velocity_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace mars_control
{

  bool CableFollower::init(hardware_interface::RobotHW *robot_hardware,
                                     ros::NodeHandle &node_handle)
  {
    cartesian_velocity_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
    if (cartesian_velocity_interface_ == nullptr)
    {
      ROS_ERROR(
          "CableFollower: Could not get Cartesian Velocity "
          "interface from hardware");
      return false;
    }

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("CableFollower: Could not get parameter arm_id");
      return false;
    }

    try
    {
      cartesian_velocity_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
          cartesian_velocity_interface_->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CableFollower: Exception getting Cartesian handle: " << e.what());
      return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR("CableFollower: Could not get state interface from hardware");
      return false;
    }

    try
    {
      auto state_handle = state_interface->getHandle(arm_id + "_robot");

      std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      for (size_t i = 0; i < q_start.size(); i++)
      {
        if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
        {
          ROS_ERROR_STREAM(
              "CableFollower: Robot is not in the expected starting position for "
              "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
              "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
          return false;
        }
      }
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CableFollower: Exception getting state handle: " << e.what());
      return false;
    }

    // node_handle.subscribe("desired_pose", 1, &CartesianVelocityController::desiredPoseCallback, this);

    return true;
  }

  void CableFollower::starting(const ros::Time & /* time */)
  {
      elapsed_time_ = ros::Duration(0.0);
  }

  void CableFollower::update(const ros::Time & /* time */,
                             const ros::Duration &period)
  {
    elapsed_time_ += period;

    // Get cable pose from GelSight
    float y = 0.0;
    float theta = 0.0;

    // Calculate model state (y, theta, alpha)
    float alpha = 0.0; // Use x, y and y_global
    Eigen::Vector3d x(y, theta, alpha);

    // Calculate phi from K
    Eigen::Vector3d K(-900.28427003, -9.54405588, 13.36354662);
    float phi = -K.dot(x);

    // Calculate velocity command from phi
    float target_dir = phi + alpha;
    target_dir = max(-3.14159265 / 3.0, min(target_dir, 3.14159265 / 3.0));
    float vnorm = 0.025;
    std::array<float, 6>
        cmd = {{vnorm * cos(target_dir), vnrom * sin(target_dir), 0.0, 0.0, 0.0, 0.0}};

    // Publish velocity to Franka
    velocity_cartesian_handle_->setCommand(cmd);
  }

  void CableFollower::stopping(const ros::Time&) {
    // DO NOT PUBLISH ZERO VELOCITIES
  }

}

PLUGINLIB_EXPORT_CLASS(mars_control::CableFollower, controller_interface::ControllerBase)