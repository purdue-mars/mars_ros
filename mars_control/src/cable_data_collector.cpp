// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <mars_control/cable_data_collector.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <mars_msgs/CableFollowingDebug.h>
#include <geometry_msgs/Pose.h>

namespace mars_control
{

  CableDataCollector::CableDataCollector()
      : gelsight_update_struct_()
  {
  }

  bool CableDataCollector::init(hardware_interface::RobotHW *robot_hardware,
                                ros::NodeHandle &node_handle)
  {
    cartesian_velocity_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
    if (cartesian_velocity_interface_ == nullptr)
    {
      ROS_ERROR(
          "CableDataCollector: Could not get Cartesian Velocity "
          "interface from hardware");
      return false;
    }

    if (!node_handle.getParam("fixed_pose/x", fixed_pos_(0)) ||
        !node_handle.getParam("fixed_pose/y", fixed_pos_(1)) ||
        !node_handle.getParam("fixed_pose/z", fixed_pos_(2)) ||
        !node_handle.getParam("fixed_pose/qx", fixed_quat_.x()) ||
        !node_handle.getParam("fixed_pose/qy", fixed_quat_.y()) ||
        !node_handle.getParam("fixed_pose/qz", fixed_quat_.z()) ||
        !node_handle.getParam("fixed_pose/qw", fixed_quat_.w()))
    {
      ROS_ERROR("CableDataCollector: Could not get parameter fixed_pose");
      return false;
    }

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("CableDataCollector: Could not get parameter arm_id");
      return false;
    }

    if (!node_handle.getParam("p_gain", p_gain_))
    {
      ROS_ERROR("CableDataCollector: Could not get parameter p_gain");
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
          "CableDataCollector: Exception getting Cartesian handle: " << e.what());
      return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR("CableDataCollector: Could not get state interface from hardware");
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
              "CableDataCollector: Robot is not in the expected starting position for "
              "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
              "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
          return false;
        }
      }
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CableDataCollector: Exception getting state handle: " << e.what());
      return false;
    }

    data_pub_ = node_handle.advertise<mars_msgs::CableFollowingDebug>("cable_data_output", 10);
    gelsight_sub_ = node_handle.subscribe("/contact", 1, &CableDataCollector::gelsightCallback, this);

    return true;
  }

  void CableDataCollector::starting(const ros::Time & /* time */)
  {
  }

  void CableDataCollector::update(const ros::Time & /* time */,
                                  const ros::Duration &period)
  {
    // Get current EE pose
    std::array<double, 16> m = cartesian_velocity_handle_->getRobotState().O_T_EE_d;
    Eigen::Vector3d pos(m[12], m[13], m[14]);
    double w = sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;
    Eigen::Quaterniond quat(w, (m[6] - m[9]) / (w * 4.0),
                            (m[8] - m[2]) / (w * 4.0),
                            (m[1] - m[4]) / (w * 4.0));

    // Find pose wrt fixed frame
    pos -= fixed_pos_;
    quat *= fixed_quat_.inverse();

    // Get cable pose from GelSight
    GelsightUpdate gelsight_update = *(gelsight_update_.readFromRT());
    double y = gelsight_update.cable_pos(1) + pos(1);
    double theta = 0.0;

    // Calculate model state (y, theta, alpha)
    double alpha = 0.0; // Use x, y and y_global
    Eigen::Vector3d x(y, theta, alpha);

    // Calculate phi from K
    Eigen::Vector3d K(-900.28427003, -9.54405588, 13.36354662);
    double phi = -K.dot(x);

    // Calculate velocity command from phi
    double y_vel = p_gain_ * gelsight_update.cable_pos(1);
    y_vel = fmin(0.05, fmax(-0.05, y_vel));
    ROS_INFO_STREAM("CableDataCollector: cmd " << y_vel << " y " << gelsight_update.cable_pos(1));
    std::array<double, 6>
        cmd = {0.0,
               0.0,
               y_vel,
               0.0,
               0.0,
               0.0};

    // Publish velocity to Franka
    cartesian_velocity_handle_->setCommand(cmd);

    // Create pose msg
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = pos(0);
    pose_msg.position.y = pos(1);
    pose_msg.position.z = pos(2);

    pose_msg.orientation.w = quat.w();
    pose_msg.orientation.x = quat.x();
    pose_msg.orientation.y = quat.y();
    pose_msg.orientation.z = quat.z();

    // Publish debug message
    mars_msgs::CableFollowingDebug msg;
    msg.ee_pose = pose_msg;
    msg.cable_y = y;
    msg.cable_theta = theta;
    msg.cable_alpha = alpha;
    msg.output_phi = phi;
    msg.output_v = 0.0;
    data_pub_.publish(msg);
  }

  void CableDataCollector::gelsightCallback(const geometry_msgs::PoseStamped &msg)
  {
    gelsight_update_struct_.cable_pos(0) = msg.pose.position.x;
    gelsight_update_struct_.cable_pos(1) = msg.pose.position.y;
    gelsight_update_struct_.cable_pos(2) = msg.pose.position.z;

    gelsight_update_struct_.cable_quat.w() = msg.pose.orientation.w;
    gelsight_update_struct_.cable_quat.x() = msg.pose.orientation.x;
    gelsight_update_struct_.cable_quat.y() = msg.pose.orientation.y;
    gelsight_update_struct_.cable_quat.z() = msg.pose.orientation.z;
    gelsight_update_.writeFromNonRT(gelsight_update_struct_);
  }
}

PLUGINLIB_EXPORT_CLASS(mars_control::CableDataCollector, controller_interface::ControllerBase)