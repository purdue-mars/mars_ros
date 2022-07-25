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
#include <random>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <mars_msgs/CableFollowingData.h>
#include <geometry_msgs/Pose.h>

std::array<double, 16> poseToTransform(Eigen::Vector3d pos, Eigen::Quaterniond quat)
{
  Eigen::Matrix3d rot = quat.toRotationMatrix();

  return {rot(0, 0), rot(1, 0), rot(2, 0), 0.0,
          rot(0, 1), rot(1, 1), rot(2, 1), 0.0,
          rot(0, 2), rot(1, 2), rot(2, 2), 0.0,
          pos(0), pos(1), pos(2), 1.0};
}

namespace mars_control
{

  CableDataCollector::CableDataCollector()
      : gelsight_update_struct_()
  {
  }

  bool CableDataCollector::init(hardware_interface::RobotHW *robot_hardware,
                                ros::NodeHandle &node_handle)
  {
    // Obtain a seed
    std::random_device rd;
    rand_gen_ = std::mt19937(rd());
    rand_dis_ = std::uniform_real_distribution<>(-0.01, 0.01);

    cartesian_velocity_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
    if (cartesian_velocity_interface_ == nullptr)
    {
      ROS_ERROR(
          "CableDataCollector: Could not get Cartesian Velocity "
          "interface from hardware");
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

    start_to_slow_ = false;

    // try
    // {
    //   auto state_handle = state_interface->getHandle(arm_id + "_robot");
    //   auto start_transform = poseToTransform(start_pos, start_quat);
    //   for (size_t i = 0; i < start_transform.size(); i++)
    //   {
    //     if (std::abs(state_handle.getRobotState().O_T_EE[i] - start_transform[i]) > 0.1)
    //     {
    //       ROS_ERROR_STREAM(
    //           "CableDataCollector: Robot is not in the expected starting position for "
    //           << start_pose_name << ". Run `roslaunch mars_control move_to.launch "
    //                                 "robot_ip:=<robot-ip> pose_name:="
    //           << start_pose_name << "` first.");
    //       return false;
    //     }
    //   }
    // }
    // catch (const hardware_interface::HardwareInterfaceException &e)
    // {
    //   ROS_ERROR_STREAM(
    //       "CableDataCollector: Exception getting state handle: " << e.what());
    //   return false;
    // }

    data_pub_ = new realtime_tools::RealtimePublisher<mars_msgs::CableFollowingData>(node_handle, "cable_data_output", 10);
    gelsight_sub_ = node_handle.subscribe("/gelsight/pose", 1, &CableDataCollector::gelsightCallback, this);
    slow_srv_ = node_handle.advertiseService("slow_controller", &CableDataCollector::slowToStop, this);
    last_y_vel_ = 0.0;
    return true;
  }

  void CableDataCollector::starting(const ros::Time & /* time */) {
    std::array<double, 16> m = cartesian_velocity_handle_->getRobotState().O_T_EE_d;
    cable_origin_pos_ = Eigen::Vector3d(m[12], m[13], m[14]);
    double w = sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;
    cable_origin_quat_ = Eigen::Quaterniond(w, (m[6] - m[9]) / (w * 4.0),
                                            (m[8] - m[2]) / (w * 4.0),
                                            (m[1] - m[4]) / (w * 4.0));
    start_to_slow_ = false;
  }

  void CableDataCollector::update(const ros::Time & /* time */,
                                  const ros::Duration &period)
  {
    double max_accel = 1.5;

    if (start_to_slow_) {
      double x_vel = 0.0;
      if (last_x_vel_ > max_accel * period.toSec()) {
        x_vel = last_x_vel_ - (max_accel * period.toSec());
      } else if (last_x_vel_ < -max_accel * period.toSec()) {
        x_vel = last_x_vel_ + (max_accel * period.toSec());
      }
      last_x_vel_ = x_vel;

      double y_vel = 0.0;
      if (last_y_vel_ > max_accel * period.toSec()) {
        y_vel = last_y_vel_ - (max_accel * period.toSec());
      } else if (last_y_vel_ < -max_accel * period.toSec()) {
        y_vel = last_y_vel_ + (max_accel * period.toSec());
      }
      last_y_vel_ = y_vel;

      std::array<double, 6>
          cmd = {0.0,
                x_vel,
                y_vel,
                0.0,
                0.0,
                0.0};
      cartesian_velocity_handle_->setCommand(cmd);
      return;
    } 
    
    // Get current EE pose
    std::array<double, 16> m = cartesian_velocity_handle_->getRobotState().O_T_EE_d;
    Eigen::Vector3d pos(m[12], m[13], m[14]);
    double w = sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;
    Eigen::Quaterniond quat(w, (m[6] - m[9]) / (w * 4.0),
                            (m[8] - m[2]) / (w * 4.0),
                            (m[1] - m[4]) / (w * 4.0));

    // Find pose wrt fixed frame
    pos -= cable_origin_pos_;
    quat *= cable_origin_quat_.inverse();

    // Get cable pose from GelSight
    GelsightUpdate gelsight_update = *(gelsight_update_.readFromRT());
    double cable_x = gelsight_update.cable_pos(0) + pos(0);
    double cable_y = gelsight_update.cable_pos(1) + pos(1);

    Eigen::Vector3d cable_euler = gelsight_update.cable_quat.toRotationMatrix().eulerAngles(0, 1, 2);
    double theta = cable_euler[2];

    // Calculate model state
    double alpha = atan2(cable_y, cable_x);
    Eigen::Vector3d state(cable_y, theta, alpha);

    // Calculate velocity command from phi
    double v_norm = -0.01;
    double y_vel = p_gain_ * gelsight_update.cable_pos(1);
    y_vel = fmin(0.1, fmax(-0.1, y_vel));

    // Add uniform noise
    // y_vel += rand_dis_(rand_gen_);

    // Limit acceleration 
    double accel = (y_vel - last_y_vel_) / period.toSec();
    if (accel > max_accel) {
      y_vel = (max_accel * period.toSec()) + last_y_vel_;
    } else if (accel < -max_accel) {
      y_vel = (-max_accel * period.toSec()) + last_y_vel_;
    }
    last_x_vel_ = v_norm;
    last_y_vel_ = y_vel;
    
    // Create command
    std::array<double, 6>
        cmd = {0.0,
               v_norm,
               y_vel,
               0.0,
               0.0,
               0.0};
    double phi = atan2(y_vel, v_norm) - alpha;

    // Publish velocity to Franka
    cartesian_velocity_handle_->setCommand(cmd);

    // Create pose msg
    if (data_pub_->trylock())
    {
      data_pub_->msg_.ee_pose.position.x = pos(0);
      data_pub_->msg_.ee_pose.position.y = pos(1);
      data_pub_->msg_.ee_pose.position.z = pos(2);
      data_pub_->msg_.ee_pose.orientation.x = quat.x();
      data_pub_->msg_.ee_pose.orientation.y = quat.y();
      data_pub_->msg_.ee_pose.orientation.z = quat.z();
      data_pub_->msg_.ee_pose.orientation.w = quat.w();
      data_pub_->msg_.cable_y = gelsight_update.cable_pos(1);
      data_pub_->msg_.cable_theta = theta;
      data_pub_->msg_.cable_alpha = alpha;
      data_pub_->msg_.output_phi = phi;
      data_pub_->msg_.output_v = v_norm;
      data_pub_->unlockAndPublish();
    }
  }

  void CableDataCollector::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
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

  bool CableDataCollector::slowToStop(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& resp)
  {
    start_to_slow_ = true;
    ROS_INFO("CableDataCollector: set to slow.");
    return true;
  }
}

PLUGINLIB_EXPORT_CLASS(mars_control::CableDataCollector, controller_interface::ControllerBase)