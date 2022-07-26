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

    // Collect ROS params
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

    if (!node_hande.getParam("is_vertical", is_vertical_))
    {
      ROS_ERROR("CableDataCollector: Could not get parameter is_vertical");
      return false;
    }

    std::string gelsight_topic;
    if (!node_handle.getParam("gelsight_topic", gelsight_topic)) {
      ROS_ERROR("CableDataCollector: Could not get parameter gelsight_topic");
      return false
    }

    std::string output_topic;
    if (!node_handle.getParam("output_topic", output_topic)) {
      ROS_ERROR("CableDataCollector: Could not get parameter output_topic");
      return false
    }

    // Setup Franka interface
    cartesian_velocity_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
    if (cartesian_velocity_interface_ == nullptr)
    {
      ROS_ERROR(
          "CableDataCollector: Could not get Cartesian Velocity "
          "interface from hardware");
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

    // Slow to stop server
    slow_srv_ = node_handle.advertiseService("slow_controller", &CableDataCollector::slowToStop, this);

    // Publisher / Subscribers
    gelsight_sub_ = node_handle.subscribe(gelsight_topic, 1, &CableDataCollector::gelsightCallback, this);
    data_pub_ = new realtime_tools::RealtimePublisher<mars_msgs::CableFollowingData>(node_handle, output_topic, 10);
    return true;
  }

  void CableDataCollector::starting(const ros::Time & /* time */) {
    // Store starting state as cable origin
    std::array<double, 16> m = cartesian_velocity_handle_->getRobotState().O_T_EE_d;
    cable_origin_pos_ = Eigen::Vector3d(m[12], m[13], m[14]);
    double w = sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;
    cable_origin_quat_ = Eigen::Quaterniond(w, (m[6] - m[9]) / (w * 4.0),
                                            (m[8] - m[2]) / (w * 4.0),
                                            (m[1] - m[4]) / (w * 4.0));
    
    // Reset slow to stop
    start_to_slow_ = false;
    last_cmd_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  void CableDataCollector::update(const ros::Time & /* time */,
                                  const ros::Duration &period)
  {
    double max_accel = 1.5;

    if (start_to_slow_) {
      std::array<double, 6> cmd = last_cmd_;
      double max_delta_vel = max_accel * period.toSec();
      for (int i = 0; i < 6; i++) {
        if (cmd[i] > max_delta_vel) {
          cmd[i] -= max_delta_vel;
        } else if (cmd[i] < -max_delta_vel) {
          cmd[i] += max_delta_vel;
        } else {
          cmd[i] = 0.0;
        }
      }
      last_cmd_ = cmd;
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
    double fwd_vel = -0.01;
    double norm_vel = p_gain_ * gelsight_update.cable_pos(1);
    norm_vel = fmin(0.1, fmax(-0.1, norm_vel));

    // Add uniform noise
    // y_vel += rand_dis_(rand_gen_);

    // Create command
    std::array<double, 6> cmd;
    if (is_vertical_) {
      cmd = {0.0, fwd_vel, norm_vel, 0.0, 0.0, 0.0};
    } else {
      cmd = {fwd_vel, norm_vel, 0.0, 0.0, 0.0};
    }

    // Limit acceleration 
    for (int i = 0; i < 6; i++) {
      double accel = (cmd[i] - last_cmd_[i]) / period.toSec();
      if (accel > max_accel) {
        cmd[i] = (max_accel * period.toSec()) + last_cmd_[i];
      } else if (accel < -max_accel) {
        cmd[i] = (-max_accel * period.toSec()) + last_cmd_[i]; 
      }
    }

    // Publish velocity to Franka
    cartesian_velocity_handle_->setCommand(cmd);
    last_cmd_ = cmd;

    // Create pose msg
    // double phi = atan2(, v_norm) - alpha;
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