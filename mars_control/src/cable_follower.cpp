#include <mars_control/cable_follower.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <mars_control/pseudo_inversion.h>

std::array<double, 16> poseToTransform(Eigen::Vector3d pos, Eigen::Quaterniond quat)
{
  Eigen::Matrix3d rot = quat.toRotationMatrix();

  return {rot(0, 0), rot(1, 0), rot(2, 0), 0.0,
          rot(0, 1), rot(1, 1), rot(2, 1), 0.0,
          rot(0, 2), rot(1, 2), rot(2, 2), 0.0,
          pos(0), pos(1), pos(2), 1.0};
}

namespace mars_control {

CableFollower::CableFollower()
      : gelsight_update_struct_()
  {
  }

bool CableFollower::init(hardware_interface::RobotHW* robot_hw,
                         ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CableFollower: Could not read parameter arm_id");
    return false;
  }


  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CableFollower: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("p_gain", p_gain_))
  {
    ROS_ERROR("CableFollower: Could not get parameter p_gain");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CableFollower: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CableFollower: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CableFollower: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CableFollower: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CableFollower: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CableFollower: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void CableFollower::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  velocity_d_.setZero();

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CableFollower::update(const ros::Time& /*time*/,
                           const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> velocity(jacobian * dq); 

  // Get cable pose from GelSight
  GelsightUpdate gelsight_update = *(gelsight_update_.readFromRT());
  double cable_x = gelsight_update.cable_pos(0) + position(0);
  double cable_y = gelsight_update.cable_pos(1) + position(1);

  Eigen::Vector3d cable_euler = gelsight_update.cable_quat.toRotationMatrix().eulerAngles(0, 1, 2);
  double theta = cable_euler[2];

  // Calculate model state
  double alpha = atan2(cable_y, cable_x);
  Eigen::Vector3d state(cable_y, theta, alpha);

  // Calculate velocity command from phi
  double fwd_vel = -0.01;
  double norm_vel = p_gain_ * gelsight_update.cable_pos(1);
  norm_vel = fmin(0.1, fmax(-0.1, norm_vel));

  // Create command
  std::array<double, 6> cmd;
  // if (is_vertical_) {
  //  cmd = {0.0, fwd_vel, norm_vel, 0.0, 0.0, 0.0};
  velocity_d_ << fwd_vel, norm_vel, 0.0, 0.0, 0.0, 0.0;
  // } else {
  //  position_d_ << fwd_vel * period, norm_vel * period, 0.0;
  //}

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error << velocity - velocity_d_;

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  // cartesian_stiffness_ =
  //    filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  // cartesian_damping_ =
  //    filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  // nullspace_stiffness_ =
  //    filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  // std::lock_guard<std::mutex> position_d_target_mutex_lock(
  //    position_and_orientation_d_target_mutex_);
  // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  // orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CableFollower::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CableFollower::gelsightCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  gelsight_update_struct_.cable_pos(0) = msg->pose.position.x;
  gelsight_update_struct_.cable_pos(1) = msg->pose.position.y;
  gelsight_update_struct_.cable_pos(2) = msg->pose.position.z;

  gelsight_update_struct_.cable_quat.w() = msg->pose.orientation.w;
  gelsight_update_struct_.cable_quat.x() = msg->pose.orientation.x;
  gelsight_update_struct_.cable_quat.y() = msg->pose.orientation.y;
  gelsight_update_struct_.cable_quat.z() = msg->pose.orientation.z;
  gelsight_update_.writeFromNonRT(gelsight_update_struct_);
}

// bool CableFollowerCombined::slowToStop(std_srvs::Empty::Request& req,
//                                     std_srvs::Empty::Response& resp)
// {
//   start_to_slow_ = true;
//   ROS_INFO("CableFollowerCombined: set to slow.");
//   return true;
// }
}  // namespace mars_control

PLUGINLIB_EXPORT_CLASS(mars_control::CableFollower,
                       controller_interface::ControllerBase)