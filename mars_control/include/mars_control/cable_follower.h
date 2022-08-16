// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace mars_control {

struct FrankaDataContainer {
  std::unique_ptr<franka_hw::FrankaStateHandle>
      state_handle_;  ///< To read to complete robot state.
  std::unique_ptr<franka_hw::FrankaModelHandle>
      model_handle_;  ///< To have access to e.g. jacobians.
  std::vector<hardware_interface::JointHandle> joint_handles_;  ///< To command joint torques.
  double filter_params_{0.005};       ///< [-] PT1-Filter constant to smooth target values set
                                      ///< by dynamic reconfigure servers (stiffness/damping)
                                      ///< or interactive markers for the target poses.
  double nullspace_stiffness_{20.0};  ///< [Nm/rad] To track the initial joint configuration in
                                      ///< the nullspace of the Cartesian motion.
  double nullspace_stiffness_target_{20.0};  ///< [Nm/rad] Unfiltered raw value.
  const double delta_tau_max_{1.0};          ///< [Nm/ms] Maximum difference in joint-torque per
                                             ///< timestep. Used to saturated torque rates to ensure
                                             ///< feasible commands.
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;         ///< To track the target pose.
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;  ///< Unfiltered raw value.
  Eigen::Matrix<double, 6, 6> cartesian_damping_;           ///< To damp cartesian motions.
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;    ///< Unfiltered raw value.
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;               ///< Target joint pose for nullspace
                                                            ///< motion. For now we track the
                                                            ///< initial joint pose.
  Eigen::Vector3d position_d_;               ///< Target position of the end effector.
  Eigen::Quaterniond orientation_d_;         ///< Target orientation of the end effector.
  Eigen::Vector3d position_d_target_;        ///< Unfiltered raw value.
  Eigen::Quaterniond orientation_d_target_;  ///< Unfiltered raw value.
};

struct GelsightUpdate {
  Eigen::Vector3d cable_pos;
  Eigen::Quaterniond cable_quat;

  GelsightUpdate() : cable_pos(0.0, 0.0, 0.0), cable_quat(1.0, 0.0, 0.0, 0.0) {}
};

class CableFollower
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  CableFollower();
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double p_gain_{0.05};
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 6, 1> velocity_d_;

  std::map<std::string, FrankaDataContainer> arms_data_;
  std::string left_arm_id_;
  std::string right_arm_id_;

  // Transformation between base frames of the robots.
  Eigen::Affine3d Ol_T_Or_;  // NOLINT (readability-identifier-naming)
  // Target transformation between the two endeffectors.
  Eigen::Affine3d EEr_T_EEl_;  // NOLINT (readability-identifier-naming)
  // Transformation from the centering frame to the left end effector.
  Eigen::Affine3d EEl_T_C_{};

  // Subscribers / Publishers
  ros::Subscriber gelsight_sub_;

  // Gelsight data
  realtime_tools::RealtimeBuffer<GelsightUpdate> gelsight_update_;
  GelsightUpdate gelsight_update_struct_;
    
  // Slow to stop server 
  ros::ServiceServer slow_srv_;
  bool start_to_slow_;
  std::array<double, 6> last_cmd_;

  /**
   * Saturates torque commands to ensure feasibility.
   *
   * @param[in] tau_d_calculated The raw command according to the control law.
   * @param[in] tau_J_d The current desired torque, read from the robot state.
   * @return The saturated torque command for the 7 joints of one arm.
   */
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  /**
   * Initializes a single Panda robot arm.
   *
   * @param[in] robot_hw A pointer the RobotHW class for getting interfaces and resource handles.
   * @param[in] arm_id The name of the panda arm.
   * @param[in] joint_names The names of all joints of the panda.
   * @return True if successful, false otherwise.
   */
  bool initArm(hardware_interface::RobotHW* robot_hw,
               const std::string& arm_id,
               const std::vector<std::string>& joint_names);

  /**
   * Computes the decoupled controller update for a single arm.
   *
   * @param[in] arm_data The data container of the arm to control.
   */
  void updateArm(FrankaDataContainer& arm_data);

  /**
   * Prepares all internal states to be ready to run the real-time control for one arm.
   *
   * @param[in] arm_data The data container of the arm to prepare for the control loop.
   */
  void startingArm(FrankaDataContainer& arm_data);

  ///< Target pose subscriber
  ros::Subscriber sub_target_pose_left_;

  /**
   * Callback method that handles updates of the gelsight poses.
   *
   * @param[in] msg New pose.
   */
  void gelsightCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * Publishes a Pose Stamped for visualization of the current centering pose.
   */
  void publishCenteringPose();
};

} // namespace mars_control