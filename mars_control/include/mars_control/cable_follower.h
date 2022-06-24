// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace mars_control
{
    class CableFollower
        : public controller_interface::MultiInterfaceController<franka_hw::FrankaVelocityCartesianInterface,
                                                                franka_hw::FrankaStateInterface>
    {
    public:
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;
        void desiredPoseCallback(const std_msgs::String::ConstPtr &);

    private:
        franka_hw::FrankaVelocityCartesianInterface *cartesian_velocity_interface_;
        std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> cartesian_velocity_handle_;
        ros::Duration elapsed_time_;
        std::array<double, 16> initial_pose_{};
        std::string test;
    };
}