// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace mars_control
{
    class CableDataCollector
        : public controller_interface::MultiInterfaceController<franka_hw::FrankaVelocityCartesianInterface,
                                                                franka_hw::FrankaStateInterface>
    {
    public:
        CableDataCollector();
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;
        void gelsightCallback(const geometry_msgs::PoseStamped &);

    private:
        franka_hw::FrankaVelocityCartesianInterface *cartesian_velocity_interface_;
        std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> cartesian_velocity_handle_;

        // Controller configuration
        double p_gain_;
        Eigen::Vector3d cable_origin_pos_;
        Eigen::Quaterniond cable_origin_quat_;

        // Subscribers / Publishers
        ros::Subscriber gelsight_sub_;
        ros::Publisher data_pub_;

        // Gelsight data
        struct GelsightUpdate
        {
            Eigen::Vector3d cable_pos;
            Eigen::Quaterniond cable_quat;

            GelsightUpdate() : cable_pos(0.0, 0.0, 0.0), cable_quat(1.0, 0.0, 0.0, 0.0) {}
        };
        realtime_tools::RealtimeBuffer<GelsightUpdate> gelsight_update_;
        GelsightUpdate gelsight_update_struct_;
    };
}