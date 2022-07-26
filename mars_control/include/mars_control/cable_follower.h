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
#include <mars_msgs/CableFollowingData.h>
#include <random>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace mars_control
{
    class CableFollower
        : public controller_interface::MultiInterfaceController<franka_hw::FrankaVelocityCartesianInterface,
                                                                franka_hw::FrankaStateInterface>
    {
    public:
        CableFollower();
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;
        void stopping(const ros::Time &) override;
        void gelsightCallback(const geometry_msgs::PoseStamped &);
        bool slowToStop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

    private:
        franka_hw::FrankaVelocityCartesianInterface *cartesian_velocity_interface_;
        std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> cartesian_velocity_handle_;

        // Controller configuration
        bool is_vertical_;
        double p_gain_;
        Eigen::Vector3d cable_origin_pos_;
        Eigen::Quaterniond cable_origin_quat_;

        // Subscribers / Publishers
        ros::Subscriber gelsight_sub_;
        realtime_tools::RealtimePublisher<mars_msgs::CableFollowingData> *data_pub_;

        // Gelsight data
        struct GelsightUpdate
        {
            Eigen::Vector3d cable_pos;
            Eigen::Quaterniond cable_quat;

            GelsightUpdate() : cable_pos(0.0, 0.0, 0.0), cable_quat(1.0, 0.0, 0.0, 0.0) {}
        };
        realtime_tools::RealtimeBuffer<GelsightUpdate> gelsight_update_;
        GelsightUpdate gelsight_update_struct_;
        
        // Slow to stop server 
        ros::ServiceServer slow_srv_;
        bool start_to_slow_;
        std::array<double, 6> last_cmd_;
        
        // Uniform noise 
        std::mt19937 rand_gen_;
        std::uniform_real_distribution<> rand_dis_;
    };
}