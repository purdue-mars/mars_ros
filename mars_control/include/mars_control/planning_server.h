#include <ros/ros.h>
#include <mars_msgs/MoveToAction.h>
#include <mars_msgs/MoveToTargetAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <stdlib.h>
#include <algorithm>

#pragma once

#define DEFAULT_PLANNING_GROUP "panda_arm"

class PlanningServer
{

private:
    ros::NodeHandle nh_;
    std::string global_planning_group_;
    actionlib::SimpleActionServer<mars_msgs::MoveToAction> as_;
    actionlib::SimpleActionServer<mars_msgs::MoveToTargetAction> target_as_;

public:
    PlanningServer();
    void execute(const mars_msgs::MoveToGoalConstPtr &goal);
    void execute_target(const mars_msgs::MoveToTargetGoalConstPtr &goal);
    void set_combined_traj(moveit_msgs::RobotTrajectory &traj, std::string planning_group);
    double eef_step_, jump_threshold_, vel_scaling_factor_, accel_scaling_factor_;
};