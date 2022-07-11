#include <ros/ros.h>
#include <mars_msgs/MoveToAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <stdlib.h>

#pragma once

#define SERVER_NAME "move_to"

class PlanningServer {

    private:
        ros::NodeHandle nh_;
        std::string planning_group_, base_link_, ee_link_;
        actionlib::SimpleActionServer<mars_msgs::MoveToAction> as_;

    public:
        PlanningServer();
        void execute(const mars_msgs::MoveToGoalConstPtr &goal);
};