/* Author: Raghava Uppuluri */

#ifndef KINEMATICS
#define KINEMATICS

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/PlanningScene.h>

class Kinematics {
  public:
    Kinematics();  // Initializes MoveGroup API within ROS
    
    void printRobotState(); 
    bool moveToPoseGoal(geometry_msgs::Pose); 
    bool moveToJointGoal(sensor_msgs::JointState); 
    bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan);
    void setVelocityScalingFactor(double);
  
    const std::string PLANNING_GROUP = "panda_arm";
    const double VELOCITY_SCALING_FACTOR = 0.01;
    const double PLANNING_TIME = 2;

    moveit::planning_interface::MoveGroupInterface* move_group_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;
};

#endif