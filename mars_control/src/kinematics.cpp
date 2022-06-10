/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International *  All rights reserved.
 *
 *********************************************************************/
/* Author: Raghava Uppuluri */

#include <math.h>
#include <ros/ros.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <mars_control/kinematics.h>

Kinematics::Kinematics() { 
  move_group_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();
  robot_state_.reset(new robot_state::RobotState(robot_model_));

  // Set Params 
  move_group_->setMaxVelocityScalingFactor(VELOCITY_SCALING_FACTOR);
  move_group_->setPlanningTime(PLANNING_TIME);

}

void Kinematics::printRobotState() {
  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(PLANNING_GROUP);

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  std::cout << "EEF Name: " << joint_model_group->getEndEffectorName() << std::endl;
  ROS_INFO_NAMED("kinematics-info", "Reference frame: %s", move_group_->getPlanningFrame().c_str());
  ROS_INFO_NAMED("kinematics-info", "End effector link: %s", move_group_->getEndEffectorLink().c_str());
} 

bool Kinematics::moveToPoseGoal(geometry_msgs::Pose goal_pose) {
  move_group_->setPoseTarget(goal_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("kinematics-info", "Planning to pose goal %s", plan_success ? "SUCCESS" : "FAILURE");

  bool move_success = plan_success ? Kinematics::executePlan(plan) : false; 
  ROS_INFO_NAMED("kinematics-info", "Moving to pose goal %s", move_success ? "SUCCESS" : "FAILURE");

  return move_success;
} 

bool Kinematics::moveToJointGoal(sensor_msgs::JointState joint_state) {
  move_group_->setJointValueTarget(joint_state);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("kinematics-info", "Planning to joint goal %s", plan_success ? "SUCCESS" : "FAILURE");

  bool move_success = plan_success ? Kinematics::executePlan(plan) : false; 
  ROS_INFO_NAMED("kinematics-info", "Moving to joint goal %s", move_success ? "SUCCESS" : "FAILURE");
  
  return move_success;
}
 
bool Kinematics::executePlan(moveit::planning_interface::MoveGroupInterface::Plan plan) {
  moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
  robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(), PLANNING_GROUP);
  rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
  
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool time_stamp_success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s", time_stamp_success ? "SUCCEEDED":"FAILED");

  rt.getRobotTrajectoryMsg(trajectory);
  plan.trajectory_ = trajectory;

  if (time_stamp_success) {
    move_group_->execute(plan);
    return true;
  } 
  else {
    return false;
  }
}