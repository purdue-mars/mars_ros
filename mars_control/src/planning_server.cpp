#include <mars_msgs/MoveToAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <mars_control/kinematics.h>

typedef actionlib::SimpleActionServer<mars_msgs::MoveToAction> Server;
static const std::string PLANNING_GROUP = "panda_arm";

void execute(const mars_msgs::MoveToGoalConstPtr &goal, Server *as)
{
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group_interface.setGoalTolerance(0.01);
    move_group_interface.setMaxVelocityScalingFactor(0.2);

  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());

  std::vector<geometry_msgs::Pose> waypoints;
  for(int i = 0; i < goal->targets.size(); i++) {
    waypoints.push_back(goal->targets[i]);
  }
  moveit_msgs::RobotTrajectory trajectory;
  move_group_interface.setMaxVelocityScalingFactor(0.01);
  move_group_interface.setMaxAccelerationScalingFactor(0.01);
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                                              0.001, // eef_step
                                                              0.00,  // jump_threshold
                                                              trajectory);

  // robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), PLANNING_GROUP);
  // rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);
  
  // trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // bool time_stamp_success = iptp.computeTimeStamps(rt);
  // ROS_INFO("Computed time stamp %s", time_stamp_success ? "SUCCEEDED":"FAILED");

  // rt.getRobotTrajectoryMsg(trajectory);

  // visual_tools.publishAxisLabeled(goal->target, "goal");
  // visual_tools.publishTrajectoryLine(trajectory, joint_model_group);

  move_group_interface.execute(trajectory);

  as->setSucceeded();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_server");
  ros::NodeHandle n;

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  Server server(n, "move_to", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}