#include <mars_control/planning_server.h>

PlanningServer::PlanningServer() : as_(nh_, SERVER_NAME, boost::bind(&PlanningServer::execute, this, _1), false) {
  ros::param::param<std::string>("planning_group",planning_group_, "panda_arm");
  ros::param::param<std::string>("ee_link",ee_link_, "panda_hand");
  ros::param::param<std::string>("bas_e_link",base_link_, "panda_link0");
  as_.start();
} 

void PlanningServer::execute(const mars_msgs::MoveToGoalConstPtr &goal)
{
  moveit::planning_interface::MoveGroupInterface move_group_interface(planning_group_);
  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(planning_group_);

  move_group_interface.setEndEffectorLink(ee_link_);
  move_group_interface.setPoseReferenceFrame(base_link_);

  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());

  std::vector<geometry_msgs::Pose> waypoints;
  for(int i = 0; i < goal->targets.size(); i++) {
    waypoints.push_back(goal->targets[i]);
  }
  moveit_msgs::RobotTrajectory trajectory;
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

  move_group_interface.execute(trajectory);

  as_.setSucceeded();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_server");
  PlanningServer server;
  ros::spin();
  return 0;
}