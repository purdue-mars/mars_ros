#include <mars_control/planning_server.h>

PlanningServer::PlanningServer() : as_(nh_, SERVER_NAME, boost::bind(&PlanningServer::execute, this, _1), false)
{
  ros::param::param<std::string>("/planning_group", global_planning_group_, DEFAULT_PLANNING_GROUP);
  as_.start();
}

void copy_vector(std::vector<double> &from, std::vector<double> &to, int at)
{
  std::vector<double> temp = to;
  for (int i = 0; i < from.size(); i++)
  {
    if (at + i >= temp.size() || at + i < 0)
    {
      throw "Invalid sizes and/or index!";
    }
    else
    {
      temp[at + i] = from[i];
    }
  }
  from = temp;
}

void PlanningServer::set_combined_traj(moveit_msgs::RobotTrajectory &traj, std::string planning_group)
{

  moveit::planning_interface::MoveGroupInterface combined_mgi(global_planning_group_);
  moveit::core::RobotModelConstPtr kinematic_model = combined_mgi.getRobotModel();
  const moveit::core::JointModelGroup *jm_group = kinematic_model->getJointModelGroup(global_planning_group_);
  moveit::core::RobotStatePtr kinematic_state = combined_mgi.getCurrentState();

  std::vector<std::string> comb_jnt_names = combined_mgi.getJointNames();
  std::string j0_name = traj.joint_trajectory.joint_names[0];
  std::vector<std::string>::iterator it = std::find(comb_jnt_names.begin(), comb_jnt_names.end(), j0_name);
  int set_id = std::distance(comb_jnt_names.begin(), it);

  std::vector<double> jnt_pos(comb_jnt_names.size());
  std::vector<double> jnt_vel(comb_jnt_names.size());
  std::vector<double> jnt_acc(comb_jnt_names.size());
  kinematic_state->copyJointGroupPositions(jm_group, jnt_pos);

  for (int i = 0; i < traj.joint_trajectory.points.size(); i++)
  {
    copy_vector(traj.joint_trajectory.points[i].positions, jnt_pos, set_id);
    copy_vector(traj.joint_trajectory.points[i].velocities, jnt_vel, set_id);
    copy_vector(traj.joint_trajectory.points[i].accelerations, jnt_acc, set_id);
  }


  traj.joint_trajectory.joint_names = comb_jnt_names;
}

void PlanningServer::execute(const mars_msgs::MoveToGoalConstPtr &goal)
{
  moveit::planning_interface::MoveGroupInterface mgi(goal->planning_group);
  moveit_msgs::RobotTrajectory trajectory;
  mgi.setEndEffectorLink(goal->ee_frame);
  mgi.setPoseReferenceFrame(goal->base_frame);
  double fraction = mgi.computeCartesianPath(goal->targets,
                                             0.001, // eef_step
                                             0.00,  // jump_threshold
                                             trajectory);

  if (fraction == -1)
  {
    as_.setAborted();
    return;
  }
  else
  {

  robot_trajectory::RobotTrajectory rt(mgi.getCurrentState()->getRobotModel(), goal->planning_group);
  rt.setRobotTrajectoryMsg(*mgi.getCurrentState(), trajectory);
  
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool time_stamp_success = iptp.computeTimeStamps(rt,0.3,0.3);
  ROS_INFO("Computed time stamp %s", time_stamp_success ? "SUCCEEDED":"FAILED");

  rt.getRobotTrajectoryMsg(trajectory);

    if (global_planning_group_ == goal->planning_group)
    {
      mgi.execute(trajectory);
    }
    else
    {
      moveit::planning_interface::MoveGroupInterface combined_mgi(global_planning_group_);
      set_combined_traj(trajectory, goal->planning_group);
      combined_mgi.execute(trajectory);
    }
    as_.setSucceeded();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_server");
  PlanningServer server;
  ros::spin();
  return 0;
}