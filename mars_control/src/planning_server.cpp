#include <mars_msgs/MoveToAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  // start_state.setFromIK(joint_model_group, goal->start);
  move_group_interface.setStartState(start_state);

  move_group_interface.setPoseTarget(goal->target);
  move_group_interface.setPlanningTime(10.0);

  // moveit_msgs::OrientationConstraint ocm;
  // ocm.link_name = "panda_link7";
  // ocm.header.frame_id = "panda_link0";
  // ocm.orientation.w = 1.0;
  // ocm.absolute_x_axis_tolerance = 0.1;
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // moveit_msgs::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  // move_group_interface.setPathConstraints(test_constraints);

  moveit::planning_interface::MoveGroupInterface::Plan m_plan;
  bool success = (move_group_interface.plan(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.publishAxisLabeled(goal->target, "goal");
  visual_tools.publishTrajectoryLine(m_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("next step");

  move_group_interface.execute(m_plan);

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