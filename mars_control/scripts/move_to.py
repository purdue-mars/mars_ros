#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.init_node("move_to")
    rospy.wait_for_message("move_group/status", GoalStatusArray)
    commander = MoveGroupCommander("panda_arm")

    pose_name = rospy.get_param("pose_name")

    pose_target = Pose()
    pose_target.position.x = rospy.get_param(f"{pose_name}/position/x")
    pose_target.position.y = rospy.get_param(f"{pose_name}/position/y")
    pose_target.position.z = rospy.get_param(f"{pose_name}/position/z")
    pose_target.orientation.x = rospy.get_param(f"{pose_name}/orientation/x")
    pose_target.orientation.y = rospy.get_param(f"{pose_name}/orientation/y")
    pose_target.orientation.z = rospy.get_param(f"{pose_name}/orientation/z")
    pose_target.orientation.w = rospy.get_param(f"{pose_name}/orientation/w")

    waypoints = []
    waypoints.append(pose_target)

    plan, fraction = commander.compute_cartesian_path(waypoints, 0.01, 0.0)
    commander.execute(plan, wait=True)
    # commander.set_pose_target(pose_target)
    # commander.go(wait=True)
    commander.stop()
    rospy.signal_shutdown(f"move_to: Successfully moved to '{pose_name}'")
