#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.init_node("move_to")
    rospy.wait_for_message("move_group/status", GoalStatusArray)
    commander = MoveGroupCommander("panda_arm")
    commander.set_named_target(rospy.get_param("pose_name"))
    commander.go(wait=True)
