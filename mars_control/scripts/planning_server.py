#!/usr/bin/env python3

import rospy
import ctypes
import rospkg
import os
from typing import List
import actionlib
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from mars_msgs.msg import MoveToAction, MoveToActionGoal
from geometry_msgs.msg import Pose, PoseStamped

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

class PlanningActionServer(object):

    def __init__(self):
        path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')
        os.chdir(path_to_src + "/relaxed_ik_core")
        self._relaxed_ik_lib = ctypes.cdll.LoadLibrary(path_to_src + '/relaxed_ik_core/target/debug/librelaxed_ik_lib.so')
        self._relaxed_ik_lib.solve.restype = Opt

        rospy.init_node('planning_server')
        rospy.wait_for_message("move_group/status", GoalStatusArray)
        self._commander = MoveGroupCommander("panda_arm") 
        self._planning_ac = actionlib.SimpleActionServer('move_to',MoveToAction,execute_cb=self.execute_cb)
    
    def execute_cb(self, goal : MoveToActionGoal):
        self._commander.set_end_effector_link("panda_hand")
        self._commander.set_goal_tolerance(0.001)
        self._commander.compute_cartesian_path(goal.targets,0.01,0.0)
        self._commander.go(wait=True)

        # pose_goals: List[Pose] = goal.targets
        # pos_arr = (ctypes.c_double * (3 * len(pose_goals)))()
        # quat_arr = (ctypes.c_double * (4 * len(pose_goals)))()

        # for i in range(len(pose_goals)):
        #     p = pose_goals[i]
        #     pos_arr[3*i] = p.position.x
        #     pos_arr[3*i+1] = p.position.y
        #     pos_arr[3*i+2] = p.position.z

        #     quat_arr[4*i] = p.orientation.x
        #     quat_arr[4*i+1] = p.orientation.y
        #     quat_arr[4*i+2] = p.orientation.z
        #     quat_arr[4*i+3] = p.orientation.w
        
        # jnts = self._commander.get_current_joint_values()
        # print(jnts)
        # new_state = (ctypes.c_double * 7)(*jnts)
        # self._relaxed_ik_lib.update_joints(new_state)
        # xopt = self._relaxed_ik_lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))

        # relaxed_jnts = [xopt.data[i] for i in range(xopt.length)]
        # print(relaxed_jnts)
        # self._commander.set_joint_value_target(relaxed_jnts)
        # self._commander.plan()
        # self._commander.go()

        self._planning_ac.set_succeeded()

if __name__ == '__main__':

    planning_server = PlanningActionServer()
    rospy.spin() 