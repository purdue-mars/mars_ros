from collections import defaultdict
from copy import deepcopy
from typing import Dict, List

import actionlib
import rospy
import tf
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController)
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander

from mars_msgs.msg import MoveToAction, MoveToGoal, MoveToResult

from .arm_tf import ArmTF


class ControlTaskInterface:
    MOVEIT_TASK: str = "moveit"
    cur_task: str = MOVEIT_TASK

    def __init__(self) -> None:
        self.task_controller_dict_ = rospy.get_param("task_controller_dict")

        rospy.wait_for_service("controller_manager/switch_controller")
        rospy.wait_for_service("controller_manager/load_controller")

        self.switch_controller_ = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_controller_ = rospy.ServiceProxy(
            "controller_manager/load_controller", LoadController
        )
        self.list_controllers_ = rospy.ServiceProxy(
            "controller_manager/list_controllers", ListControllers
        )

        controller_list: List[ControllerState] = self.list_controllers_().controller
        for key in self.task_controller_dict_:
            name = self.task_controller_dict_[key]
            controller_names = [c.name for c in controller_list]
            if not name in controller_names:
                self.load_controller_(name)

    def run_task(self, task: str):
        rospy.loginfo(f"RUNNING TASK: {task}")
        if not task in self.task_controller_dict_:
            rospy.logerr(f"Invalid task: {task}")
            rospy.loginfo(f"Valid tasks: {self.task_controller_dict_.keys()}")
            return False
        task_controller = self.task_controller_dict_[task]
        cur_controller = self.task_controller_dict_[self.cur_task]
        self.switch_controller_(
            start_controllers=[task_controller],
            stop_controllers=[cur_controller],
            strictness=2,
        )
        self.cur_task = task
        return True


class ArmInterface:
    planning_goals_: Dict[str, List[Pose]] = defaultdict(list)  # key is robot_id

    def __init__(self):
        self.task_interface_ = ControlTaskInterface()
        self.commander_ = MoveGroupCommander(rospy.get_param("planning_group"))
        self.move_ = actionlib.SimpleActionClient("move_to", MoveToAction)

        self.tf_listener = tf.TransformListener()
        self.robot_ids = rospy.get_param("robot_ids")
        self.arm_tfs_ = {id: ArmTF(self.tf_listener, id) for id in self.robot_ids}
        rospy.loginfo("waiting for move_to server")
        self.move_.wait_for_server()

    def get_arm_tf(self, id: str):
        if not id in self.robot_ids:
            rospy.logerr(f"Invalid robot_id: {id}, must be one of {self.robot_ids}")
            return
        return self.arm_tfs_[id]

    def execute_planned_goals(self, robot_id: str):
        arm_tf = self.get_arm_tf(robot_id)
        goal = MoveToGoal(
            targets=self.planning_goals_[arm_tf.id],
            planning_group=arm_tf.id + "_arm",
            ee_frame=arm_tf.ee_frame,
            base_frame=arm_tf.base_frame,
        )
        self.move_.send_goal(goal)
        self.move_.wait_for_result()
        res: MoveToResult = self.move_.get_result()
        self.planning_goals_[arm_tf.id] = []
        return res.was_success

    def go_to(self, named_target="ready"):
        self.commander_.set_named_target(named_target)
        self.commander_.go(wait=True)

    def add_goal(self, goal: PoseStamped, robot_id: str):
        arm_tf = self.get_arm_tf(robot_id)
        g = deepcopy(goal)
        if goal.header.frame_id != arm_tf.base_frame:
            g = arm_tf.to_base(g)
        self.planning_goals_[arm_tf.id].append(g.pose)

    def run_task(self, task, callback):
        success = self.task_interface_.run_task(task)
        if success:
            callback()
        self.task_interface_.run_task(self.task_interface_.MOVEIT_TASK)
