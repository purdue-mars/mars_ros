import queue

import actionlib
import numpy as np
import rospy
from franka_gripper.msg import (GraspEpsilon, HomingAction, HomingActionGoal,
                                MoveAction, MoveGoal, StopAction,
                                StopActionGoal)
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty

from gelsight_ros.msg import GelsightFlowStamped
from wsg_50_common.msg import Cmd, Status
from wsg_50_common.srv import Home


class GripperInterface:
    def __init__(self):
        pass

    def home(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def grasp(self) -> None:
        raise NotImplementedError


class PandaInterface(GripperInterface):
    grasped_flow_norm_ = 0.0
    GRASP_THRESHOLD_ = 20

    def __init__(self) -> None:
        super().__init__()
        self.flow_ = rospy.Subscriber("flow", GelsightFlowStamped, self.flow_cb_)
        self.grasping_ = actionlib.SimpleActionClient("franka_gripper/move", MoveAction)
        self.homing_ = actionlib.SimpleActionClient(
            "franka_gripper/homing", HomingAction
        )
        self.stopping_ = actionlib.SimpleActionClient("franka_gripper/stop", StopAction)
        rospy.loginfo("waiting for grasping server")
        self.grasping_.wait_for_server()
        rospy.loginfo("waiting for homing server")
        self.homing_.wait_for_server()
        rospy.loginfo("waiting for stop server")
        self.stopping_.wait_for_server()

    def flow_cb_(self, msg: GelsightFlowStamped):
        flow_np = np.array(msg.cur_markers.data) - np.array(msg.ref_markers.data)
        flow_np = np.reshape(flow_np, (msg.cur_markers.m, msg.cur_markers.n, 2))
        self.grasped_flow_norm_ = np.linalg.norm(flow_np)

    def home(self):
        home_goal = HomingActionGoal()
        self.homing_.send_goal(home_goal)
        # self.homing.wait_for_result()

    def stop(self):
        self.stopping_.send_goal(StopActionGoal())
        self.stopping_.wait_for_result()
        self.grasping_.cancel_all_goals()

    def grasp(self):
        self.grasping_.send_goal(MoveGoal(width=0, speed=0.02))
        rate = rospy.Rate(30)
        while self.grasped_flow_norm_ < self.GRASP_THRESHOLD_:
            rospy.loginfo("flow_norm: %4f", self.grasped_flow_norm_)
            if (
                self.grasping_.get_state() != actionlib.SimpleGoalState.PENDING
                and self.grasping_.get_state() != actionlib.SimpleGoalState.ACTIVE
            ):
                rospy.loginfo("MISSED DETECTION!")
                self.stop()
                break
            if rospy.is_shutdown():
                rospy.loginfo("SHUTDOWN!")
                self.stop()
                raise Exception("GRASP INCOMPLETE!")
            rate.sleep()
        self.stop()
        rospy.loginfo("GRASP COMPLETE!")


class WSGInterface(GripperInterface):
    DEFAULT_SPEED: float = 40  # mm/s

    def __init__(self, robot_id: str) -> None:
        super().__init__()
        root_id = rospy.get_param("/root_id")
        self.grasping_ = rospy.Publisher(
            f"/{root_id}/{robot_id}/wsg/goal_speed", Float32, queue_size=1
        )
        self.is_moving_ = rospy.Subscriber(
            f"/{root_id}/{robot_id}/wsg/moving", Bool, self.is_moving_cb_
        )
        self.homing_ = rospy.ServiceProxy(f'/{root_id}/{robot_id}/wsg/homing',Empty)
        self.width_: float = 0.0

    def is_moving_cb_(self, msg: Bool):
        self.is_moving_ = msg.data

    def home(self):
        self.homing_()
        self.grasping_.publish(self.DEFAULT_SPEED)
        rospy.sleep(rospy.Duration(2))

    def stop(self):
        self.grasping_.publish(0.0)

    def grasp(self):
        self.grasping_.publish(-self.DEFAULT_SPEED)
        rospy.sleep(rospy.Duration(2))
        rate = rospy.Rate(30)
        while (
            self.is_moving_
            and not rospy.is_shutdown()
        ):
            rate.sleep()
        self.stop()
        rospy.loginfo("GRASP COMPLETE!")