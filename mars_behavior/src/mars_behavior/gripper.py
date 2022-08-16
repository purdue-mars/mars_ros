import queue

import actionlib
import numpy as np
import rospy
import enum
from franka_gripper.msg import (GraspEpsilon, HomingAction, HomingActionGoal,
                                MoveAction, MoveGoal, StopAction,
                                StopActionGoal)
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty

from gelsight_ros.msg import GelsightFlowStamped
from wsg_50_common.msg import Cmd as WSGCmd, Status as WSGStatus
from wsg_50_common.srv import Conf as WSGConf, Move as WSGMove


class GripperInterface:
    def __init__(self):
        pass

    def home(self):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

    def grasp(self, speed: float, force: float):
        raise NotImplementedError()

    def release(self, width: float, speed: float):
        raise NotImplementedError()

    def move_at(self, vel: float):
        raise NotImplementedError()


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

    def grasp(self, speed=0.02):
        self.grasping_.send_goal(MoveGoal(width=0, speed=speed))
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


class WSGMode(enum.Enum):
    SCRIPT = 1
    POLLING = 2

class WSGInterface(GripperInterface):
    DEFAULT_OPEN_WIDTH: float = 50 # mm
    DEFAULT_SPEED: float = 40  # mm/s
    DEFAULT_FORCE: float = 10.0 # N

    def __init__(self, robot_id: str, mode: WSGMode = WSGMode.POLLING) -> None:
        super().__init__()
        root_id = rospy.get_param("/root_id")

        self.mode = mode
        if mode == WSGMode.SCRIPT:
            rospy.Subscriber(f"/{root_id}/{robot_id}/wsg/moving", Bool, self.is_moving_cb_)
            rospy.Subscriber(f"/{root_id}/{robot_id}/wsg/status", WSGStatus, self.status_cb_)

            self.speed_pub_ = rospy.Publisher(
                f"/{root_id}/{robot_id}/wsg/goal_speed", Float32, queue_size=1, latch=True
            )
            
            self.width_: float = 0.0
            self.width_: float = 0.0
            self.force_: float = 0.0 
        elif mode == WSGMode.POLLING:
            rospy.wait_for_service(f'/{root_id}/{robot_id}/wsg/grasp')
            self.grasp_srv_ = rospy.ServiceProxy(f'/{root_id}/{robot_id}/wsg/grasp', WSGMove)

            rospy.wait_for_service(f'/{root_id}/{robot_id}/wsg/release')
            self.release_srv_ = rospy.ServiceProxy(f'/{root_id}/{robot_id}/wsg/release', WSGMove)

            rospy.wait_for_service(f'/{root_id}/{robot_id}/wsg/stop')
            self.stop_srv_ = rospy.ServiceProxy(f'/{root_id}/{robot_id}/wsg/stop', Empty)

            rospy.wait_for_service(f'/{root_id}/{robot_id}/wsg/set_force')
            self.set_force_srv_ = rospy.ServiceProxy(f'/{root_id}/{robot_id}/wsg/set_force', WSGConf)

            rospy.wait_for_service(f'/{root_id}/{robot_id}/wsg/homing')
            self.homing_srv_ = rospy.ServiceProxy(f'/{root_id}/{robot_id}/wsg/homing', Empty)

    def status_cb_(self, msg: WSGStatus):
        self.width_ = msg.width
        self.speed_ = msg.speed
        self.force_ = msg.force

    def is_moving_cb_(self, msg: Bool):
        self.is_moving_ = msg.data

    def home(self):
        if self.mode != WSGMode.POLLING:
            rospy.logfatal(f"Cannot home in mode {self.mode}")

        self.homing_()
        self.grasping_.publish(WSGInterface.DEFAULT_SPEED)
        rospy.sleep(rospy.Duration(2))

    def stop(self):
        if self.mode == WSGMode.POLLING:
            self.stop_srv_()
        elif self.mode == WSGMode.SCRIPT:
            self.speed_pub_.publish(Float32(0.0))

    def move_at(self, speed: float = None):
        speed = (speed or self.DEFAULT_SPEED)
        if self.mode == WSGMode.POLLING:
            rospy.logfatal(f"Cannot move speed in mode {self.mode}")
        elif self.mode == WSGMode.SCRIPT:
            self.speed_pub_.publish(Float32(0.0))

    def grasp(self, speed: float = None, force: float = None):
        speed = (speed or self.DEFAULT_SPEED)
        force = (force or self.DEFAULT_FORCE)
        if self.mode == WSGMode.POLLING:
            self.set_force_srv_(force)
            self.grasp_srv_(0.0, speed)
        elif self.mode == WSGMode.SCRIPT:
            rate = rospy.Rate(30)
            while self.force_ < force and not rospy.is_shutdown():
                self.speed_pub_.publish(Float32(-speed))
                rate.sleep()
            self.speed_pub_.publish(Float32(0.0)) 
        rospy.loginfo("GRASP COMPLETE!")

    def release(self, width: float = None, speed: float = None):
        width = (width or self.DEFAULT_OPEN_WIDTH)
        speed = (speed or self.DEFAULT_SPEED)
        if self.mode == WSGMode.POLLING:
            self.release_srv_(width, speed)
        elif self.mode == WSGMode.SCRIPT:
            rate = rospy.Rate(30)
            while self.width_ < width and not rospy.is_shutdown():
                self.speed_pub_.publish(Float32(speed))
                rate.sleep()
            self.speed_pub_.publish(Float32(0.0)) 
        rospy.loginfo("RELEASE COMPLETE!")