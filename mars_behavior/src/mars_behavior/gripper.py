import queue
import rospy
import numpy as np
import actionlib
from gelsight_ros.msg import GelsightFlowStamped
from franka_gripper.msg import (GraspEpsilon, HomingAction, HomingActionGoal,
                                MoveAction, MoveGoal, StopAction,
                                StopActionGoal)
from std_srvs.srv import Empty                               
from std_msgs.msg import Float32, Bool
from wsg_50_common.srv import Move
from wsg_50_common.msg import Cmd, Status 

class GripperInterface:
    def __init__(self) -> None:
        pass
    def home(self):
        raise NotImplementedError

    def grasp(self) -> None:
        raise NotImplementedError
    
class PandaInterface(GripperInterface):
    grasped_flow_norm_ = 0.0
    GRASP_THRESHOLD_ = 20

    def __init__(self) -> None:
        super().__init__()
        self.flow = rospy.Subscriber('perception/flow',GelsightFlowStamped, self.flow_cb_)
        self.grasping = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
        self.homing = actionlib.SimpleActionClient('franka_gripper/homing', HomingAction)
        self.stop = actionlib.SimpleActionClient('franka_gripper/stop', StopAction)
        rospy.loginfo('waiting for grasping server')
        self.grasping.wait_for_server()
        rospy.loginfo('waiting for homing server')
        self.homing.wait_for_server()
        rospy.loginfo('waiting for stop server')
        self.stop.wait_for_server()

    def flow_cb_(self,msg: GelsightFlowStamped):
        flow_np = np.array(msg.cur_markers.data) - np.array(msg.ref_markers.data)
        flow_np = np.reshape(flow_np, (msg.cur_markers.m,msg.cur_markers.n,2))
        self.grasped_flow_norm_ = np.linalg.norm(flow_np)
    
    def home(self):
        home_goal = HomingActionGoal()
        self.homing.send_goal(home_goal)
        #self.homing.wait_for_result()

    def grasp(self):
        self.grasping.send_goal(MoveGoal(width=0,speed=0.02))
        rate = rospy.Rate(30)
        while self.grasped_flow_norm_ < self.GRASP_THRESHOLD_:
            rospy.loginfo("flow_norm: %4f",self.grasped_flow_norm_)
            if self.grasping.get_state() != actionlib.SimpleGoalState.PENDING and self.grasping.get_state() != actionlib.SimpleGoalState.ACTIVE:
                rospy.loginfo("MISSED DETECTION!")
                self.stop.send_goal(StopActionGoal())
                break
            if rospy.is_shutdown():
                rospy.loginfo("SHUTDOWN!")
                self.stop.send_goal(StopActionGoal())
                raise Exception("GRASP INCOMPLETE!") 
            rate.sleep()
        self.grasping.cancel_all_goals()
        self.stop.send_goal(StopActionGoal())
        rospy.loginfo("GRASP COMPLETE!")

class WSGInterface(GripperInterface):
    grasped_flow_norm_: float = 0.0
    GRASP_THRESHOLD_: float = 20
    DEFAULT_SPEED: float = 40 # mm/s

    def __init__(self,ns='') -> None:
        super().__init__()
        ns = ns + '/'
        self.flow = rospy.Subscriber('/perception/flow',GelsightFlowStamped, self.flow_cb_)
        self.grasping_ = rospy.Publisher(f'{ns}wsg_50_driver/goal_speed', Float32, queue_size=1)
        self.is_moving_ = rospy.Subscriber(f'{ns}wsg_50_driver/moving', Bool,self.is_moving_cb_)
        self.width_: float = 0.0

    def flow_cb_(self,msg: GelsightFlowStamped):
        flow_np = np.array(msg.cur_markers.data) - np.array(msg.ref_markers.data)
        flow_np = np.reshape(flow_np, (msg.cur_markers.m,msg.cur_markers.n,2))
        self.grasped_flow_norm_ = np.linalg.norm(flow_np)
    
    def is_moving_cb_(self, msg: Bool):
        self.is_moving_ = msg.data
    
    def home(self):
        self.grasping_.publish(self.DEFAULT_SPEED)
        rospy.sleep(rospy.Duration(3))
    
    def stop(self):
        self.grasping_.publish(0.0)

    def grasp(self):
        self.grasping_.publish(-self.DEFAULT_SPEED)
        rospy.sleep(rospy.Duration(2))
        rate = rospy.Rate(30)
        while self.is_moving_ and self.grasped_flow_norm_ < self.GRASP_THRESHOLD_ and not rospy.is_shutdown():
            rospy.loginfo("flow_norm: %4f",self.grasped_flow_norm_)
            rate.sleep()
        self.stop()
        rospy.loginfo("GRASP COMPLETE!")
