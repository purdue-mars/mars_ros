from collections import defaultdict
import rospy
from copy import deepcopy
import numpy as np

import tf
from typing import Dict, List
import actionlib
from moveit_commander import MoveGroupCommander
from mars_msgs.msg import MoveToAction, MoveToGoal
from geometry_msgs.msg import (Pose, PoseStamped)
import ros_numpy
from mars_behavior.pose_utils import get_transform

class ArmTF:
    def __init__(self, robot_id : str = 'panda') -> None:
        self.robot_id_ = robot_id
        self.eef_frame_: str = f'{robot_id}_link8' 
        self.grasp_frame_: str = f'{robot_id}_gelsight_pad' 
        self.tf_listener = tf.TransformListener()

    def grasp_to_eef_frame(self,T_GcurGnew : PoseStamped) -> PoseStamped:
        T_EG = get_transform(self.eef_frame_,self.grasp_frame_,self.tf_listener) # E - end effector, G - grasp frame
        if T_EG is None: return
        T_EG: np.ndarray = ros_numpy.numpify(T_EG)
        T_Gcur_Gnew: np.ndarray = ros_numpy.numpify(T_GcurGnew.pose)
        new_pose = ros_numpy.msgify(Pose, np.matmul(np.linalg.inv(T_EG),np.matmul(T_Gcur_Gnew,T_EG)))
        new_pose = PoseStamped(pose=new_pose)
        new_pose.header.frame_id = self.eef_frame_
        return new_pose 

    def grasp_pose_from_object(self,T_XobjXgrasp : Pose, object_frame) -> PoseStamped:
        T_XobjGcur = get_transform(object_frame, self.grasp_frame_, self.tf_listener)
        assert T_XobjGcur is not None 
        T_XobjGcur: np.ndarray = ros_numpy.numpify(T_XobjGcur)
        T_XobjXgrasp: np.ndarray = ros_numpy.numpify(T_XobjXgrasp)
        new_pose = ros_numpy.msgify(Pose, np.matmul(T_XobjGcur,T_XobjXgrasp))
        new_pose = PoseStamped(pose=new_pose)
        new_pose.header.frame_id = self.grasp_frame_
        return new_pose 
    
    @property 
    def ee_frame(self):
        return self.eef_frame_
    @property
    def id(self):
        return self.robot_id_
    @property 
    def grasp_frame(self):
        return self.grasp_frame_


class ArmInterface:

    planning_goals_: Dict[str,List[Pose]] = defaultdict(list) # key is robot_id 

    def __init__(self, planning_group, base_frame) -> None:
        self.move = actionlib.SimpleActionClient('move_to', MoveToAction)
        self.base_frame_ = base_frame
        self.planning_group_= planning_group
        self.tf_listener_ = tf.TransformListener()
        rospy.loginfo('waiting for move_to server')
        self.move.wait_for_server()
        self.commander = MoveGroupCommander(self.planning_group_)

    def execute_planned_goals(self,arm_tf : ArmTF):
        goal = MoveToGoal(targets=self.planning_goals_[arm_tf.id],
                            planning_group=arm_tf.id + '_arm', 
                            ee_frame=arm_tf.ee_frame, 
                            base_frame=self.base_frame_)
        self.move.send_goal(goal)
        self.move.wait_for_result()
        self.planning_goals_[arm_tf.id] = []

    def go_to(self,named_target='ready') -> None:
        # move to ready pose
        self.commander.set_named_target(named_target)
        self.commander.go(wait=True)
    
    def to_base(self, pose : PoseStamped):
        try:
            pose = self.tf_listener_.transformPose(self.base_frame_, pose)
        except Exception() as e:
            print(e)
            return
        return pose
    
    def add_goal(self, goal : PoseStamped, arm_tf : ArmTF) -> None:
        g = deepcopy(goal)
        if goal.header.frame_id != self.base_frame_:
            g = self.to_base(g)
        self.planning_goals_[arm_tf.id].append(g.pose)

    @property 
    def base_frame(self):
        return self.base_frame_

class ControllerInterface:
    def __init__(self):
        pass