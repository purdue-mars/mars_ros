from collections import defaultdict
import rospy
from copy import deepcopy
import numpy as np

import tf
from typing import Dict, List
import actionlib
from moveit_commander import MoveGroupCommander
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers
from controller_manager_msgs.msg import ControllerState
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
    
    
class ControlTaskInterface:
    cur_task: str = 'moveit'

    def __init__(self, task_controller_dict : Dict[str,str]) -> None:
        rospy.wait_for_service('controller_manager/switch_controller')
        rospy.wait_for_service('controller_manager/load_controller')

        self.switch_controller_ = rospy.ServiceProxy(
            'controller_manager/switch_controller', SwitchController)
        self.load_controller_ = rospy.ServiceProxy(
            'controller_manager/load_controller', LoadController)
        self.list_controllers_ = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        
        self.task_controller_dict_ = task_controller_dict
        controller_list: List[ControllerState] = self.list_controllers_().controller
        for key in task_controller_dict:
            name = task_controller_dict[key] 
            controller_names = [c.name for c in controller_list]
            if not name in controller_names:
                self.load_controller_(name)


    def run_task(self, task : str):
        rospy.loginfo(f"RUNNING TASK: {task}")
        if not task in self.task_controller_dict_:
            rospy.logerr(f'Invalid task: {task}')
            rospy.loginfo(f'Valid tasks: {self.task_controller_dict_.keys()}')
            return False
        task_controller = self.task_controller_dict_[task]
        cur_controller = self.task_controller_dict_[self.cur_task]
        self.switch_controller_(start_controllers=[task_controller],stop_controllers=[cur_controller],strictness=2)
        self.cur_task = task
        return True

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
        task_controller_dict: Dict[str,str] = rospy.get_param('/task_controller_dict')
        self.task_interface = ControlTaskInterface(task_controller_dict)

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
    
    def run_task(self,task,callback):
        success = self.task_interface.run_task(task)
        if success:
            callback()
        self.task_interface.run_task('moveit')

    @property 
    def base_frame(self):
        return self.base_frame_

class ControllerInterface:
    def __init__(self):
        pass