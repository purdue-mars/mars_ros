#! /usr/bin/env python

import rospy
from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from mars_msgs.msg import MoveToAction, MoveToActionGoal
from mars_msgs.msg import ICPMeshTF
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import MoveAction, StopAction, HomingAction, HomingActionGoal

import tf

class CableInsertionNode:

    def __init__(self) -> None:
        self.move = actionlib.SimpleActionClient('move_to', MoveToAction)
        self.grasp = actionlib.SimpleActionClient('/franka_gripper/move', MoveToAction)
        self.homing = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
        self.stop = actionlib.SimpleActionClient('/franka_gripper/stop', StopAction)

        self.gelsight_sub = rospy.Subscriber('/grasped',10,self.gelsight_cb)
        self.finger_offset = 0.094 + 0.081
        self.grasp_height_offset = 0.2

        self.icp = rospy.ServiceProxy('icp_mesh_tf',ICPMeshTF) 
        self.tf_listener = tf.TransformListener()
        rospy.wait_for_service('add_two_ints')
        self.move.wait_for_server()
        self.grasp.wait_for_server()
        self.homing.wait_for_server()
        self.stop.wait_for_server()
        home_goal = HomingActionGoal()
        self.homing.send_goal(home_goal)
        self.homing.wait_for_result()

    def gelsight_cb(msg):
        pass

    def get_tf(self,frame):
        base_frame = rospy.get_param("/base_frame")
        try:
            (trans,rot) = self.tf_listener.lookupTransform(base_frame, frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
           return 

        pose = PoseStamped()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        return pose

    def get_object_pose(self, mesh_name):
        rospy.set_param('detect_class_names',)
        resp = self.icp(mesh_name)
        rospy.sleep(3.0)
        pose = self.get_tf(mesh_name + '_frame')
        return pose
    
    def to_finger_frame(self,pose):
        pose.pose.position.z += self.finger_offset 

    def cable_pick(self):
        # Get pose of object
        self.move.wait_for_server()
        rospy.info("Action server started, sending goal.");
        goal = MoveToActionGoal()
        cable_male_pose = self.get_object_pose('cable_male')
        self.to_finger_frame(cable_male_pose)
        goal.target = cable_male_pose
        goal.target.pose.position.z += self.grasp_height_offset 
        self.grasp_pose = goal.target 
        self.move.send_goal(goal)
        self.move.wait_for_result()

        pose = self.get_tf('')
        goal.target = cable_male_pose
        self.grasp_pose = goal.target 
        self.move.send_goal(goal)
        self.move.wait_for_result()

    def cable_insert():
        pass


if __name__ == '__main__':
    rospy.init_node('cable_insertion_node')