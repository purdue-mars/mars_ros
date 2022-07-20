#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from gelsight_ros.msg import GelsightFlowStamped
from controller_manager_msgs.srv import  ListControllers, LoadController, SwitchController, ListControllerTypes, UnloadController
from wsg_50_common.msg import Cmd as GripperCmd, Status as GripperStatus
from wsg_50_common.srv import Move as GripperMove, Conf as GripperConf
from enum import Enum
import numpy as np
from copy import deepcopy

class State(Enum):
    START = 1 
    FOLLOW = 2
    STOP = 3
    GRIP = 4
    RESET = 5
    OPEN = 6


class ScriptGripper:

    # Gripper constants
    ideal_friction = 5.0
    max_speed = 20.0
    max_force = 8.0
    open_pos = 15.0
    close_pos = 2.0
    friction_p_gain = -0.2
    pos_p_gain = 0.2

    def __init__(self, name: str):

        # Setup member variables
        self.pos = 0.0 
        self.speed = 0.0
        self.force = 0.0

        # Setup gripper topics
        rospy.Subscriber(f"{name}/wsg_50_driver/status", GripperStatus, self.status_cb)
        self.speed_pub = rospy.Publisher(f"{name}/wsg_50_driver/goal_speed", Float32, queue_size=1, latch=True)

    def status_cb(self, msg: GripperStatus):
        self.pos = msg.width
        self.speed = msg.speed
        self.force = (self.force * 0.75) + (msg.force * 0.25)

    def open(self):
        while self.pos < self.open_pos:
            print(self.pos, self.open_pos)
            self.speed_pub.publish(Float32(self.max_speed))
            rospy.sleep(0.05)
        self.speed_pub.publish(Float32(0.0))

    def grasp_by_friction(self, friction):
        error = self.ideal_friction - friction
        if error < 0.0:
            vel = error * self.friction_p_gain / 2.5
        else:     
            vel = error * self.friction_p_gain
        vel = max(-self.max_speed, min(self.max_speed, vel))

        self.speed_pub.publish(Float32(vel))

    def close(self):
        while self.force < self.max_force and self.pos > self.close_pos:
            self.speed_pub.publish(Float32(-self.max_speed))
            rospy.sleep(0.05)
        self.speed_pub.publish(Float32(0.0))

class PollingGripper:

    # Gripper constants
    max_speed = 20.0
    max_force = 25.0
    open_pos = 4.0
    close_pos = 0.0

    def __init__(self, name: str):
        # Setup gripper topics
        rospy.wait_for_service(f'{name}/wsg_50_driver/grasp') 
        self.grasp_srv = rospy.ServiceProxy(f'{name}/wsg_50_driver/grasp', GripperMove)

        rospy.wait_for_service(f'{name}/wsg_50_driver/release') 
        self.release_srv = rospy.ServiceProxy(f'{name}/wsg_50_driver/release', GripperMove)

        rospy.wait_for_service(f'{name}/wsg_50_driver/set_force') 
        set_force_srv = rospy.ServiceProxy(f'{name}/wsg_50_driver/set_force', GripperConf)
        set_force_srv(self.max_force)

    def open(self):
        self.release_srv(self.open_pos, self.max_speed)

    def grasp_by_friction(self, friction):
        raise NotImplementedError()

    def close(self):
        self.grasp_srv(self.close_pos, self.max_speed)


class FollowCableRoutine:

    # Gelsight constants
    cable_y_range = 0.20

    def __init__(self):
        rospy.init_node("follow_cable")

        # Create controller manager services
        rospy.wait_for_service('controller_manager/list_controllers') 
        self.list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)

        rospy.wait_for_service('controller_manager/load_controller') 
        self.load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
        self.load_controller("cable_data_collector")

        rospy.wait_for_service('controller_manager/switch_controller') 
        self.switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

        rospy.wait_for_service('controller_manager/list_controller_types') 
        self.list_types = rospy.ServiceProxy('controller_manager/list_controller_types', ListControllerTypes)

        rospy.wait_for_service('/cable_data_collector/slow_controller')
        self.slow_controller = rospy.ServiceProxy('/cable_data_collector/slow_controller', Empty)

        # Set internal variables
        self.state = State.START
        self.friction = 0.0
        self.cable_y = 0.0
        self.follow_started = None

        # Setup gelsight topics
        rospy.Subscriber("gelsight/flow", GelsightFlowStamped, self.gelsight_flow_cb)
        rospy.Subscriber("gelsight/pose", PoseStamped, self.gelsight_pose_cb)

        # Setup grippers
        self.anan_gripper = ScriptGripper("anan")
        self.panpan_gripper = PollingGripper("panpan")
        self.anan_gripper.open()
        self.panpan_gripper.close()
        
        # Move to starting position for cable following
        rospy.wait_for_message("move_group/status", GoalStatusArray)
        self.commander = MoveGroupCommander("panda_arm")
        self.commander.set_max_velocity_scaling_factor(0.1)
        self.commander.set_max_acceleration_scaling_factor(0.1)
        self.commander.set_named_target("cable-following")
        self.commander.go(wait=True)

        self.reset_pose = deepcopy(self.commander.get_current_pose().pose)

        # Start main loop
        while not rospy.is_shutdown():
            try:
                self.step()
                rospy.sleep(0.03)
            except rospy.ROSInterruptException: 
               break

    def gelsight_flow_cb(self, msg: GelsightFlowStamped):
        ref = np.array(msg.ref_markers.data)
        ref = ref.reshape((ref.shape[0]//2, 2))
        cur = np.array(msg.cur_markers.data)
        cur = cur.reshape((cur.shape[0]//2, 2))
        self.friction = np.linalg.norm(np.mean((cur - ref)*10.0, axis=0))

    def gelsight_pose_cb(self, msg: PoseStamped):
        self.cable_y = (self.cable_y * 0.75) + (msg.pose.position.y * 0.25)

    def step(self):
        if self.state == State.START:
            # Wait for cable to be gripped
            self.anan_gripper.close()
            self.switch_controller(
                start_controllers=["cable_data_collector"],
                stop_controllers=["position_joint_trajectory_controller"],
                strictness=2)
            self.state = State.FOLLOW
            self.follow_started = rospy.get_time()

        elif self.state == State.FOLLOW: 
            self.anan_gripper.grasp_by_friction(self.friction) 
            
            if abs(self.cable_y) > self.cable_y_range or rospy.get_time() - self.follow_started > 15.0:
                self.state = State.GRIP
                
                self.slow_controller() 
                rospy.sleep(1.5)
                self.switch_controller(
                    start_controllers=["position_joint_trajectory_controller"],
                    stop_controllers=["cable_data_collector"],
                    strictness=2
                )

        elif self.state == State.GRIP: 
            # Grasp cable and open holder 
            self.anan_gripper.close()
            rospy.sleep(0.1)
            self.panpan_gripper.open()

            # Move cable back to holder
            self.move_start = rospy.get_time()
            self.commander.stop()

            rospy.sleep(0.1)
            pose = deepcopy(self.reset_pose)
            pose.position.y -= self.cable_y / 10.0
            waypoints = [pose]
            (plan, fraction) = self.commander.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.001,        # eef_step
                0.0)

            ref_state = self.commander.get_current_state()
            retimed_plan = self.commander.retime_trajectory(
                ref_state,
                plan,
                velocity_scaling_factor=0.1,
                acceleration_scaling_factor=0.1,
            )
            self.commander.execute(retimed_plan, wait=True)

            # Close holder and release cable for re-adjustment
            self.panpan_gripper.close() 
            rospy.sleep(0.1)
            self.anan_gripper.open()

            # Adjust cable follower back to start
            waypoints = [self.reset_pose]
            (plan, fraction) = self.commander.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.001,        # eef_step
                0.0)
            ref_state = self.commander.get_current_state()
            retimed_plan = self.commander.retime_trajectory(
                ref_state,
                plan,
                velocity_scaling_factor=0.1,
                acceleration_scaling_factor=0.05,
            )

            self.commander.execute(retimed_plan, wait=True)

            self.state = State.START

if __name__ == "__main__":
    FollowCableRoutine()