#!/usr/bin/env python3

import rospy
from wsg_50_common.msg import Cmd, Status
from std_msgs.msg import Bool, Float32
import numpy as np

POS_MIN = 2.0
POS_MAX = 50.0
MAX_SPEED = 50.0
FRICTION_SETPOINT = 10.0
P_GAIN = 0.1

friction = None
def friction_cb(msg):
    global friction

    ref = np.array(msg.ref_markers.data)
    ref = ref.reshape((ref.shape[0]//2, 2))
    cur = np.array(msg.cur_markers.data)
    cur = cur.reshape((cur.shape[0]//2, 2))

    friction = np.mean(cur - ref, axis=1)

cur_pos = None
def pos_cb(msg: Cmd):
    global cur_pos
    cur_pos = msg.width

if __name__ == "__main__":
    rospy.init_node("gripper_controller")
    rate = rospy.Rate(10)
    rospy.Subscriber("/wsg_50_driver/status", Status, pos_cb)
    pub = rospy.Publisher("/wsg_50_driver/goal_speed", Float32, queue_size=1, latch=True)

    rospy.sleep(1.)

    while not rospy.is_shutdown():
        vel = (FRICTION_SETPOINT - friction) * P_GAIN

        if cur_pos and cur_pos > POS_MAX:
            # If max pos exceeded, limit to neg dir
            vel = max(-MAX_SPEED, min(0.0, vel))
        elif cur_pos and cur_pos < POS_MIN:
            # If min pos exceeded, limit to pos dir
            vel = max(0.0, min(MAX_SPEED, vel)) 
        else:
            # If within pos limits, only limit speed
            vel = max(-MAX_SPEED, min(MAX_SPEED, vel))
        msg = Float32(vel)
        pub.publish(msg)
        rate.sleep()