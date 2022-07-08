#!/usr/bin/env python3

import rospy
from wsg_50_common.msg import Cmd, Status
from std_msgs.msg import Bool, Float32
from gelsight_ros.msg import GelsightFlowStamped
import numpy as np

POS_MIN = 4.0
POS_MAX = 50.0
MAX_SPEED = 10.0
FRICTION_SETPOINT = 5.0
P_GAIN = -0.4
D_GAIN = -1.0
RATE = 30.0

friction = None
def friction_cb(msg):
    global friction
    ref = np.array(msg.ref_markers.data)
    ref = ref.reshape((ref.shape[0]//2, 2))
    cur = np.array(msg.cur_markers.data)
    cur = cur.reshape((cur.shape[0]//2, 2))
    friction = np.linalg.norm(np.mean(np.absolute(cur - ref)*10.0, axis=0))

cur_pos = None
def pos_cb(msg: Cmd):
    global cur_pos
    cur_pos = msg.width

if __name__ == "__main__":
    rospy.init_node("gripper_controller")
    rate = rospy.Rate(RATE)
    rospy.Subscriber("/wsg_50_driver/status", Status, pos_cb)
    rospy.Subscriber("/flow", GelsightFlowStamped, friction_cb)
    pub = rospy.Publisher("/wsg_50_driver/goal_speed", Float32, queue_size=1, latch=True)

    e_pub = rospy.Publisher("/data/e_pub", Float32, queue_size=3)
    friction_pub = rospy.Publisher("/data/friction", Float32, queue_size=3)
    p_com_pub = rospy.Publisher("/data/p_com", Float32, queue_size=3)
    d_com_pub = rospy.Publisher("/data/d_com", Float32, queue_size=3)
    vel_pub = rospy.Publisher("/data/vel", Float32, queue_size=3)

    last_e = 0.0
    last_friction = 0.0
    dt = 1/RATE
    while not rospy.is_shutdown():
        if friction is not None:
            friction = (friction * 0.25) + (last_friction * 0.75)
            last_friction = friction
            e = FRICTION_SETPOINT - friction
            p_com = e * P_GAIN
            d_com = (e-last_e) * D_GAIN
            vel = p_com + d_com
            last_e = e
            
            if cur_pos and cur_pos > POS_MAX:
            # If max pos exceeded, limit to neg dir
                vel = max(-MAX_SPEED, min(0.0, vel))
            elif cur_pos and cur_pos < POS_MIN:
            # If min pos exceeded, limit to pos dir
                vel = max(0.0, min(MAX_SPEED, vel)) 
            else:
            # If within pos limits, only limit speed
                vel = max(-MAX_SPEED, min(MAX_SPEED, vel))

            e_pub.publish(Float32(e))
            friction_pub.publish(Float32(friction))
            p_com_pub.publish(Float32(p_com))
            d_com_pub.publish(Float32(d_com))
            vel_pub.publish(Float32(vel))

            msg = Float32(vel)
            pub.publish(msg)
        rate.sleep()