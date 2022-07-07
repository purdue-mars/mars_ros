#!/usr/bin/env python3

import rospy
from wsg_50_common.msg import Cmd, Status
from std_msgs.msg import Bool
import numpy as np

MOVE_SPEED = 50.0 # mm/s (Default 50mm/s)
FRICTION_MIN = 5.0 # avg marker px diff
FRICTION_MAX = 10.0
MOVE_DX = 2.0 # mm

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

move_done = False
def move_done_cb(msg):
    global move_done
    move_done = msg.data

if __name__ == "__main__":
    rospy.init_node("gripper_controller")
    rate = rospy.Rate(10)
    rospy.Subscriber("wsg_50_driver/moving", Bool, move_done_cb) 
    rospy.Subscriber("/wsg_50_driver/status", Status, pos_cb)
    pub = rospy.Publisher("/wsg_50_driver/goal_position", Cmd, queue_size=1, latch=True)

    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        dx = 0.0
        if friction < FRICTION_MIN:
            dx -= MOVE_DX
        elif friction > FRICTION_MAX:
            dx += MOVE_DX

        if dx != 0.0 and cur_pos and move_done:
            msg = Cmd()
            msg.pos = cur_pos + dx
            msg.speed = MOVE_SPEED
            pub.publish(msg)
        rate.sleep()