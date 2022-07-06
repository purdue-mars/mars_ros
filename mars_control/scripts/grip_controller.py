#!/usr/bin/env python3

import rospy
from wsg_50_common.msg import Cmd, Status

POS_MIN = 3.0 # mm (Point where gelsight fingers touch) 
POS_MAX = 68.0 # mm (Physical limit)
POS_MARGIN = 1.0 # mm
MOVE_SPEED = 50.0 # mm/s (Default 50mm/s)

friction = None
def friction_cb(msg: ):
    global friction
    

cur_pos = None
def pos_cb(msg: Cmd):
    global cur_pos
    cur_pos = msg.width

if __name__ == "__main__":
    rospy.init_node("gripper_controller")
    rate = rospy.Rate(10)

    rospy.Subscriber("/wsg_50_driver/status", Status, pos_cb)
    pub = rospy.Publisher("/wsg_50_driver/goal_position", Cmd, queue_size=1, latch=True)

    # Start by opening gripper
    msg = Cmd()
    msg.pos = POS_MAX
    msg.speed = MOVE_SPEED
    pub.publish(msg)

    opening = True
    while not rospy.is_shutdown():
        if cur_pos: 
            # Alternate between open and close 
            msg = Cmd()
            if opening and cur_pos > (POS_MAX - POS_MARGIN):
                msg.pos = POS_MIN
                opening = False
            elif not opening and cur_pos < (POS_MIN + POS_MARGIN):
                opening = True
                msg.pos = POS_MAX
            else:
                continue
            msg.speed = MOVE_SPEED
            pub.publish(msg)

        rate.sleep()