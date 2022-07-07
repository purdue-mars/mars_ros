#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from wsg_50_common.msg import Cmd

MOVE_SPEED = 50.0 # mm/s (Default 50mm/s)

# TODO: Condense into move_to.py

move_done = False
def move_done_cb(msg):
    global move_done
    move_done = msg.data

if __name__ == "__main__":
    rospy.init_node("gripper_move_to")
    rospy.Subscriber("wsg_50_driver/moving", Bool, move_done_cb) 
    pub = rospy.Publisher("wsg_50_driver/goal_position", Cmd, queue_size=1, latch=True)

    # Wait for wsg pkg
    rospy.sleep(1.)

    msg = Cmd()
    msg.pos = rospy.get_param(rospy.get_param("pos_name"))
    msg.speed = MOVE_SPEED
    pub.publish(msg)

    # Wait for cmd to register
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not move_done:
        rate.sleep()