#!/usr/bin/env python3

import rospy
from wsg_50_common.msg import Cmd, Status
from std_msgs.msg import Float32

MOVE_SPEED = 50.0 # mm/s (Default 50mm/s)

if __name__ == "__main__":
    rospy.init_node("weiss_vel_test")
    rate = rospy.Rate(10)

    pub = rospy.Publisher("/wsg_50_driver/goal_speed", Float32, queue_size=4, latch=True)

    while not rospy.is_shutdown():
        pub.publish(Float32(MOVE_SPEED))
        rate.sleep()