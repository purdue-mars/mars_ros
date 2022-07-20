#!/usr/bin/python3

import rospy
from std_srvs.srv import Empty

rospy.init_node("ack", anonymous=True)

rospy.wait_for_service('wsg_50_driver/ack')
rospy.ServiceProxy('wsg_50_driver/ack', Empty)()
rospy.ServiceProxy('wsg_50_driver/stop', Empty)()
rospy.signal_shutdown("Finished")