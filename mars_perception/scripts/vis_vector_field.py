import numpy as np
import matplotlib.pyplot as plt

import rospy
import ros_numpy

from gelsight_ros.msg import MarkerFlow

class FlowVisualizer:
    def __init__(self):
        self.flow_sub = rospy.Subscriber("/flow",MarkerFlow,callback=self.flow_cb)
        self.curr_flow = None

    def flow_cb(self, flow):
        flow = ros_numpy.msgify(flow.data)
        print(flow.shape)


if __name__ == '__main__':
    rospy.init_node("vis_vector_field")
    flow_viz = FlowVisualizer()
    rospy.spin()
