#!/usr/bin/env python3
from mars_perception.object_face_classifier.util import obj_center_crop
import rospy
import numpy as np
import yaml
import os
import cv2 
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from mars_perception.object_face_classifier.eval import EvalObjectFaceClassifier


im: cv2.Mat = None

def depth_cb(im_msg: Image):
    global im
    im = CvBridge().imgmsg_to_cv2(im_msg)

if __name__ == '__main__':
    rospy.init_node('eval_face_detection')

    gelsight_ns = rospy.get_param('~gelsight_ns','/gelsight')
    cfg_path = rospy.get_param('~cfg_path','/home/ruppulur/catkin_ws/src/icra_2022_mars/mars_perception/scripts/face_detection/config.yml')
    s = rospy.Subscriber(f'{gelsight_ns}/depth_image',Image,callback=depth_cb)
    p = rospy.Publisher(f'{gelsight_ns}/grasp_face',Float32, queue_size=10)

    with open(cfg_path, "r") as f:
        params = yaml.safe_load(f)

    params = params['eval']
    buf_size = 5
    buffer = np.zeros(buf_size)
    buf_i = 0
    classifier = EvalObjectFaceClassifier(params)
    r = rospy.Rate(20)
    while(not rospy.is_shutdown()):
        if im is not None:
            out = classifier.run(im)
            if out != -1:
                buffer[buf_i % (buf_size-1)] = out
                buf_i += 1
                p.publish(buffer.mean())
            else:
                p.publish(out)
        r.sleep()