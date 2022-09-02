#!/usr/bin/env python3
import rospy
import numpy as np
import yaml
import os
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from mars_perception.object_face_classifier.util import obj_center_crop

im: cv2.Mat = None

def depth_cb(im_msg: Image):
    global im
    im = CvBridge().imgmsg_to_cv2(im_msg)

if __name__ == '__main__':
    rospy.init_node('recorder')

    gelsight_ns = rospy.get_param('~gelsight_ns','gelsight')
    face_i = rospy.get_param('~face',0)
    cfg_path = rospy.get_param('~cfg_path','/home/ruppulur/catkin_ws/src/icra_2022_mars/mars_perception/scripts/face_detection/config.yml')
    rospy.Subscriber(f'{gelsight_ns}/depth_image',Image,callback=depth_cb)

    with open(cfg_path, "r") as f:
        params = yaml.safe_load(f)

    params = params['dataset']
    data_dir = params['data_dir']
    raw_dataset_name = params['dataset_name'] 
    raw_dataset_path = os.path.join(data_dir,raw_dataset_name)

    if not os.path.exists(raw_dataset_path):
        rospy.loginfo('Making dataset')
        os.mkdir(raw_dataset_path)

    file_i = 0
    face_pth = os.path.join(raw_dataset_path,f'face_{face_i}')
    if not os.path.exists(face_pth):
        os.mkdir(face_pth)
    else:
        rospy.logwarn("Face exists! Overwriting!")
    r = rospy.Rate(3)
    while(not rospy.is_shutdown()):
        if im is not None:
            feats = len(np.argwhere(im != 0))
            if(feats > params['min_features']):
                cv2.imwrite(f'{face_pth}/{file_i}.jpg',im)
                rospy.loginfo(f"writing {file_i}.jpg")
                file_i += 1
        r.sleep()