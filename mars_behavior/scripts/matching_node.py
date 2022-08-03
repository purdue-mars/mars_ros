import rospy

from sensor_msgs import Image

from mars_behavior.gelsight.face_matching import orb
from cv_bridge import CvBridge


image = None

def depth_im_cb(im_msg: Image):
    im = CvBridge().imgmsg_to_cv2(im_msg)
    orb(im,)


if __name__ == '__main__':
    rospy.init_node('matching_node')
    rospy.Subscriber('/gelsight/depth_image',Image,callback=depth_im_cb)