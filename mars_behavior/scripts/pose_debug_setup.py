import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster, TransformListener

rospy.init_node('test2_node')
tf_broadcaster = TransformBroadcaster()

t1 = TransformStamped()
t1.header.stamp = rospy.Time.now()
t1.header.frame_id = 'hand'
t1.child_frame_id = 'object'
t1.transform.rotation.x = -0.5788183
t1.transform.rotation.y = -0.1642797
t1.transform.rotation.z = -0.5788183 
t1.transform.rotation.w = -0.5504099
t1.transform.translation.x = 0.5
t1.transform.translation.y = 0
t1.transform.translation.z = 0

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    t1.header.stamp = rospy.Time.now()
    tf_broadcaster.sendTransformMessage(t1)
    rate.sleep()