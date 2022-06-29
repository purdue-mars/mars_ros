import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from pose_utils import (AXIS_Z, normalize, rot, rot_from_a_to_b, rot_from_a_to_b_preserve_axis, 
                        tf_mat_to_quat_stamped, transform_vector)
from tf import TransformBroadcaster, TransformListener

rospy.init_node('test_node')
tf_listener = TransformListener()
tf_listener.waitForTransform('gelsight_pad','cable_male_frame',rospy.Time(0),rospy.Duration(15))
tf_broadcaster = TransformBroadcaster()
x_axis_hand = np.array([1,0,0])

y_axis_object_in_hand = transform_vector(np.array([0,1,0]),'cable_male_frame','gelsight_pad',tf_listener)
rot_mat = rot_from_a_to_b_preserve_axis(x_axis_hand, y_axis_object_in_hand,AXIS_Z)
tf_quat = tf_mat_to_quat_stamped(rot_mat,'gelsight_pad')

print(f'{tf_quat.quaternion.x} {tf_quat.quaternion.y} {tf_quat.quaternion.z} {tf_quat.quaternion.w}')
t = TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = 'gelsight_pad'
t.child_frame_id = 'gelsight_pad_grasp'
t.transform.rotation = tf_quat.quaternion
t.transform.translation.x = 0.5
t.transform.translation.y = 0.5
t.transform.translation.z = 0

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    t.header.stamp = rospy.Time.now()
    tf_broadcaster.sendTransformMessage(t)
    rate.sleep()