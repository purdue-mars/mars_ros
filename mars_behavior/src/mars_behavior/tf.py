import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Vector3Stamped

import ros_numpy


class TFInterface:
    def __init__(self, tf_listener: tf.TransformListener, robot_id: str = "panda"):
        self.robot_id_ = robot_id
        self.base_frame_: str = robot_id + rospy.get_param("/base_frame_postfix")
        self.eef_frame_: str = robot_id + rospy.get_param("/eef_frame_postfix")
        self.grasp_frame_: str = robot_id + rospy.get_param("/grasp_frame_postfix")
        self.tf_listener_ = tf_listener

    def to_base(self, pose: PoseStamped):
        if pose.header.frame_id == self.grasp_frame_:
            pose = self.grasp_pose_to_eef_pose(pose)
        try:
            pose = self.tf_listener_.transformPose(self.base_frame_, pose)
        except Exception() as e:
            print(e)
            return
        return pose

    def get_transform(self, source_frame: str, target_frame: str):
        try:
            (trans, rot) = self.tf_listener_.lookupTransform(
                target_frame, source_frame, rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            print(e)
            return

        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        return pose

    def transform_vector(self, v: np.ndarray, from_frame: str, target_frame: str):
        vec_msg = Vector3Stamped()
        vec_msg.header.frame_id = from_frame
        vec_msg.vector = ros_numpy.msgify(Vector3, v)
        return ros_numpy.numpify(
            self.tf_listener_.transformVector3(target_frame, vec_msg).vector
        )

    def grasp_pose_to_eef_pose(
        self, new_grasp_to_cur_grasp: PoseStamped
    ) -> PoseStamped:
        """Generates the eef pose that corresponds with the grasp pose"""
        eef_frame_rel_grasp_frame = self.get_transform(
            self.eef_frame_, self.grasp_frame_
        ).pose

        # G = grasp frame, E = eef_frame
        X_GE: np.ndarray = ros_numpy.numpify(eef_frame_rel_grasp_frame)
        X_GcurGnew: np.ndarray = ros_numpy.numpify(new_grasp_to_cur_grasp.pose)
        X_EcurEnew = ros_numpy.msgify(
            Pose, np.matmul(np.linalg.inv(X_GE), np.matmul(X_GcurGnew, X_GE))
        )
        new_eef_to_cur_eef = PoseStamped(pose=X_EcurEnew)
        new_eef_to_cur_eef.header.frame_id = self.eef_frame_
        return new_eef_to_cur_eef

    def grasp_pose_from_object(
        self, object_frame, grasp_point_rel_obj: Pose = None
    ) -> PoseStamped:
        """Generates pose in the grasp frame corresponding to a pose relative to the object"""
        object_frame_rel_grasp_frame: PoseStamped = self.get_transform(
            object_frame, self.grasp_frame_
        ).pose
        assert object_frame_rel_grasp_frame is not None
        if grasp_point_rel_obj is None:
            return object_frame_rel_grasp_frame
        # G = grasp frame, O = object frame, P = grasp point on object frame
        X_GO: np.ndarray = ros_numpy.numpify(object_frame_rel_grasp_frame)
        X_OP: np.ndarray = ros_numpy.numpify(grasp_point_rel_obj)
        X_GP = ros_numpy.msgify(Pose, np.matmul(X_GO, X_OP))
        grasp_point_rel_grasp_frame = PoseStamped(pose=X_GP)
        grasp_point_rel_grasp_frame.header.frame_id = self.grasp_frame_
        return grasp_point_rel_grasp_frame

    @property
    def ee_frame(self):
        return self.eef_frame_

    @property
    def base_frame(self):
        return self.base_frame_

    @property
    def id(self):
        return self.robot_id_

    @property
    def grasp_frame(self):
        return self.grasp_frame_
