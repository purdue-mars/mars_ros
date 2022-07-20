import rospy

from mars_msgs.srv import PointCorrTF


class PerceptionInterface:
    def __init__(self, ns: str = "perception"):
        self.ns_ = ns
        self.registration_srvs = {}
        self.registration_srvs["icp"] = rospy.ServiceProxy(
            f"/{self.ns_}/icp_mesh_tf", PointCorrTF
        )

    def run_object_registration(self, mesh_name: str, alg="icp"):
        rospy.set_param(f"{self.ns_}/detect_class_names", mesh_name)
        rospy.sleep(0.1)
        self.registration_srvs["icp"](mesh_name)
        rospy.sleep(3.0)  # time to converge
