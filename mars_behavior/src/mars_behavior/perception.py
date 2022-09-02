import rospy

from mars_msgs.srv import RegistrationSrv
from geometry_msgs.msg import Pose


class PerceptionInterface:
    def __init__(self, ns: str = "perception"):
        self.ns_ = ns
        self.registration_srv = rospy.ServiceProxy(
            f"/{self.ns_}/registration", RegistrationSrv
        )

    def run_object_registration(self, mesh_name: str, initial_tf: Pose = Pose()):
        rospy.set_param(f"/{self.ns_}/detect_class_names", mesh_name)
        rospy.sleep(0.1)
        self.registration_srv(mesh_name=mesh_name, init_tf=initial_tf)
        rospy.sleep(3.0)  # time to converge
