import roslib
roslib.load_manifest('mars_control')
import rospy
import actionlib
from panda_robot import PandaArm

from mars_control.msg import MoveToAction

class MoveToServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('move_to', MoveToAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    r = PandaArm()
    pos, ori = r.ee_pose()
    print(pos, ori)
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('move_to_server')
  server = MoveToServer()
  rospy.spin()