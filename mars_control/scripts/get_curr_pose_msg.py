#!/usr/bin/python3

import frankx
import sys

robot_ip = "172.16.1.2"
if len(sys.argv) > 1:
    robot_ip = sys.argv[1]
robot = frankx.Robot(robot_ip)

state = robot.read_once()
print(f"""<joint name="panda_joint1" value="{state.q[0]}" />
<joint name="panda_joint2" value="{state.q[1]}" />
<joint name="panda_joint3" value="{state.q[2]}" />
<joint name="panda_joint4" value="{state.q[3]}" />
<joint name="panda_joint5" value="{state.q[4]}" />
<joint name="panda_joint6" value="{state.q[5]}" />
<joint name="panda_joint7" value="{state.q[6]}" />""")