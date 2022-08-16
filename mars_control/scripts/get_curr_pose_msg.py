#!/usr/bin/python3

import frankx
import sys

if len(sys.argv) != 2:
    print("Missing argument name. Usage: python3 get_curr_pose_msg.py <name: an_an or pan_pan>")
name = sys.argv[1]

if name == "an_an":
    robot_ip = "172.18.1.2"
elif name == "pan_pan":
    robot_ip = "172.16.1.2"
elif name == "panda":
    robot_ip = "172.18.1.2"
    name = "${prefix}"
else:
    print(f"Name not recognized: {name}. Accepted values are 'pan_pan' or 'an_an'.")

robot = frankx.Robot(robot_ip)
print(robot.current_pose())
state = robot.read_once()
print(f"""<joint name="{name}_joint1" value="{state.q[0]}" />
<joint name="{name}_joint2" value="{state.q[1]}" />
<joint name="{name}_joint3" value="{state.q[2]}" />
<joint name="{name}_joint4" value="{state.q[3]}" />
<joint name="{name}_joint5" value="{state.q[4]}" />
<joint name="{name}_joint6" value="{state.q[5]}" />
<joint name="{name}_joint7" value="{state.q[6]}" />""")