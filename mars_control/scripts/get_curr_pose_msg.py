#!/usr/bin/python3

import frankx
import sys

robot_ip = "192.168.0.144"
if len(sys.argv) > 1:
    robot_ip = sys.argv[1]
robot = frankx.Robot(robot_ip)

pose = robot.current_pose()
print(
    f"""position:
    x: {pose.x}
    y: {pose.y}
    z: {pose.z}
orientation:
    x: {pose.q_x}
    y: {pose.q_y}
    z: {pose.q_z}
    w: {pose.q_w}
"""
)
