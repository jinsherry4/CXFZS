#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""细粒度监控: 每5s记录 robot/方块/障碍物 位姿(从 /model_states)"""
import rclpy
from gazebo_msgs.msg import ModelStates
import time

WATCH = ('six_arm', 'red_cube_1', 'blue_cube_1', 'blue_cube_2', 'obstacle_1', 'obstacle_2')

rclpy.init()
node = rclpy.create_node('fine_monitor')
latest = {}

def cb(m):
    for name, pose in zip(m.name, m.pose):
        if name in WATCH:
            latest[name] = (pose.position.x, pose.position.y, pose.position.z)

node.create_subscription(ModelStates, '/model_states', cb, 10)
t0 = time.time()
while time.time() - t0 < 1800:
    rclpy.spin_once(node, timeout_sec=0.5)
    elapsed = time.time() - t0
    if latest and int(elapsed) % 5 == 0:
        parts = []
        for k in WATCH:
            v = latest.get(k)
            if v:
                parts.append(f"{k}=({v[0]:.1f},{v[1]:.1f},{v[2]:.2f})")
        print(f"[{elapsed:6.1f}s] " + " ".join(parts), flush=True)
