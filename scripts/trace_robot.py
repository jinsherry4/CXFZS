#!/usr/bin/env python3
# 轨迹采样：2Hz 打印 six_arm 位置 40s，对照封门口坐标
import rclpy, math, time
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

rclpy.init()
n = Node('trace')
last = [None]
def cb(m):
    for i, nm in enumerate(m.name):
        if nm == 'six_arm':
            p = m.pose[i].position
            o = m.pose[i].orientation
            yaw = math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.z*o.z+o.y*o.y))
            last[0] = (p.x, p.y, yaw)
n.create_subscription(ModelStates, '/model_states', cb, 10)
t0 = time.monotonic()
dmin = 99.0
while time.monotonic() - t0 < 40:
    rclpy.spin_once(n, timeout_sec=0.2)
    if last[0] and last[0] != n.__dict__.get('_l'):
        x, y, yaw = last[0]
        d = math.hypot(x + 0.9, y + 7.0)
        dmin = min(dmin, d)
        print(f'{time.monotonic()-t0:5.1f}s ({x:6.2f},{y:6.2f}) yaw{math.degrees(yaw):6.1f} 距门{d:5.2f}', flush=True)
        n._l = last[0]
print('MIN-DIST-TO-MOUTH', round(dmin, 2), flush=True)
