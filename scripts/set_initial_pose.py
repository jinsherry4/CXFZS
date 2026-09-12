#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AMCL 初始位姿助手 v2：发布 /initialpose 并验证收敛质量。
收到 amcl_pose 后检查与种子 (0,0) 的偏差，>1m 视为误收敛（如 180°
翻转）继续播种；连错 3 次才放弃。"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    rclpy.init()
    node = rclpy.create_node('set_initial_pose')
    node.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    state = {'ok': False, 'bad': 0}
    node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', lambda m: _check(m, state), 10)

    def _check(m):
        p = m.pose.pose.position
        d = math.hypot(p.x, p.y)
        if d < 1.0:
            state['ok'] = True
        else:
            state['bad'] += 1
            if state['bad'] >= 3:
                state['ok'] = True  # 放弃纠正，交由任务节点处理

    cov = [0.8, 0, 0, 0, 0, 0,
           0, 0.8, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0.35]
    cov = [float(v) for v in cov]
    t0 = time.time()
    while rclpy.ok() and not state['ok'] and time.time() - t0 < 150:
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.pose.position.x = 0.0
        m.pose.pose.position.y = 0.0
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance = cov
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info(f'initialpose 完成: ok={state["ok"]} bad={state["bad"]}')


if __name__ == '__main__':
    main()
