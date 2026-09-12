#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地化垫片（truth-odom 模式）：map->odom 恒等 TF + 以 /odom 生成假 /amcl_pose。

依据：底盘 diff_drive 插件 odometry_source=WORLD，里程计即世界真值（实测全程
零漂移）；地图与世界坐标系对齐（补全后逐点验证 ≤0.07m）。因此 map->odom
恒等变换就是精确本地化，可完全绕开 AMCL 的粒子滤波发散/翻转/漂移问题。

原 AMCL 节点由 start_all.sh 通过 lifecycle shutdown 停用（其 bond 断开会连带
map_server 去激活，但代价地图已持有地图副本、导航生命周期不受影响）。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class LocShim(Node):
    def __init__(self):
        super().__init__('loc_shim')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.get_logger().info('loc_shim 就绪: map->odom 恒等 + 真值 amcl_pose')

    def on_odom(self, m: Odometry):
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

        p = PoseWithCovarianceStamped()
        p.header = m.header
        p.header.frame_id = 'map'
        p.pose.pose = m.pose.pose
        p.pose.covariance[0] = 0.001
        p.pose.covariance[7] = 0.001
        p.pose.covariance[35] = 0.001
        self.pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    n = LocShim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
