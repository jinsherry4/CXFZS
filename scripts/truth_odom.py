#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真值里程计发布器：从 Gazebo /model_states 读取六臂底盘世界真值位姿，
发布 /odom（nav_msgs）与 odom->base_footprint TF。

配合运动学底盘（planar_move, publish_odom=false）使用：
- 位姿为世界真值，不存在轮地打滑/被阻挡时的假积分问题；
- twist 以真值差分计算，供进度检测等使用。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TruthOdom(Node):
    def __init__(self):
        super().__init__('truth_odom')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.br = TransformBroadcaster(self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Odometry, '/odom', qos)
        self.prev = None
        self.prev_t = None
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.get_logger().info('truth_odom 就绪: /odom + odom->base_footprint 来自世界真值')

    def on_states(self, m):
        if 'six_arm' not in m.name:
            return
        i = m.name.index('six_arm')
        pose = m.pose[i]
        now = self.get_clock().now().to_msg()

        # twist：真值差分
        lin = ang = 0.0
        if self.prev and self.prev_t:
            dt = (now.sec + 1e-9 * now.nanosec) - self.prev_t
            if dt > 1e-3:
                lin = math.hypot(pose.position.x - self.prev[0],
                                 pose.position.y - self.prev[1]) / dt
                ang = (yaw_of(pose.orientation) - self.prev[2]) / dt
                if ang > math.pi:
                    ang -= 2 * math.pi
                elif ang < -math.pi:
                    ang += 2 * math.pi
        self.prev = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self.prev_t = now.sec + 1e-9 * now.nanosec

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose = pose
        odom.twist.twist.linear.x = lin
        odom.twist.twist.angular.z = ang
        odom.pose.covariance[0] = 0.0001
        odom.pose.covariance[7] = 0.0001
        odom.pose.covariance[35] = 0.0001
        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    n = TruthOdom()
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
