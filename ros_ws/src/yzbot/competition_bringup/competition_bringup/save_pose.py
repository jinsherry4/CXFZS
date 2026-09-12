# -*- coding: utf-8 -*-
"""航点记录工具：把机器人当前位姿合并写入 waypoints.yaml。

用法（SSH 里跑）：
    1) RViz 用 2D Pose Estimate 定位、Nav2 Goal/teleop 把车开到理想站位
    2) ros2 run competition_bringup save_pose red      # 记录红色方块站位
       ros2 run competition_bringup save_pose zone_A   # 记录 A 区站位
       ros2 run competition_bringup save_pose home
可用名称：red / blue / A / B / C / home（zone 前缀可省略）
"""
import os
import sys

import math
import rclpy
from rclpy.node import Node
import yaml

from ament_index_python.packages import get_package_share_directory

NAME_MAP = {'red': 'cubes.red', 'blue': 'cubes.blue',
            'A': 'zones.A', 'B': 'zones.B', 'C': 'zones.C',
            'zone_A': 'zones.A', 'zone_B': 'zones.B', 'zone_C': 'zones.C',
            'home': 'home'}


class SavePose(Node):

    def __init__(self, name):
        super().__init__('save_pose')
        self.name = name
        self.path = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'yzbot',
                                 'competition_bringup', 'config', 'waypoints.yaml')
        self.done = False
        self.create_subscription(
            __import__('nav_msgs.msg', fromlist=['Odometry']).Odometry,
            '/odom', self._cb, 10)

    def _cb(self, msg):
        if self.done:
            return
        self.done = True
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw = math.degrees(yaw)
        data = {'cubes': {}, 'zones': {}, 'home': {}}
        if os.path.exists(self.path):
            with open(self.path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or data
        group, key = NAME_MAP[self.name].split('.')
        data.setdefault(group, {})[key] = {
            'x': round(float(p.x), 3), 'y': round(float(p.y), 3),
            'yaw': round(float(yaw), 1)}
        with open(self.path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True)
        self.get_logger().info(
            f'已保存 {self.name} -> x={p.x:.3f} y={p.y:.3f} yaw={yaw:.1f} ({self.path})')
        raise SystemExit(0)


def main(args=None):
    if len(sys.argv) < 2 or sys.argv[1] not in NAME_MAP:
        print('用法: ros2 run competition_bringup save_pose <red|blue|A|B|C|home>')
        return
    rclpy.init(args=args)
    node = SavePose(sys.argv[1])
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
