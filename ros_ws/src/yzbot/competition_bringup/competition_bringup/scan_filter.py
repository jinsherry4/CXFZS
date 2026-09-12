#!/usr/bin/env python3
"""雷达自视过滤中继：/scan_raw → /scan。

机器人自身臂体在侧后方扇区（|角|>70°）的 1.0-1.5m 处持续回波（自视，旋转
时稳定锁在机器人坐标系），污染体素层/障碍层并腐蚀 AMCL 扫描匹配——该扇区
1.8m 内命中置 inf。前向扇区只做 0.25m 噪声底：LIVE15 实测全局 1.6m 截止会
把 1m 处的真实墙壁一并抹掉，机器人撞墙推挤直至 ODE 弹射（整机抛飞 50km+），
因此前向近距障碍必须保留。"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SELF_SECTOR = math.radians(70.0)   # 侧后方自视扇区半宽
SELF_MIN = 1.8                     # 自视扇区内低于此距离视为自体
FRONT_MIN = 0.25                   # 前向噪声底


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self._cb, 10)

    def _cb(self, m):
        a = m.angle_min
        out = []
        for v in m.ranges:
            ang = math.atan2(math.sin(a), math.cos(a))
            if abs(ang) > SELF_SECTOR:
                out.append(v if v >= SELF_MIN else float('inf'))
            else:
                out.append(v if v >= FRONT_MIN else float('inf'))
            a += m.angle_increment
        m.ranges = out
        self.pub.publish(m)


def main():
    rclpy.init()
    n = ScanFilter()
    rclpy.spin(n)


if __name__ == '__main__':
    main()
