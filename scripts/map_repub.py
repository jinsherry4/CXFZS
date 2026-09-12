#!/usr/bin/env python3
"""地图转发 v2：latched /map → 4x 降采样 volatile /map_volatile。

54万格 → 3.4万格：面板 JSON 解析与像素重建成本降 16 倍。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

F = 4


def decimate(m: OccupancyGrid) -> OccupancyGrid:
    w, h = m.info.width, m.info.height
    w2, h2 = (w + F - 1) // F, (h + F - 1) // F
    m2 = OccupancyGrid()
    m2.header = m.header
    m2.info = m.info
    m2.info.width, m2.info.height = w2, h2
    m2.info.resolution = m.info.resolution * F
    d = [-1] * (w2 * h2)
    for r in range(h2):
        for c in range(w2):
            occ = known = 0
            for rr in range(r * F, min((r + 1) * F, h)):
                base = rr * w
                for cc in range(c * F, min((c + 1) * F, w)):
                    v = m.data[base + cc]
                    if v != -1:
                        known += 1
                        if v > 65:
                            occ += 1
            d[r * w2 + c] = -1 if not known else (100 if occ else 0)
    m2.data = d
    return m2


class MapRepub(Node):
    def __init__(self):
        super().__init__('map_repub')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.cb, qos)
        self.pub = self.create_publisher(OccupancyGrid, '/map_volatile', 5)
        self.msg = None
        self.create_timer(5.0, self.tick)
        self.get_logger().info('map_repub v2: /map -> /map_volatile (4x 降采样)')

    def cb(self, m):
        self.msg = decimate(m)
        self.get_logger().info(f'地图降采样 {m.info.width}x{m.info.height} -> '
                               f'{self.msg.info.width}x{self.msg.info.height}')

    def tick(self):
        if self.msg is not None:
            self.pub.publish(self.msg)


def main():
    rclpy.init()
    n = MapRepub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
