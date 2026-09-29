#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3D 简化可视化节点（演示用）。

发布：
  /map_clean  (nav_msgs/OccupancyGrid)        — /map 滤除小噪点簇后的"纯墙壁"图
  /viz/scene  (visualization_msgs/MarkerArray)— 机器人 + 2 个移动障碍的 3D 标记

配合 Foxglove「3D简化版」布局使用：3D 面板只显示这两个话题，
去掉 scan / costmap / 路径 / volatile 地图等图层。
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy)
from builtin_interfaces.msg import Duration
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

MIN_CLUSTER = 25        # 连通簇小于该格数 → 视为噪点丢弃
REPUB_INTERVAL = 10.0   # /map_clean 重发周期（晚连接的客户端也能拿到）
ROBOT = 'six_arm'
OBSTACLES = ('obstacle_1', 'obstacle_2')
OBS_COLORS = {'obstacle_1': (0.90, 0.30, 0.24),   # 红
              'obstacle_2': (0.95, 0.67, 0.15)}   # 橙


def clean_map(src: OccupancyGrid) -> OccupancyGrid:
    """4 邻域连通域分析：丢弃 < MIN_CLUSTER 的占用簇（地板噪点）。"""
    w, h, data = src.info.width, src.info.height, list(src.data)
    seen = bytearray(w * h)
    keep = bytearray(w * h)
    for r0 in range(h):
        for c0 in range(w):
            i0 = r0 * w + c0
            if data[i0] <= 65 or seen[i0]:
                continue
            comp = []
            q = deque([(r0, c0)])
            seen[i0] = 1
            while q:
                r, c = q.popleft()
                comp.append(r * w + c)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    r2, c2 = r + dr, c + dc
                    if 0 <= r2 < h and 0 <= c2 < w:
                        j = r2 * w + c2
                        if data[j] > 65 and not seen[j]:
                            seen[j] = 1
                            q.append((r2, c2))
            if len(comp) >= MIN_CLUSTER:
                for j in comp:
                    keep[j] = 1
    out = OccupancyGrid()
    out.header = src.header
    out.info = src.info
    out.data = [100 if keep[i] else (0 if data[i] == 0 else -1)
                for i in range(w * h)]
    return out


class VizScene(Node):

    def __init__(self):
        super().__init__('viz_scene')
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map_clean',
                                             latched)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.pub_scene = self.create_publisher(MarkerArray, '/viz/scene', 10)
        self.create_subscription(ModelStates, '/model_states',
                                 self._on_states, 30)
        self._last_map = None
        self.create_timer(REPUB_INTERVAL, self._repub)
        self.get_logger().info(
            f'viz_scene 就绪: /map_clean(噪声簇<{MIN_CLUSTER}格滤除) + '
            f'/viz/scene({ROBOT}+{",".join(OBSTACLES)})')

    # ---------- 地图 ----------
    def _on_map(self, m):
        try:
            clean = clean_map(m)
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f'地图清理失败: {e}')
            return
        self._last_map = clean
        self.pub_map.publish(clean)
        occ0 = sum(1 for v in m.data if v > 65)
        occ1 = sum(1 for v in clean.data if v > 65)
        self.get_logger().info(
            f'地图已清理: 占用 {occ0} → {occ1} 格（滤除噪点 {occ0 - occ1}）')

    def _repub(self):
        if self._last_map is not None:
            self.pub_map.publish(self._last_map)

    # ---------- 场景标记 ----------
    def _on_states(self, msg: ModelStates):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        names = list(msg.name)
        poses = list(msg.pose)

        if ROBOT in names:
            p = poses[names.index(ROBOT)]
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = now
            mk.ns = 'robot'
            mk.id = 0
            mk.type = Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose.position.x = p.position.x
            mk.pose.position.y = p.position.y
            mk.pose.position.z = 0.18
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 0.5
            mk.scale.z = 0.36
            mk.color = ColorRGBA(r=0.23, g=0.51, b=0.96, a=0.95)
            mk.lifetime = Duration(sec=1)
            arr.markers.append(mk)
            # 朝向箭头
            ar = Marker()
            ar.header = mk.header
            ar.ns = 'robot_heading'
            ar.id = 1
            ar.type = Marker.ARROW
            ar.action = Marker.ADD
            ar.pose.position.x = p.position.x
            ar.pose.position.y = p.position.y
            ar.pose.position.z = 0.45
            ar.pose.orientation = p.orientation
            ar.scale.x = 0.55
            ar.scale.y = 0.08
            ar.scale.z = 0.16
            ar.color = ColorRGBA(r=0.23, g=0.51, b=0.96, a=0.95)
            ar.points = [Point(x=0.0, y=0.0, z=0.0),
                         Point(x=0.55, y=0.0, z=0.0)]
            ar.lifetime = Duration(sec=1)
            arr.markers.append(ar)

        for k, nm in enumerate(OBSTACLES):
            if nm not in names:
                continue
            p = poses[names.index(nm)]
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = now
            mk.ns = 'obstacles'
            mk.id = 10 + k
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = p.position.x
            mk.pose.position.y = p.position.y
            mk.pose.position.z = 0.25
            mk.pose.orientation = p.orientation
            mk.scale.x = mk.scale.y = mk.scale.z = 0.5
            c = OBS_COLORS[nm]
            mk.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.9)
            mk.lifetime = Duration(sec=1)
            arr.markers.append(mk)

        if arr.markers:
            self.pub_scene.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    n = VizScene()
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