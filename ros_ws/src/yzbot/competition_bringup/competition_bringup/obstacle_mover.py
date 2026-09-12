# -*- coding: utf-8 -*-
"""移动障碍节点（闭环版）：让障碍沿目标正弦轨迹往复巡逻。

world 中每个障碍模型挂载 libgazebo_ros_planar_move.so（命名空间 /<model>），
实测该插件将 cmd_vel 按世界系直接应用（不随机体朝向旋转）。
本节点订阅 /model_states 获取障碍实际位姿，按
    v_cmd = v_ff + k_p * (p_target - p_actual)
直接下发世界系速度（前馈 + P 反馈），并锁零朝向，漂移自动纠正。

参数（YAML 字符串）：
    obstacles: "[{model: obstacle_1, x: -3.5, y: 0.0, axis: x, amp: 2.0, period: 8.0}, ...]"
"""
import math
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


DEFAULT_OBSTACLES = (
    '[{model: obstacle_1, x: -3.5, y: 0.0, axis: x, amp: 2.0, period: 24.0},'
    ' {model: obstacle_2, x: -2.0, y: -6.0, axis: x, amp: 1.4, period: 22.0}]'
)


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ObstacleMover(Node):

    def __init__(self):
        super().__init__('obstacle_mover')
        self.declare_parameter('obstacles', DEFAULT_OBSTACLES)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('k_p', 3.0)
        self.declare_parameter('k_yaw', 2.0)
        cfg = yaml.safe_load(self.get_parameter('obstacles').value) or []
        self.k_p = float(self.get_parameter('k_p').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.items = []
        for it in cfg:
            amp = float(it.get('amp', 2.0))
            period = max(1.0, float(it.get('period', 8.0)))
            self.items.append({
                'model': it['model'],
                'x': float(it.get('x', 0.0)),
                'y': float(it.get('y', 0.0)),
                'axis': it.get('axis', 'x'),
                'amp': amp,
                'period': period,
                'v_max': 1.3 * amp * (2 * math.pi / period),
                'pub': self.create_publisher(Twist, f"/{it['model']}/cmd_vel", 10),
            })
        if not self.items:
            self.get_logger().warn('无障碍配置，节点空转')
        self.pose = {}
        self.sub = self.create_subscription(
            ModelStates, '/model_states', self._on_states, 30)
        rate = float(self.get_parameter('rate_hz').value)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'移动障碍(闭环): {[i["model"] for i in self.items]}')

    def _on_states(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            self.pose[name] = (
                pose.position.x, pose.position.y,
                yaw_from_quat(pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w))

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        for it in self.items:
            phase = 2 * math.pi * t / it['period']
            center = it['x'] if it['axis'] == 'x' else it['y']
            p_ref = center + it['amp'] * math.sin(phase)
            v_ff = it['amp'] * (2 * math.pi / it['period']) * math.cos(phase)
            msg = Twist()
            state = self.pose.get(it['model'])
            if state is None:
                # 尚未收到位姿：仅前馈
                if it['axis'] == 'x':
                    msg.linear.x = v_ff
                else:
                    msg.linear.y = v_ff
            else:
                px, py, yaw = state
                p_actual = px if it['axis'] == 'x' else py
                v_axis = v_ff + self.k_p * (p_ref - p_actual)
                if it['axis'] == 'x':
                    msg.linear.x = v_axis
                else:
                    msg.linear.y = v_axis
                msg.angular.z = -self.k_yaw * yaw
            lim = it['v_max']
            msg.linear.x = max(-lim, min(lim, msg.linear.x))
            msg.linear.y = max(-lim, min(lim, msg.linear.y))
            it['pub'].publish(msg)

    def stop(self):
        zero = Twist()
        for it in self.items:
            it['pub'].publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # 关停时发零速，避免障碍持续滑行
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
