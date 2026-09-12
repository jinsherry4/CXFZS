# -*- coding: utf-8 -*-
"""cmd_vel 看门狗：超时未收到速度指令时发布零速。

Gazebo 的 libgazebo_ros_planar_move 会永久保持最后一条 cmd_vel；若导航中途
ABORT 后 controller 停发指令，机器人会以旧速度无限滑行（实测飞出 2000m+）。
本节点检测到超时即补发零速，兜底一切异常退出路径。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState

REST_Z = -0.01      # 底盘静息高度（略低于 0）
LIFT_Z = 0.14       # 高出静息此值即视为离地悬浮（r13: 坎带颠簸峰值 0.10 贴 0.12 边界仍触发；真弹飞 z>0.15）
LIFT_CONFIRM_SEC = 0.6  # r14 复盘：(-1.13,-5.44) 接缝单帧弹起 abs_z0.12（rel
                        # ≈0.15 贴阈值）误触发自愈损失 7s。颠簸 <0.3s 内回落，
                        # 真弹飞持续悬浮——连续悬空 >=0.6s 才回置。


class CmdVelWatchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('timeout', 1.2)
        self.declare_parameter('rate_hz', 20.0)
        topic = self.get_parameter('topic').value
        self.timeout = float(self.get_parameter('timeout').value)
        self.last_cmd = None
        self.base_z = None
        self.cool_until = 0.0
        self._lift_since = None
        self.prev_pos = None
        self._yaw = 0.0
        self.prev_t = None
        self.cmd_mag = 0.0
        self.stuck_t = 0.0
        self.sub = self.create_subscription(Twist, topic, self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        # planar_move 底盘 z 速度每周期被强制为 0，一旦碰撞轻微离地，
        # 重力无法使其回落——机器人悬空冻结，里程计/激光全废。
        # 这里监测真值高度，发现离地即放回地面。
        self.create_subscription(ModelStates, '/model_states', self.on_states, 10)
        # r41: autopilot 直驱期间暂停卡死/悬空回置——两者都靠 SetEntityState
        # 操作机器人，同时工作会互相拽（r40 实测直驱 28s 超时全败于此）。
        self.pause_until = 0.0
        self.create_subscription(Bool, '/watchdog/pause', self.on_pause, 10)
        self.cli_set = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info(f'cmd_vel看门狗: {topic} 超时{self.timeout}s + 悬空回置')

    def on_pause(self, msg):
        # True=暂停 2s（周期重发续期）；False=立即恢复
        now = self.get_clock().now().nanoseconds * 1e-9
        if msg.data:
            self.pause_until = max(self.pause_until, now + 2.0)
        else:
            self.pause_until = 0.0

    def on_states(self, m):
        if 'six_arm' not in m.name:
            return
        p = m.pose[m.name.index('six_arm')].position
        o = m.pose[m.name.index('six_arm')].orientation
        self._yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                               1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        now = self.get_clock().now().nanoseconds * 1e-9
        # 真值速度：弹跳后底盘轻微倾斜骑在万向轮上会进入 metastable 卡死
        # （轮悬空自转、真值不动），常规 z 阈值抓不到——用「指令在动而真值
        # 不动」检测，抬 3cm 重落放平即可复位。
        if self.prev_pos is not None and self.prev_t is not None:
            dt = now - self.prev_t
            if dt > 0.2:
                sp = math.hypot(p.x - self.prev_pos[0], p.y - self.prev_pos[1]) / dt
                if self.cmd_mag > 0.08 and sp < 0.02:
                    self.stuck_t += dt
                else:
                    self.stuck_t = 0.0
                if (self.stuck_t > 2.5 and now > self.cool_until
                        and now > self.pause_until):
                    self.cool_until = now + 3.0
                    self.stuck_t = 0.0
                    yaw = getattr(self, '_yaw', 0.0)
                    req = SetEntityState.Request()
                    req.state.name = 'six_arm'
                    req.state.reference_frame = 'world'
                    req.state.pose.position.x = p.x - 0.5 * math.cos(yaw)
                    req.state.pose.position.y = p.y - 0.5 * math.sin(yaw)
                    req.state.pose.position.z = 0.03
                    req.state.pose.orientation.w = 1.0
                    self.cli_set.call_async(req)
                    self.get_logger().warn(
                        f'检测到卡死(指令{self.cmd_mag:.2f} 真值{sp:.3f}m/s) at ({p.x:.2f},{p.y:.2f})，后撤0.5m重置')
        self.prev_pos = (p.x, p.y)
        self.prev_t = now
        if self.base_z is None:
            self.base_z = p.z - 0.01
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if p.z > self.base_z + LIFT_Z:
            if self._lift_since is None:
                self._lift_since = now_sec
            elif (now_sec - self._lift_since >= LIFT_CONFIRM_SEC
                    and now_sec > self.cool_until
                    and now_sec > self.pause_until):
                self._lift_since = None
                self.cool_until = now_sec + 3.0
                req = SetEntityState.Request()
                req.state.name = 'six_arm'
                req.state.reference_frame = 'world'
                yaw = getattr(self, '_yaw', 0.0)
                req.state.pose.position.x = p.x - 0.6 * math.cos(yaw)
                req.state.pose.position.y = p.y - 0.6 * math.sin(yaw)
                req.state.pose.position.z = 0.0
                req.state.pose.orientation.w = 1.0
                self.cli_set.call_async(req)
                # 放回 z=0 后基线同步重置，防旧 base_z 过低反复触发（r10）
                self.base_z = -0.01
                self.get_logger().warn(f'检测到悬空 z={p.z:.2f}m at ({p.x:.2f},{p.y:.2f})，后撤0.6m放回')
        else:
            self._lift_since = None

    def on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        self.cmd_mag = max(self.cmd_mag * 0.9,
                           abs(msg.linear.x) + abs(msg.angular.z) * 0.3)

    def tick(self):
        if self.last_cmd is None:
            return
        dt = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if dt > self.timeout:
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
