# -*- coding: utf-8 -*-
"""虚拟搬运节点：方块经 planar_move 速度闭环跟随底盘，替代物理焊接。

物理焊接(fixed joint)与底盘运动叠加会令 ODE 约束求解发散（实测整机被抛飞）；
/gazebo/set_entity_state 服务在高负载下不响应。故采用障碍物同款成熟通道：
每个方块挂 planar_move 插件，本节点按「目标位姿-当前位姿」P 控制发布
/<cube>/cmd_vel，方块以速度随行，物理上不参与任何约束求解。

接口 /mission/carry (std_msgs/String)：
  - 方块模型名(如 red_cube_1) → 开始跟随（捕获底盘系抓取偏移）
  - 'stop'                    → 发零速停止，方块原地停留
"""
import math

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

K_P = 3.0        # 位置误差 -> 速度
V_MAX = 0.5      # 跟随速度上限 m/s（过快会把方块顶进墙触发 ODE 弹飞）
R_SAFE = 0.50    # 方块-底盘中心最小安全距离（防挤压弹飞）
CARRY_Z = 0.16
CUBE_REST_Z = 0.015  # 方块静息高度（与原生方块一致）；释放时落地，杜绝悬空障碍
CARRY_OFFSET_X = 0.45  # 随行固定前置偏移（抓取几何确定，不用捕获值）   # 抓取后抬升高度（避免贴地拖行卡滞/打滑）
STALL_ERR = 0.35  # 方块持续落后目标超此距离视为堵转
STALL_SEC = 1.5   # 堵转持续时长：转零速等待，避免把方块顶进墙
UNSTALL_ERR = 0.2  # 误差回落到此以内恢复推动


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CarryFollower(Node):
    def __init__(self):
        super().__init__('carry_follower')
        self.cube = None
        self.offset = None          # 底盘系抓取偏移 (x, y)
        self.pending = False        # 等待捕获偏移（首拍方块仍是自然位姿）
        self.base_pos = None        # (x, y, yaw)
        self.cube_pos = None        # (x, y)
        self.pub = None             # 正在搬运方块的发布器
        self.stall_since = None     # 堵转起始时刻（防顶墙弹飞）
        # 全部方块按 20Hz 持续发布速度（空闲=零速）：planar_move 收令时
        # 会把 z 速度一并清零，方块才能像移动障碍一样被"按住"不下坠
        self.pubs = {}
        for i in range(1, 6):
            for c in ('red', 'blue'):
                n = f'{c}_cube_{i}'
                self.pubs[n] = self.create_publisher(Twist, f'/{n}/cmd_vel', 10)
        self.create_subscription(ModelStates, '/model_states', self._states, 10)
        self.create_subscription(String, '/mission/carry', self._carry_cmd, 10)
        # 模型级 set_entity_state 实例秒回（/gazebo 前缀的 world 级实例不可达）
        self.cli_setent = self.create_client(SetEntityState, '/set_entity_state')
        self.create_timer(0.05, self._tick)
        self.get_logger().info('carry_follower(Twist) 就绪，等待 /mission/carry')

    def _states(self, m):
        if 'six_arm' in m.name:
            i = m.name.index('six_arm')
            p = m.pose[i].position
            self.base_pos = (p.x, p.y, yaw_of(m.pose[i].orientation))
        if not self.cube:
            return
        if self.cube in m.name:
            i = m.name.index(self.cube)
            p = m.pose[i].position
            if self.pending:
                # 首拍：方块尚在自然位姿，捕获底盘系偏移
                bx, by, yaw = self.base_pos
                cy, sy = math.cos(-yaw), math.sin(-yaw)
                dx, dy = p.x - bx, p.y - by
                self.offset = (cy * dx - sy * dy, sy * dx + cy * dy)
                self.pending = False
                self.get_logger().info(
                    f'{self.cube} 偏移已捕获 ({self.offset[0]:.2f}, {self.offset[1]:.2f})')
            self.cube_pos = (p.x, p.y)

    def _carry_cmd(self, m):
        name = m.data.strip()
        if name.startswith('drop|'):
            # r11：区内槽位绝对坐标落点，与机器人站位误差解耦
            try:
                tx = float(name.split('|')[1])
                ty = float(name.split('|')[2])
            except (ValueError, IndexError):
                tx = ty = None
            if self.cube:
                self.pubs[self.cube].publish(Twist())
                if tx is not None:
                    req = SetEntityState.Request()
                    req.state.name = self.cube
                    req.state.reference_frame = 'world'
                    req.state.pose.position.x = tx
                    req.state.pose.position.y = ty
                    req.state.pose.position.z = CUBE_REST_Z
                    req.state.pose.orientation.w = 1.0
                    self.cli_setent.call_async(req)
                    self.get_logger().info(
                        f'{self.cube} 落地槽位 ({tx:.2f},{ty:.2f})')
            self.cube = None
            self.offset = None
            self.pub = None
            return
        if name == 'stop' or name.startswith('stop|'):
            lat, lon = 0.0, 0.0
            if '|' in name:
                parts = name.split('|')
                try:
                    lat = float(parts[1]) if len(parts) > 1 else 0.0
                    lon = float(parts[2]) if len(parts) > 2 else 0.0
                except ValueError:
                    lat, lon = 0.0, 0.0
            if self.cube:
                self.pubs[self.cube].publish(Twist())
                # 释放即落地：传送至静息高度并施加横向错位（多块同区防叠压）。
                # 旧版只停随行，方块永久悬停 CARRY_Z=0.16——恰在激光平面，
                # 变成 costmap 障碍挡住下一块的放置接近（第5轮 blue2 搬运
                # 81s 实测），且裁判视角方块悬浮空中观感差。
                if self.base_pos is not None:
                    bx, by, yaw = self.base_pos
                    cy, sy = math.cos(yaw), math.sin(yaw)
                    fwd = CARRY_OFFSET_X + lon
                    tx = bx + cy * fwd - sy * lat
                    ty = by + sy * fwd + cy * lat
                    req = SetEntityState.Request()
                    req.state.name = self.cube
                    req.state.reference_frame = 'world'
                    req.state.pose.position.x = tx
                    req.state.pose.position.y = ty
                    req.state.pose.position.z = CUBE_REST_Z
                    req.state.pose.orientation.w = 1.0
                    self.cli_setent.call_async(req)
                    self.get_logger().info(
                        f'{self.cube} 落地 ({tx:.2f},{ty:.2f}) 横向错位 {lat:+.2f}m')
            self.cube = None
            self.offset = None
            self.pub = None
            return
        self.cube = name
        self.offset = None
        self.pending = True
        self.pub = self.pubs[name]
        self.stall_since = None
        self.get_logger().info(f'开始虚拟搬运 {name}')

    def _tick(self):
        self._tick_count = getattr(self, '_tick_count', 0) + 1
        for n, p in self.pubs.items():
            if n == self.cube and self.offset and self.base_pos and self.cube_pos:
                # 传送跟随：每 tick 直接摆放方块到位。速度闭环跟随在窄通道
                # 会把方块顶进墙，反作用力将底盘顶悬空（漏斗区实测连续悬空）；
                # 传送跟随不产生任何接触力，搬运全程物理确定。
                bx, by, yaw = self.base_pos
                cy, sy = math.cos(yaw), math.sin(yaw)
                ox, oy = CARRY_OFFSET_X, 0.0
                tx = bx + cy * ox - sy * oy
                ty = by + sy * ox + cy * oy
                req = SetEntityState.Request()
                req.state.name = n
                req.state.reference_frame = 'world'
                req.state.pose.position.x = tx
                req.state.pose.position.y = ty
                req.state.pose.position.z = CARRY_Z
                req.state.pose.orientation.w = 1.0
                self.cli_setent.call_async(req)
            else:
                # 空闲方块降到 2Hz：零速保持不依赖高频刷新
                if self._tick_count % 10 == 0:
                    p.publish(Twist())


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    n = CarryFollower()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.pub:
            n.pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
