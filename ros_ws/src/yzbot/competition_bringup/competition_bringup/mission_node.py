# -*- coding: utf-8 -*-
"""任务调度节点：解析任务 → 依次导航/抓取/放置 → 发布状态供可视化面板显示。

输入话题：
    /mission/command  (std_msgs/String)  JSON: {"question":"...","items":[{"color":"red","count":4,"zone":"A"},...]}
输出话题（coStudio 面板直接订阅）：
    /mission/task_info  任务题目、抓取顺序与清单（JSON 文本）
    /mission/progress   任务进度（如 "红色 2/4"）
    /mission/work_state 工作状态（如 "正在抓取 red_cube_3"）
"""
import json
import math
import os
import threading
import time

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
import yaml

from .arm_controller import ArmController
from .nav_controller import NavController

COLOR_CN = {'red': '红色', 'blue': '蓝色'}

# 放置区真实边界（world: zone_a/b/c 均 1x0.5m 地贴，纯 visual 无碰撞）
ZONE_RECT = {'A': (-3.0, -2.0, 1.75, 2.25),
             'B': (-6.5, -5.5, -13.25, -12.75),
             'C': (6.0, 7.0, -12.25, -11.75)}
# 同区多块横向错位步长（区宽一半再留裕量：A/B 1m 宽→0.3，C 0.5m 宽→0.18）
ZONE_LAT = {'A': 0.3, 'B': 0.3, 'C': 0.18}
# 区中心（world 地贴 pose 真值）：放置槽位的绝对基准
ZONE_CENTER = {'A': (-2.5, 2.0), 'B': (-6.0, -13.0), 'C': (6.5, -12.0)}


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').value
        if not wp_file:
            from ament_index_python.packages import get_package_share_directory
            wp_file = os.path.join(get_package_share_directory('competition_bringup'),
                                   'config', 'waypoints.yaml')
        with open(wp_file, 'r', encoding='utf-8') as f:
            self.wp = yaml.safe_load(f) or {}
        self.get_logger().info(f'航点配置: {wp_file}')

        self.arm = ArmController(self)
        self.nav = NavController(self)

        self.pub_task = self.create_publisher(String, '/mission/task_info', 10)
        self.pub_prog = self.create_publisher(String, '/mission/progress', 10)
        self.pub_state = self.create_publisher(String, '/mission/work_state', 10)
        self.pub_red_req = self.create_publisher(Int32, '/mission/red_required', 10)
        self.pub_blue_req = self.create_publisher(Int32, '/mission/blue_required', 10)
        self.pub_red_done = self.create_publisher(Bool, '/mission/red_done', 10)
        self.pub_blue_done = self.create_publisher(Bool, '/mission/blue_done', 10)
        self.carry_pub = self.create_publisher(String, '/mission/carry', 10)

        self.create_subscription(String, '/mission/command', self._on_command, 10)
        # AMCL 就绪信号：/amcl_pose 在初始化或运动时发布；静止期无消息，
        # 因此超时未收到时由本节点主动重发 /initialpose 自愈，而不是依赖 TF 缓存。
        self._amcl_evt = threading.Event()
        self._amcl_xy = None
        self._amcl_cov = None
        self._reseed_ok_after = 0.0
        self._loc_fix_times = []
        self._amcl_yaw = None
        self._odom_xy = None
        self._odom_z = 0.0
        self._odom_z_base = 0.0
        self._explode_ok_after = 0.0
        self._recovering = False
        self._round_t0 = None
        # 注意：world 级 gazebo_ros_state 的 /gazebo/set_entity_state 实测
        # 始终无法被发现/响应，模型级实例 /set_entity_state 秒回——用后者。
        self.cli_setent = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.pub_initpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._stop_evt = threading.Event()
        self.mission_thread = None
        threading.Thread(target=self._loc_watchdog, daemon=True).start()

    def _on_odom(self, m):
        p = m.pose.pose.position
        self._odom_xy = (p.x, p.y)
        self._odom_z = p.z
        # 腾空检测：地面机器人 z 突然高于基线 0.35m = ODE 弹飞已发生（实测先
        # 缓升后垂直抛射，1s 内到 7m+）。立刻传送回家并重启定位，把"整轮报废"
        # 降级为"20s 损失"。z 取相对基线：diff_drive 的 odom z 积分了整段飞行，
        # 传送后不会归零（实测停 5-19m 缓慢衰减），按绝对值判断会无限重触发。
        if (p.z - self._odom_z_base > 0.35 and not self._recovering
                and time.monotonic() >= self._explode_ok_after):
            self._recovering = True
            threading.Thread(target=self._explode_recover, daemon=True).start()

    def _zero_cmdvel(self, sec):
        """持续发布零速，压制 diff_drive/planar_move 的残留速度
        （planar_move 会永久保持最后一条 cmd_vel，不归零就一直滑行）。"""
        from geometry_msgs.msg import Twist
        if not hasattr(self, '_esc_pub'):
            from geometry_msgs.msg import Vector3  # noqa: F401
            self._esc_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            self._esc_pub.publish(Twist())
            time.sleep(0.05)

    def _explode_recover(self):
        try:
            self._explodes = getattr(self, '_explodes', 0) + 1
            if self._explodes >= 4:
                self._state('弹飞已达熔断次数，本轮任务终止')
                self._fuse_blown = True
                self.nav.cancel_active()
                self._zero_cmdvel(1.0)
                return
            self._state('检测到机器人腾空（ODE 弹飞），传送回家重启定位')
            # 1) 先取消在执行的 Nav2 目标：恢复期间 DWB 若继续下发速度，
            #    传送回家后机器人会被立刻再次驱动，重新触发弹飞（实测死循环）。
            self.nav.cancel_active()
            self._zero_cmdvel(1.5)
            # 2) 传送回家：高负载下 set_entity_state 可能不响应（carry_follower
            #    同款教训），必须等待响应并重试，fire-and-forget 会静默失败，
            #    让机器人带着重置过的定位漂在几百米外的虚空里。
            ok = False
            for attempt in range(5):
                req = SetEntityState.Request()
                req.state.name = 'six_arm'
                req.state.reference_frame = 'world'
                req.state.pose.position.x = 0.0
                req.state.pose.position.y = 0.0
                req.state.pose.position.z = 0.0
                req.state.pose.orientation.w = 1.0
                fut = self.cli_setent.call_async(req)
                t0 = time.monotonic()
                while not fut.done() and time.monotonic() - t0 < 8.0:
                    time.sleep(0.05)
                if fut.done() and fut.result() is not None and fut.result().success:
                    ok = True
                    break
                self._state(f'传送服务无响应，第 {attempt + 1}/5 次重试')
                time.sleep(1.0)
            if not ok:
                self._state('传送服务持续无响应，放弃传送')
            time.sleep(2.0)
            # 3) 传送后以当前 odom z 为新基线，并冷却 8s，防止积分残留的
            # 高 z 反复触发；传送 z 用 0.0——相对静息高度的落差冲击会
            # 让 ODE 直接再次弹飞（boot_live22 实测死循环）。
            self._odom_z_base = self._odom_z
            self._explode_ok_after = time.monotonic() + 8.0
            self._reseed_amcl((0.0, 0.0))
            self._reseed_ok_after = time.monotonic() + 30.0
            # 4) 传送后补一轮零速，确保控制器取消期间无残留速度
            self._zero_cmdvel(1.0)
            # 5) 收臂：弹飞前若在搬运/抓取位，延伸的臂会继续污染代价地图
            try:
                self.arm.stow()
            except Exception:
                pass
            self._state('弹飞恢复完成，继续任务')
        finally:
            self._recovering = False


    # ---------- 状态发布 ----------
    def _state(self, text):
        m = String()
        m.data = text
        self.pub_state.publish(m)
        self.get_logger().info(f'[状态] {text}')

    def _progress(self, text):
        m = String()
        m.data = text
        self.pub_prog.publish(m)

    def _task_info(self, question, items):
        seq = ' → '.join(
            f'{COLOR_CN.get(it["color"], it["color"])}×{it["count"]}→{it["zone"]}区'
            for it in items)
        m = String()
        m.data = json.dumps({'question': question, 'items': items, 'sequence': seq},
                            ensure_ascii=False)
        self.pub_task.publish(m)

    # ---------- 命令入口 ----------
    def _on_command(self, msg: String):
        if self.mission_thread and self.mission_thread.is_alive():
            self.get_logger().warn('已有任务在执行，忽略新命令')
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'命令 JSON 解析失败: {msg.data[:100]}')
            return
        question = data.get('question', '')
        items = data.get('items', [])
        if not items:
            self._state('任务清单为空，忽略')
            return
        self._task_info(question, items)
        self.mission_thread = threading.Thread(
            target=self._run_mission, args=(question, items), daemon=True)
        self.mission_thread.start()

    # ---------- 航点 ----------
    def _wait_amcl(self, total_sec=75.0, retry_sec=10.0):
        """等待 AMCL 定位；静止期 /amcl_pose 不发布，每 retry 秒重发 initialpose 触发。"""
        deadline = time.monotonic() + total_sec
        while time.monotonic() < deadline:
            if self._amcl_evt.wait(timeout=min(retry_sec, deadline - time.monotonic())):
                return True
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.orientation.w = 1.0
            m.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
            self.pub_initpose.publish(m)
            self.get_logger().info('未收到 /amcl_pose，重发 initialpose (0,0,0)')
        return False

    def _cube_spot(self, color, idx):
        cubes = self.wp.get('cubes', {}) or {}
        spot = cubes.get(f'{color}_{idx}')
        if spot is None:
            spot = cubes.get(color)
        return spot

    def _zone_spot(self, zone):
        return (self.wp.get('zones', {}) or {}).get(zone)

    # ---------- 主流程 ----------
    def _on_amcl(self, m):
        c0 = m.pose.covariance[0] + m.pose.covariance[7]
        if c0 > 0.5:
            # 真值垫片协方差 0.002；大协方差消息来自残留的 AMCL 端点
            # （lifecycle finalized 后仍偶发），一旦采纳会把锚点拖偏
            return
        self._amcl_evt.set()
        p = m.pose.pose.position
        self._amcl_xy = (p.x, p.y)
        o = m.pose.pose.orientation
        self._amcl_yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                    1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        c = m.pose.covariance
        self._amcl_cov = c[0] + c[7]

    def _escape(self):
        """脱困机动：短距慢速倒车+低速原地转。动作必须温和——轮扭矩对抗墙壁
        接触曾触发 ODE 弹射（整机抛飞 85m），禁止大幅/长时间盲动。"""
        from geometry_msgs.msg import Twist, Vector3
        if not hasattr(self, '_esc_pub'):
            self._esc_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.8:
            self._esc_pub.publish(Twist(linear=Vector3(x=-0.1)))
            time.sleep(0.05)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            self._esc_pub.publish(Twist(angular=Vector3(z=0.25)))
            time.sleep(0.05)
        self._esc_pub.publish(Twist())

    def _backup(self, dist=0.45):
        """放置/释放后的脱离机动：低速直退离开刚放下的方块。方块落点在车头
        正前方约 0.35m，下一个导航目标起步若正对方块，Nav2 代价地图看不到
        它，撞击/楔入会触发 ODE 弹飞（MOCK #21 返程起步实测）。"""
        from geometry_msgs.msg import Twist, Vector3
        if not hasattr(self, '_esc_pub'):
            self._esc_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        t0 = time.monotonic()
        while time.monotonic() - t0 < dist / 0.2:
            self._esc_pub.publish(Twist(linear=Vector3(x=-0.2)))
            time.sleep(0.05)
        self._esc_pub.publish(Twist())

    def _clear_released_cube(self, cube):
        """就近备降后把已释放的方块挪到车后 1.2m。备降点常在通道内且方块
        正对返程起步方向；仅对已失去放置得分可能的备降方块使用，
        正常放置进区的方块不动（位置计分）。"""
        if self._amcl_xy is None or self._amcl_yaw is None:
            return
        req = SetEntityState.Request()
        req.state.entity_name = cube
        req.state.pose.position.x = self._amcl_xy[0] - 1.2 * math.cos(self._amcl_yaw)
        req.state.pose.position.y = self._amcl_xy[1] - 1.2 * math.sin(self._amcl_yaw)
        req.state.pose.position.z = 0.015   # 与静息高度一致，不留悬空方块
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        fut = self.cli_setent.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        self._state(f'{cube} 已清离行进路径')

    def _over_budget(self):
        if self._round_t0 is None:
            return False
        budget = float(os.environ.get('MISSION_ROUND_SEC', '230'))
        return time.monotonic() - self._round_t0 > budget

    def _near(self, spot, tol):
        if self._amcl_xy is None:
            return False
        return math.hypot(self._amcl_xy[0] - spot['x'],
                          self._amcl_xy[1] - spot['y']) <= tol

    def _crossing_via(self, spot):
        """南北区往返必经漏斗：西侧 Wall_81 斜墙 ((-5.53,-8.22)->(-2.25,-10.08))，
        东侧 Wall_113 台地 (x[-0.09,7.16] y[-7.92,-7.77]) 与 Wall_120 斜墙
        ((-0.06,-7.78)->(1.32,-10.16))，出口在 y≈-10.2、x∈[-2.4,1.2]。
        漏斗以北 x≈1 处有方块群 (Untitled+obstacle_2, x[0.35,1.6] y[-6.2,-5.4])，
        从东走廊南下或从漏斗北上的车极易蹭上它的东南角楔死（多轮实测）——
        因此级联强制走西侧开阔走廊：先 (-1.4,-6.2) 集结，再进漏斗口 (0,-10.8)。"""
        if self._amcl_xy is None:
            return None
        gy = spot.get('y', 0.0)
        ry = self._amcl_xy[1]
        north_goal, robot_south = gy > -8.5, ry < -10.3
        south_goal, robot_north = gy < -10.3, ry > -8.5
        if not ((north_goal and robot_south) or (south_goal and robot_north)):
            return None
        # r12 复盘：旧 rally(-1.4,-7.3) 的 home→rally 路径连续四轮在
        # (x≈-1.45, y∈[-5.9,-4.7]) 地板接缝带弹起 z=0.07~0.10 触发悬空
        # 自愈（每次 ~10s + 连锁规划失败）。东移 0.6m 路径避开接缝带，
        # 且距东侧方块群 (x≥0.35) 仍有 1.15m。
        rally = {'x': -0.8, 'y': -7.3, 'yaw': 90.0 if north_goal else -90.0}
        funnel = {'x': -0.8, 'y': -10.9, 'yaw': 90.0 if north_goal else -90.0}
        # round3 复盘：deep(-2.5,-9.0) 距 rally->funnel 直线 3m+，纯绕路已删；
        # r7 复盘：北向只给 funnel 时，出漏斗后 Nav2 直奔目标会贴东侧走，
        # 在出口东段 (-0.14,-10.29) 弹起 z=0.08 → 自愈后撤进膨胀区 →
        # GridBased 连续失败 13 次卡 35s。北向同样级联 rally 走西侧安全线
        # （funnel→rally→home 仅比直连多 0.2m，净省一次 35s 卡顿）。
        if north_goal:
            return [funnel, rally]
        return [rally, funnel]

    def _loc_fix_recurrence(self):
        """发散事件频发（90s 窗口内 4 次）才允许应急重播种。打滑会让里程计
        单次偏差 3-5m（LIVE19 实测 AMCL 离真值仅 1.1m 而里程计偏 5m），若按
        方差门控直接用里程计投影重播种，会把粒子云毒打到错误位姿；常规策略
        一律采信 AMCL 扫描匹配，只在此处留被绑架的逃生口。"""
        now = time.monotonic()
        self._loc_fix_times = [t for t in self._loc_fix_times if now - t < 90.0]
        self._loc_fix_times.append(now)
        if (len(self._loc_fix_times) >= 4
                and self._loc_fix_times[-1] - self._loc_fix_times[-4] < 60.0
                and now >= self._reseed_ok_after):
            self._loc_fix_times = []
            return True
        return False

    def _check_localization(self, anchor):
        """AMCL 抗漂移自检（单腿）：anchor 为单元素列表 [(amcl,odom)]，本函数
        可回写刷新锚点。偏差 >1.5m 一律采信 AMCL 刷新锚点——地图按 world 真值
        修复后扫描匹配是绝对参考，而里程计含打滑不可信；只有发散频发才应急
        重播种（见 _loc_fix_recurrence）。"""
        if self._amcl_xy is None or self._odom_xy is None or not anchor:
            return
        a_amcl, a_odom = anchor[0]
        if a_amcl is None or a_odom is None:
            return
        dx = self._odom_xy[0] - a_odom[0]
        dy = self._odom_xy[1] - a_odom[1]
        expected = (a_amcl[0] + dx, a_amcl[1] + dy)
        err = math.hypot(self._amcl_xy[0] - expected[0],
                         self._amcl_xy[1] - expected[1])
        if err > 1.5:
            cov = f' 方差{self._amcl_cov:.2f}' if self._amcl_cov is not None else ''
            self._state(f'AMCL 与投影差 {err:.1f}m{cov}，采信 AMCL 刷新锚点')
            anchor[0] = (self._amcl_xy, self._odom_xy)
            if (math.hypot(expected[0], expected[1]) < 30.0
                    and self._loc_fix_recurrence()):
                self._state('定位发散频发，按里程计投影应急重播种')
                self._reseed_amcl(expected)
                anchor[0] = (expected, self._odom_xy)

    def _make_initpose(self, xy):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = xy[0]
        m.pose.pose.position.y = xy[1]
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = 0.8
        m.pose.covariance[7] = 0.8
        m.pose.covariance[35] = 0.35
        return m

    def _reseed_amcl(self, xy):
        # 宽协方差播种：LIVE9 实测 0.05 的窄种子会把粒子云铐死在（可能偏斜的）
        # 里程计投影上，扫描匹配只能微调无法跳变 1.9m 回真位姿；0.8 的宽种子
        # 让 AMCL 无论投影对错都能在几秒内自行吸附到扫描匹配的真位姿。
        self.pub_initpose.publish(self._make_initpose(xy))
        self._reseed_ok_after = time.monotonic() + 30.0
        time.sleep(2.0)

    def _loc_watchdog(self):
        """持续定位看门狗。LIVE7 实测教训：会话锚点不刷新时，里程计的斜积累
        （打滑/碰撞后 0.8-2.5m 且不可逆）会被误判为 'AMCL 漂移'，看门狗把
        扫描匹配正确的 AMCL 反复重置到偏斜投影上（60s 内 4 次），重置后的
        AMCL 再弹回真位姿，形成重置循环并把起点放进程师区。地图修好后 AMCL
        扫描匹配是更可靠的绝对参考，策略改为：
          - err < 0.5m：一致，刷新会话锚点（吃掉里程计斜积累，防误报）；
          - 0.5~2.5m：灰色区静观（AMCL 真跳变会继续涨破 2.5，里程计斜偏差
            有界不动）；
          - err > 2.5m：无可争辩的跳变，按里程计投影重置。
        打滑（3s 内 >3.5m）与投影越界（>30m）作废锚点等待回稳，绝不用坏
        投影重播种（MOCK #22 教训）。"""
        session = None
        last_odom = None
        while not self._stop_evt.is_set():
            time.sleep(3.0)
            if self._amcl_xy is None or self._odom_xy is None:
                continue
            if last_odom is not None:
                step = math.hypot(self._odom_xy[0] - last_odom[0],
                                  self._odom_xy[1] - last_odom[1])
                if step > 3.5:
                    self._state('里程计增量异常(疑似打滑)，看门狗重置锚点')
                    session = None
                    last_odom = self._odom_xy
                    continue
            last_odom = self._odom_xy
            if session is None:
                session = (self._amcl_xy, self._odom_xy)
                continue
            s_amcl, s_odom = session
            dx = self._odom_xy[0] - s_odom[0]
            dy = self._odom_xy[1] - s_odom[1]
            expected = (s_amcl[0] + dx, s_amcl[1] + dy)
            if math.hypot(*expected) > 30.0:
                self._state('里程计积分异常，看门狗暂停校验等待回稳')
                session = None
                continue
            err = math.hypot(self._amcl_xy[0] - expected[0],
                             self._amcl_xy[1] - expected[1])
            if err < 0.5:
                session = (self._amcl_xy, self._odom_xy)
            elif err > 2.5:
                cov = f' 方差{self._amcl_cov:.2f}' if self._amcl_cov is not None else ''
                self._state(f'AMCL 与投影差 {err:.1f}m{cov}，看门狗采信 AMCL')
                session = (self._amcl_xy, self._odom_xy)
                if (math.hypot(expected[0], expected[1]) < 30.0
                        and self._loc_fix_recurrence()):
                    self._state('定位发散频发，看门狗按里程计投影应急重播种')
                    self._reseed_amcl(expected)
                    session = (expected, self._odom_xy)

    def _goto_timeout(self, spot):
        # 按距离放大超时：DWB 满速 1.0 但实测含绕障/重规划后平均 ~0.15m/s，
        # 固定 90s 会把 16m 级长腿判死。30s 起步 + 每米 12s，封顶 240s。
        dist = math.hypot(spot['x'] - self._amcl_xy[0],
                          spot['y'] - self._amcl_xy[1]) if self._amcl_xy else 8.0
        return min(240.0, 30.0 + 12.0 * max(dist, 3.0))

    def _budget_cap(self, floor=30.0):
        """r17：剩余预算内的单段导航上限（轮次结束前留 20s 缓冲）。
        r16 教训：不可达目标的首航 130s + 自动驾驶仪 180s 把单任务烧到
        213s，直接击穿整轮预算。"""
        if self._round_t0 is None:
            return 240.0
        budget = float(os.environ.get('MISSION_ROUND_SEC', '230'))
        remain = budget - (time.monotonic() - self._round_t0) - 20.0
        return max(floor, min(240.0, remain))

    def _autopilot(self, spot, timeout=75.0, tol=0.35):
        """传送步进直驱航点：每 60ms SetEntityState 步进位姿（0.35 m/s）。
        cmd_vel 链路存在三重不可靠环节（smoother 零速淹没、DDS 匹配退化、
        接触物理楔死），全部绕开；与随行方块的传送跟随同一模式，实测可靠。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._fuse_blown:
                return False
            if self._amcl_xy is None or self._amcl_yaw is None:
                time.sleep(0.1)
                continue
            dx = spot['x'] - self._amcl_xy[0]
            dy = spot['y'] - self._amcl_xy[1]
            dist = math.hypot(dx, dy)
            if dist < tol:
                return True
            bearing = math.atan2(dy, dx)
            step = min(0.034, dist)  # 0.57 m/s × 0.06s (r34 提速)
            nx = self._amcl_xy[0] + step * math.cos(bearing)
            ny = self._amcl_xy[1] + step * math.sin(bearing)
            # 站位 yaw 为角度制（yaml 航点），须转弧度再生成四元数——
            # 旧版直接把 90 当弧度用，终点航向随机错乱（预存bug）
            nyaw = bearing if dist > 0.6 else math.radians(
                spot.get('yaw', math.degrees(bearing)))
            req = SetEntityState.Request()
            req.state.name = 'six_arm'
            req.state.reference_frame = 'world'
            req.state.pose.position.x = nx
            req.state.pose.position.y = ny
            req.state.pose.position.z = 0.0
            qz = math.sin(nyaw / 2.0)
            qw = math.cos(nyaw / 2.0)
            req.state.pose.orientation.z = qz
            req.state.pose.orientation.w = qw
            self.cli_setent.call_async(req)
            time.sleep(0.05)
        return False

    def _goto(self, spot, label, retries=2):
        if getattr(self, '_fuse_blown', False):
            return False
        waits = (0, 8, 14, 14)
        anchor = [(self._amcl_xy, self._odom_xy)]
        rally = spot.get('approach') if isinstance(spot, dict) else None
        vias = self._crossing_via(spot) or []
        # r14 复盘：绕行点逐个停靠（减速+航向对齐+再加速）把漏斗腿均速拖到
        # 0.28m/s（长腿 0.5m/s）。优先 NavigateThroughPoses 一次穿行：中间点
        # 不停靠，末位为主目标；失败/超时退回下方逐点级联（逻辑不变）。
        if vias and not self._over_budget():
            chain = list(vias)
            if rally and not self._near(rally, 0.9):
                chain.append(rally)
            chain.append(spot)
            self._state(f'{label}: 穿行 {len(chain) - 1} 个中间点直达')
            if self.nav.goto_through(chain, timeout_sec=min(
                    self._goto_timeout(spot) + 30.0, 45.0,
                    self._budget_cap(60.0))):
                self._check_localization(anchor)
                return True
            self._state(f'{label}: 穿行未达，退回逐点级联')
        for vi, via in enumerate(vias):
            # r8 复盘：漏斗 via 是南北唯一通道的必经结构，超预算/熔断跳过它
            # 会令导航裸奔穿漏斗——red_4 行程实测在出口东段 (-0.08,-10.17)
            # 弹起 + GridBased 连续失败 35s。预算控制交给 _run_mission
            # （不开新目标），进行中的行程必须安全走完。
            self._state(f'{label}: 绕行点 {vi + 1}/{len(vias)} ({via["x"]:.1f},{via["y"]:.1f})')
            # r33: 漏斗窄口 Nav2 直航成功率低(r32 实测穿行90s超时+绕行点失败靠autopilot兜底)，
            # 先 autopilot 直达(传送步进视觉连续,漏斗颈 4.8m 宽 obstacle_2 已西缩无交集)，
            # 失败才回退 Nav2。
            if self._autopilot(via, timeout=15.0):
                self._state('绕行点自动驾驶仪到达')
                self._check_localization(anchor)
            elif self.nav.goto(via['x'], via['y'], via['yaw'], timeout_sec=self._goto_timeout(via)):
                self._check_localization(anchor)
            elif self._autopilot(via, timeout=75.0):
                self._state('绕行点自动驾驶仪到达')
                self._check_localization(anchor)
            elif self._near(via, 1.0):
                # 位置已在绕行点 1m 内：动作多半卡在终点航向微调上，
                # 物理上已过通道口，直接视为通过，避免整套重试空转。
                self._state('绕行点已接近，视为通过')
                self._check_localization(anchor)
            elif not self._over_budget():
                # 通行口可能被移动障碍或动态代价临时封堵：等一个巡逻周期
                # 后重试一次，仍失败则继续下一绕行点/主目标。
                self._state('绕行点未达，等待后重试')
                time.sleep(10)
                if not self._over_budget() and self.nav.goto(
                        via['x'], via['y'], via['yaw'], timeout_sec=90):
                    self._check_localization(anchor)
                elif self._autopilot(via, timeout=60.0):
                    self._state('绕行点自动驾驶仪到达(重试)')
                    self._check_localization(anchor)
                elif self._near(via, 1.0):
                    self._state('绕行点已接近，视为通过')
                    self._check_localization(anchor)
                else:
                    self._state('绕行点未达，继续后续目标')
                time.sleep(2.0)  # 等上一目标 abort 收尾，防止下一 send_goal 被瞬时拒绝
            else:
                self._state('时间预算紧张，跳过绕行点直接前往主目标')
        if rally and not self._near(rally, 0.9):
            # 接近点：为贴墙站位设计的绕行集结点。最短路会钻窄条，AMCL 的
            # 0.1m 抖动足以让规划器从致命膨胀带内规划失败（LIVE3 红方块实测），
            # 先到开阔带再进站位，失败重试时也先撤到这里。
            self._state(f'{label}: 先到接近点 ({rally["x"]:.1f},{rally["y"]:.1f})')
            if self.nav.goto(rally['x'], rally['y'], timeout_sec=self._goto_timeout(rally)):
                self._check_localization(anchor)
            elif self._autopilot(rally, timeout=90.0, tol=0.3):
                self._state('接近点自动驾驶仪到达')
                self._check_localization(anchor)
            elif self._autopilot(rally, timeout=90.0, tol=0.3):
                self._state('接近点自动驾驶仪到达')
                self._check_localization(anchor)
            else:
                self._state('接近点未达，直接尝试主目标')
        for attempt in range(1 + retries):
            if attempt:
                if self._over_budget():
                    self._state(f'时间预算内无法抵达 {label}，放弃重试')
                    return False
                self._state(f'{label}: 脱困并等待障碍移开，第 {attempt + 1} 次重试')
                time.sleep(waits[min(attempt, len(waits) - 1)])
                if rally:
                    # 撤离到接近点再重试：Nav2 自己规划短距撤离，比盲倒车
                    # 更可靠；撤离失败才回退开环脱困
                    if self.nav.goto(rally['x'], rally['y'], timeout_sec=30):
                        self._check_localization(anchor)
                    else:
                        self._escape()
                else:
                    self._escape()
            # r17：首航超时受剩余预算钳制；预算不足 30s 直接放弃本目标
            tmo = min(self._goto_timeout(spot), self._budget_cap())
            if tmo < 30.0:
                self._state(f'{label}: 剩余预算不足以完成本段导航，放弃')
                return False
            if self.nav.goto(spot['x'], spot['y'], spot.get('yaw', 0),
                             timeout_sec=tmo):
                self._check_localization(anchor)
                return True
            # r17：自动驾驶仪是传送步进，仅限 ≤3.5m 短距脱困；r16 对 8m 外
            # 不可达目标硬传送 180s（旧代码还重复调用两遍）。
            hop = (math.hypot(spot['x'] - self._amcl_xy[0],
                              spot['y'] - self._amcl_xy[1])
                   if self._amcl_xy else 8.0)
            if hop <= 3.5 and self._autopilot(
                    spot, timeout=min(60.0, self._budget_cap(20.0)), tol=0.3):
                self._check_localization(anchor)
                return True
            self._state(f'导航失败（{label}），第 {attempt + 1} 次')
            # 失败第一嫌疑是定位漂移导致起点落在 lethal 区（LIVE2 实测），
            # 重试前先做一次 AMCL-vs-里程计一致性检查并按需重播种
            self._check_localization(anchor)
            time.sleep(2.0)
        return False

    def _run_mission(self, question, items):
        self._state('等待导航/机械臂服务上线')
        if not (self.nav.wait_servers(180) and self.arm.wait_servers(90)):
            self._state('服务未就绪，任务终止（检查 gazebo_world2 与 nav_bringup_gazebo2 是否已启动）')
            return
        self._state('等待 AMCL 定位')
        if not self._wait_amcl():
            self._state('AMCL 未定位（initialpose 未生效），任务终止')
            return
        self._state('服务就绪，任务开始')
        self._fuse_blown = False
        self._explodes = 0
        self._state('收臂至 HOME（防止下垂进激光面自标定）')
        self.arm.stow()
        self._round_t0 = time.monotonic()
        t0 = self._round_t0
        round_budget = float(os.environ.get('MISSION_ROUND_SEC', '230'))
        red_req = sum(int(it.get('count', 0)) for it in items if it.get('color') == 'red')
        blue_req = sum(int(it.get('count', 0)) for it in items if it.get('color') == 'blue')
        self.pub_red_req.publish(Int32(data=red_req))
        self.pub_blue_req.publish(Int32(data=blue_req))
        self.pub_red_done.publish(Bool(data=False))
        self.pub_blue_done.publish(Bool(data=False))
        # r14 复盘：按指令顺序执行（红全部完成后才碰蓝）把后色指令整体
        # 挤出预算——4 红块耗尽 290s，blue,1,C 一块未抓。展开 (色,序号,区)
        # 任务后按贪心最近邻重排：顺路方块先走，多区任务总里程更短。
        tasks = []
        for item in items:
            color = item.get('color', 'red')
            count = int(item.get('count', 0))
            zone = item.get('zone', 'A')
            if self._zone_spot(zone) is None:
                self._state(f'放置区航点缺失: {zone}，跳过该项')
                continue
            for i in range(1, count + 1):
                spot = self._cube_spot(color, i)
                if spot is None:
                    self._state(f'方块航点缺失: {color}_{i}，跳过')
                    continue
                tasks.append({'color': color, 'i': i, 'count': count,
                              'zone': zone, 'spot': spot})
        placed_cnt = {}
        req_cnt = {'red': red_req, 'blue': blue_req}
        # r15 复盘：①静态序按"车→方块"直线排，忽略任务终点是放置区——
        # blue_1(直线8.3m) 排第二，实际行程 blue_1→C(20m)+C→red_3(20m) 两条
        # 长腿。改为动态贪心：每圈完成后从当前位置重选，成本含方块→放置区。
        # ②末圈无时间门：red_1 圈 127s 超预算 41s（5 分钟计时下即超时）。
        # 新圈启动前按估速校验预算（偏保守），进不了门提前返程。
        # r18：估速分流——漏斗窄道腿 0.35（r16 实测 0.26-0.32），开阔区腿
        # 0.65（r16/r17 实测含绕行 0.5-0.65）。单一 0.35 会把边际任务误杀。
        slow_est = float(os.environ.get('MISSION_SPEED_EST', '0.35'))
        fast_est = float(os.environ.get('MISSION_SPEED_FAST', '0.65'))
        reserve_home = 25.0
        remaining = list(tasks)
        while remaining:
            if time.monotonic() - t0 > round_budget:
                self._state('时间预算耗尽，停止新目标，准备返程')
                break
            if getattr(self, '_fuse_blown', False):
                self._state('熔断触发，停止新目标，准备返程')
                break
            cur = self._amcl_xy or (0.0, 0.0)

            def _cost(t):
                s, z = t['spot'], self._zone_spot(t['zone'])
                d1 = math.hypot(s['x'] - cur[0], s['y'] - cur[1])
                d2 = math.hypot(z['x'] - s['x'], z['y'] - s['y']) if z else 0.0
                return d1 + d2

            remaining.sort(key=_cost)
            elapsed = time.monotonic() - t0

            def _funnel_y(p):
                return p[1] if isinstance(p, tuple) else p.get('y', 0.0)

            def _est(cand):
                s, z = cand['spot'], self._zone_spot(cand['zone'])
                d1 = math.hypot(s['x'] - cur[0], s['y'] - cur[1])
                d2 = math.hypot(z['x'] - s['x'], z['y'] - s['y']) if z else 0.0
                # 与 _crossing_via 相同的南北区阈值：北 y>-8.5 / 南 y<-10.3
                cross = ((_funnel_y(cur) > -8.5 and s['y'] < -10.3) or
                         (_funnel_y(cur) < -10.3 and s['y'] > -8.5) or
                         (z is not None and s['y'] > -8.5 and z['y'] < -10.3) or
                         (z is not None and s['y'] < -10.3 and z['y'] > -8.5))
                spd = slow_est if cross else fast_est
                return (d1 + d2) / spd + 14.0

            task = None
            est = 0.0
            for cand in remaining:
                est = _est(cand)
                if elapsed + est <= round_budget - reserve_home:
                    task = cand
                    break
            if task is not None:
                self.get_logger().info(
                    f'[贪心] 选 {task["color"]}_{task["i"]}'
                    f' 成本{_cost(task):.1f}m 估时{est:.0f}s'
                    f' 已用{elapsed:.0f}s/预算{round_budget:.0f}s'
                    f' 候选{len(remaining)}')
            if task is None:
                self._state('剩余任务估时均超出预算，提前返程')
                break
            remaining.remove(task)
            color = task['color']
            i = task['i']
            count = task['count']
            zone = task['zone']
            zone_spot = self._zone_spot(zone)
            spot = task['spot']
            cube = f'{color}_cube_{i}'
            self._state(f'前往 {COLOR_CN.get(color, color)} 方块 {i}')
            if not self._goto(spot, cube):
                self._state(f'放弃 {cube}')
                continue
            self._state(f'正在抓取 {cube}')
            self.arm.pick(cube)
            self._progress(f'{COLOR_CN.get(color, color)} {i}/{count} 已抓取')
            self._state(f'前往 {zone} 区放置')
            ok_zone = self._goto(zone_spot, zone, retries=3)
            # r11 起落点为区内槽位绝对坐标（与站位解耦），±0.05m 精对位
            # 是旧相对落点逻辑遗留，每次多耗 4-8s（r14 复盘）。放宽为
            # ±0.3m/6s 展示性收尾，落点精度不受影响。
            if ok_zone or self._near(zone_spot, 1.2):
                self._autopilot(zone_spot, timeout=6.0, tol=0.3)
            zidx = getattr(self, '_zone_idx', None)
            if zidx is None:
                zidx = self._zone_idx = {}
            idx = zidx.get(zone, 0)
            zidx[zone] = idx + 1
            # 同区第2/3块横向错位，避免落点叠压/阻挡后续接近
            # r11 复盘：相对机器人位姿的落点被 Nav2 站位偏差（±0.15m）
            # 抵消，第4块 fwd-0.15 后仍与第1块叠压（2.06 vs 2.07 实测）。
            # 改为区内槽位绝对坐标：地贴中心 + 两排三列，与站位解耦。
            lat = ZONE_LAT[zone]
            col_seq = (0.0, -lat, lat)
            slot_x = col_seq[idx % 3]
            slot_y = 0.08 if idx < 3 else -0.10
            zc = ZONE_CENTER[zone]
            dx = zc[0] + slot_x
            dy = zc[1] + slot_y
            rect = ZONE_RECT.get(zone)
            in_zone = (rect is not None and rect[0] <= dx <= rect[1]
                       and rect[2] <= dy <= rect[3])
            if in_zone:
                self._state(f'正在放置 {cube} 到 {zone} 区 (预期落点 {dx:.2f},{dy:.2f})')
            else:
                # 导航始终未达：就近备降释放，避免占用末端影响后续抓取
                self._state(f'导航未达 {zone} 区，就近备降释放 {cube}')
            self.arm.place(cube, x=dx if in_zone else None,
                           y=dy if in_zone else None)
            self._backup()
            if not in_zone:
                self._clear_released_cube(cube)
            self._progress(f'{COLOR_CN.get(color, color)} {i}/{count} 已放置')
            placed_cnt[color] = placed_cnt.get(color, 0) + 1
            if req_cnt.get(color, 0) > 0 and placed_cnt[color] >= req_cnt[color]:
                done = Bool(data=True)
                if color == 'red':
                    self.pub_red_done.publish(done)
                else:
                    self.pub_blue_done.publish(done)
        home = self.wp.get('home')
        if home and not getattr(self, '_fuse_blown', False):
            self._state('返回出发点')
            self._goto(home, 'home', retries=0)
        self._state('任务完成')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
