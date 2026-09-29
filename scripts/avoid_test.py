#!/usr/bin/env python3
# 避障实证脚本（2026-09-22）
# 事件B: 直驱途中把 obstacle_1 定身在机器人前方 → 取证 [r49] 超时 + 改走 Nav2
# 事件A: 后续航段再放障、6s 后撤走 → 取证 障碍已让开继续直驱
import math, time, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

LOG = sys.argv[1] if len(sys.argv) > 1 else '/home/ros/comp_logs/mission.log'
PARK = (-6.5, -2.0)   # 撤障停放点（远离主航路）

class T(Node):
    def __init__(self):
        super().__init__('avoid_test')
        self.odom = None
        self.obs_z = 0.05
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Odometry, '/odom', self._o, 10)
        self.create_subscription(ModelStates, '/model_states', self._ms, 10)
    def _o(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.z*q.z + q.y*q.y))
        self.odom = (p.x, p.y, yaw)
    def _ms(self, m):
        for i, n in enumerate(m.name):
            if n == 'obstacle_1':
                self.obs_z = m.pose[i].position.z

def set_obs(node, x, y):
    req = SetEntityState.Request()
    req.state = EntityState()
    req.state.name = 'obstacle_1'
    req.state.reference_frame = 'world'
    req.state.pose.position.x = x
    req.state.pose.position.y = y
    req.state.pose.position.z = node.obs_z
    req.state.pose.orientation.w = 1.0
    for _ in range(50):
        if node.cli.wait_for_service(timeout_sec=0.2):
            break
    f = node.cli.call_async(req)
    t0 = time.monotonic()
    while not f.done() and time.monotonic() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    return f.result() and f.result().success

def main():
    rclpy.init()
    node = T()
    rclpy.spin_once(node, timeout_sec=1.0)
    off = 0
    def newlines():
        nonlocal off
        try:
            with open(LOG, 'r', errors='ignore') as fh:
                fh.seek(off); data = fh.read()
                off = fh.tell()
        except Exception:
            data = ''
        return data
    newlines()  # 跳过头部
    state = 'B_pending'
    last_mode = 'ap'
    ev, t_start = {}, time.monotonic()
    t_re = 0.0
    leg_active = False
    last_xy = None
    print('[test] 等待直驱航段开始...', flush=True)
    while time.monotonic() - t_start < 360:
        rclpy.spin_once(node, timeout_sec=0.2)
        text = newlines()
        if '改走 Nav2' in text:
            last_mode = 'nav2'
        elif '[ap]' in text or '直驱' in text:
            last_mode = 'ap'
        for ln in text.splitlines():
            if '轮完成' in ln and state == 'A_hold':
                print('[test] 警告: 轮次在A取证前完成', flush=True); state = 'done'
        now_xy = node.odom
        moving = (last_xy and now_xy and
                  math.hypot(now_xy[0]-last_xy[0], now_xy[1]-last_xy[1]) > 0.15)
        if now_xy:
            lead = 1.0 if state in ('B_pending',) else 0.4
            ax = now_xy[0] + lead*math.cos(now_xy[2])
            ay = now_xy[1] + lead*math.sin(now_xy[2])
            if state == 'B_pending' and moving and leg_active:
                if set_obs(node, ax, ay):
                    print(f'[test] B: obstacle_1 定身 {ax:.2f},{ay:.2f}', flush=True)
                    state, t0 = 'B_hold', time.monotonic()
                    last_xy = None; continue
            if state == 'A_pending' and moving and last_mode == 'ap':
                if set_obs(node, ax, ay):
                    print(f'[test] A: obstacle_1 现身 {ax:.2f},{ay:.2f}', flush=True)
                    state, t0 = 'A_hold', time.monotonic()
                    last_xy = None; continue
        if '前往' in text:
            leg_active = True
        if any(k in text for k in ('到达', '抓取', '放置完成')):
            leg_active = False
        if state == 'B_hold':
            # 跟随重放(+0.5m, 1.2s 节奏)：曲线路径/转向都甩不出 0.75m 判定圈
            if now_xy and time.monotonic() - t_re > 1.2:
                set_obs(node, now_xy[0] + 0.5*math.cos(now_xy[2]),
                        now_xy[1] + 0.5*math.sin(now_xy[2]))
                t_re = time.monotonic()
            if time.monotonic() - t0 > 30:
                print('[test] B 超时窗口已过仍未见事件（检查日志）', flush=True)
                set_obs(node, *PARK); state = 'A_pending'
        if state == 'A_hold':
            if now_xy and time.monotonic() - t_re > 1.2:
                set_obs(node, now_xy[0] + 0.5*math.cos(now_xy[2]),
                        now_xy[1] + 0.5*math.sin(now_xy[2]))
                t_re = time.monotonic()
        if state == 'A_hold' and time.monotonic() - t0 > 6:
            set_obs(node, *PARK)
            print('[test] A: 障碍已撤走', flush=True)
            state, t0 = 'A_wait', time.monotonic()
        if state == 'A_wait' and time.monotonic() - t0 > 30:
            print('[test] A 未等到让开日志', flush=True); state = 'done'
        last_xy = node.odom
        for key, pat in (('B1','障碍未让开'), ('B2','改走 Nav2'),
                         ('A1','障碍已让开')):
            if key not in ev and pat in text:
                ev[key] = time.strftime('%H:%M:%S')
                print(f'[test] ★{key} 命中「{pat}」@{ev[key]}', flush=True)
        if 'B1' in ev and 'B2' in ev and state == 'B_hold':
            set_obs(node, *PARK); print('[test] B完成→撤障等下一航段做A', flush=True)
            state = 'A_pending'
        if 'A1' in ev:
            break
    set_obs(node, *PARK)
    print('[test] 事件汇总:', ev, flush=True)
    node.destroy_node(); rclpy.shutdown()
    sys.exit(0 if {'B1','B2','A1'} <= set(ev) else 1)

main()
