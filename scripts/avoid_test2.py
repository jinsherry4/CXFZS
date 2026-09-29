#!/usr/bin/env python3
# 高速档避障回归：obstacle_1 静态封漏斗口 → 必撞判定圈 → 取证 B1(超时)+B2(改Nav2)
# 前提：任务链以 obstacles:=false 启动（无巡航干扰）。撤障后等轮完成。
import math, time, sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

LOG = '/home/ros/comp_logs/mission_noobs.log'
MOUTH = (-0.9, -7.0)     # 漏斗口中心（via 链必经）
PARK = (-6.5, -2.0)

class T(Node):
    def __init__(self):
        super().__init__('avoid_test2')
        self.obs_z = 0.05
        self.obs_pos = None
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(ModelStates, '/model_states', self._ms, 10)
        from geometry_msgs.msg import Twist
        self.twist = Twist
        self.vel = self.create_publisher(Twist, '/obstacle_1/cmd_vel', 10)
    def zero_vel(self):
        self.vel.publish(self.twist())
    def _ms(self, m):
        for i, n in enumerate(m.name):
            if n == 'obstacle_1':
                p = m.pose[i].position
                self.obs_z = p.z
                self.obs_pos = (p.x, p.y, p.z)
    def set_obs(self, name, x, y):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.reference_frame = 'world'
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = self.obs_z   # 保持原高度，z=0 会被地面拒收
        req.state.pose.orientation.w = 1.0
        self.cli.wait_for_service(timeout_sec=3)
        f = self.cli.call_async(req)
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        ok = bool(f.result() and f.result().success)
        for _ in range(30):  # 回读验证落位
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.obs_pos and math.hypot(self.obs_pos[0]-x,
                                           self.obs_pos[1]-y) < 0.15:
                return True
        print(f'[t2] 落位验证失败 pos={self.obs_pos}', flush=True)
        return False

def main():
    rclpy.init(); node = T()
    for _ in range(40):  # 先收 model_states 拿真实 obs_z
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.obs_pos:
            break
    off = 0
    def newlines():
        nonlocal off
        try:
            with open(LOG, errors='ignore') as fh:
                fh.seek(off); d = fh.read(); off = fh.tell()
        except Exception:
            d = ''
        return d
    assert node.set_obs('obstacle_1', *MOUTH), '封门失败'
    print('[t2] obstacle_1 已封漏斗口', flush=True)
    newlines()
    ev, t0 = {}, time.monotonic()
    t_seal = 0.0
    while time.monotonic() - t0 < 240:
        rclpy.spin_once(node, timeout_sec=0.2)
        if 'B2' not in ev and time.monotonic() - t_seal > 2.0:
            node.zero_vel()
            node.set_obs('obstacle_1', *MOUTH)
            t_seal = time.monotonic()
        text = newlines()
        for k, pat in (('B1', '障碍未让开'), ('B2', '改走 Nav2'),
                       ('DONE', '轮完成')):
            if k not in ev and pat in text:
                ev[k] = time.strftime('%H:%M:%S')
                print(f'[t2] ★{k}「{pat}」@{ev[k]}', flush=True)
        if 'B2' in ev and 'B1' in ev:
            node.set_obs('obstacle_1', *PARK)
            print('[t2] 已撤障，等轮完成', flush=True)
        if 'DONE' in ev or ('B2' in ev and time.monotonic() - t0 > 150):
            break
    node.set_obs('obstacle_1', *PARK)
    ok = 'B1' in ev and 'B2' in ev
    print('[t2] 结果:', 'PASS' if ok else 'FAIL', ev, flush=True)
    node.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 1)

main()
