#!/usr/bin/env python3
# r63：臂序列收紧 + 携带档提速
import os
D = '/home/ros/dev_ws/src/yzbot/competition_bringup/competition_bringup/'

def patch(f, pairs):
    src = open(f, encoding='utf-8').read()
    for old, new in pairs:
        n = src.count(old)
        assert n == 1, f'{os.path.basename(f)} 锚点 {n} 次: {old[:52]}'
        src = src.replace(old, new)
        print('OK:', old[:48])
    open(f, 'w', encoding='utf-8').write(src)

patch(D + 'arm_controller.py', [
    ("    def move_gripper(self, position, duration=0.45):",
     "    def move_gripper(self, position, duration=0.35):  # r63"),
    ("            self.move_arm(REACH, 0.9)",
     "            self.move_arm(REACH, 0.7)"),
    ("            self.move_arm(GRASP, 0.7)",
     "            self.move_arm(GRASP, 0.55)"),
    ("            self.move_arm(CARRY, 0.6)",
     "            self.move_arm(CARRY, 0.5)"),
    ("            self.move_arm(HOME, 1.1)",
     "            self.move_arm(HOME, 0.85)"),
])

patch(D + 'mission_node.py', [
    ("            step = min(0.18 if fast else 0.09, dist)",
     "            step = min(0.18 if fast else 0.11, dist)"),
    ("""                # 携带档: r60 60ms——model_states 10Hz 下 0.9m 步进方块跟随
                # 视觉滞后≤10cm 可接受；再快会台阶化
                _spent = time.monotonic() - _iter_t0
                time.sleep(max(0.02, 0.06 - _spent))""",
     """                # 携带档: r63 0.11m/50ms≈1.5-2m/s（model_states 10Hz 滞后
                # ≤15cm 仍可接受）
                _spent = time.monotonic() - _iter_t0
                time.sleep(max(0.02, 0.05 - _spent))"""),
])
print('R63-PATCHED')
