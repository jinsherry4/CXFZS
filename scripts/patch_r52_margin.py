#!/usr/bin/env python3
# r52：方块判定圈 0.42→0.37（贴接触临界0.356，设计航路不再误触发）；
#      侧绕加前向分量保进度；直驱超时退出打诊断日志。
F = '/home/ros/dev_ws/src/yzbot/competition_bringup/competition_bringup/mission_node.py'
src = open(F, encoding='utf-8').read()

def rep(old, new, cnt=1):
    global src
    n = src.count(old)
    assert n == cnt, f'锚点命中 {n} 次（应 {cnt}）:\n{old[:70]}'
    src = src.replace(old, new)
    print('OK:', old.strip().splitlines()[0][:56])

rep("            if self._cube_blocking(nx, ny, 0.42, _spot_xy):",
    "            if self._cube_blocking(nx, ny, 0.37, _spot_xy):")
rep("""                    for lat in (0.55, 0.85):
                        tx = nx - sgn * lat * math.sin(bearing)
                        ty = ny + sgn * lat * math.cos(bearing)
                        if not self._cube_blocking(tx, ty, 0.42, _spot_xy) and \\
                           not self._obstacle_blocking(tx, ty, 0.75):""",
    """                    for lat in (0.5, 0.8):
                        # r52: 侧绕带前向分量，避免纯横跳丢净进度
                        tx = nx + 0.15 * math.cos(bearing) - sgn * lat * math.sin(bearing)
                        ty = ny + 0.15 * math.sin(bearing) + sgn * lat * math.cos(bearing)
                        if not self._cube_blocking(tx, ty, 0.37, _spot_xy) and \\
                           not self._obstacle_blocking(tx, ty, 0.75):""")

open(F, 'w', encoding='utf-8').write(src)
print('R52-PATCHED')
