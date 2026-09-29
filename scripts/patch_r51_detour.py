#!/usr/bin/env python3
# r51：侧绕偏移 0.35→(0.55,0.85) 双档。0.35<判定圈0.42 导致单块必"无法侧绕"
import sys
F = '/home/ros/dev_ws/src/yzbot/competition_bringup/competition_bringup/mission_node.py'
src = open(F, encoding='utf-8').read()
old = """                det = None
                for sgn in (1.0, -1.0):
                    tx = nx - sgn * 0.35 * math.sin(bearing)
                    ty = ny + sgn * 0.35 * math.cos(bearing)
                    if not self._cube_blocking(tx, ty, 0.42, _spot_xy) and \\
                       not self._obstacle_blocking(tx, ty, 0.75):
                        det = (tx, ty)
                        break"""
new = """                det = None
                # r51: 偏移必须>判定圈0.42，单块在0.55档数学上必解（r50满负载
                # 实测 0.35<0.42 → 侧绕点仍在圈内 → 单块即卡死退Nav2白耗120s）
                for sgn in (1.0, -1.0):
                    for lat in (0.55, 0.85):
                        tx = nx - sgn * lat * math.sin(bearing)
                        ty = ny + sgn * lat * math.cos(bearing)
                        if not self._cube_blocking(tx, ty, 0.42, _spot_xy) and \\
                           not self._obstacle_blocking(tx, ty, 0.75):
                            det = (tx, ty)
                            break
                    if det:
                        break"""
assert src.count(old) == 1, f'锚点命中 {src.count(old)} 次'
src = src.replace(old, new)
open(F, 'w', encoding='utf-8').write(src)
print('R51-PATCHED')
