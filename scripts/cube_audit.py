#!/usr/bin/env python3
# 方块碰撞审计：before 快照存 json；after 对比，任何非豁免方块位移>3cm 即 FAIL
# 用法: python3 cube_audit.py before /tmp/cubes_before.json
#       python3 cube_audit.py after  /tmp/cubes_before.json red_cube_5 red_cube_1
import json, subprocess, sys

def snap():
    out = subprocess.run(
        'source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash'
        ' && timeout 15 ros2 topic echo /model_states --once',
        shell=True, executable='/bin/bash', capture_output=True, text=True).stdout
    names, poses, cur = [], [], None
    in_n, in_p = False, False
    for ln in out.splitlines():
        s = ln.strip()
        if s == 'name:': in_n, in_p = True, False; continue
        if s == 'pose:': in_n, in_p = False, True; continue
        if s.startswith('- position:') or s.startswith('position:') and in_p:
            cur = {}
        if in_n and s.startswith('- '):
            names.append(s[2:].strip("'"))
        elif in_p and cur is not None and (s.startswith('x:') or s.startswith('y:')):
            k, v = s.split(':', 1)
            cur[k] = float(v)
            if 'x' in cur and 'y' in cur:
                poses.append((cur['x'], cur['y'])); cur = None
    return {n: poses[i] for i, n in enumerate(names) if '_cube_' in n}

mode, path = sys.argv[1], sys.argv[2]
if mode == 'before':
    json.dump(snap(), open(path, 'w')); print('before-saved', len(json.load(open(path))))
else:
    exempt = set(sys.argv[3:])
    before, after = json.load(open(path)), snap()
    bad = 0
    for n, (x0, y0) in before.items():
        if n in exempt or n not in after: continue
        x1, y1 = after[n]
        d = ((x0-x1)**2 + (y0-y1)**2) ** 0.5
        flag = 'OK ' if d <= 0.03 else 'MOVED!'
        bad += d > 0.03
        print(f'{n}: 位移 {d:.3f}m {flag}')
    print('AUDIT-' + ('FAIL' if bad else 'PASS'))
