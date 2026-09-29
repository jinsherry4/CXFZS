#!/usr/bin/env python3
# r62：中→北直驱链前置（保留原晚段块作为级联后第二机会，不加守卫）
F = '/home/ros/dev_ws/src/yzbot/competition_bringup/competition_bringup/mission_node.py'
src = open(F, encoding='utf-8').read()

def rep(old, new, cnt=1):
    global src
    n = src.count(old)
    assert n == cnt, f'锚点 {n} 次(应{cnt}): {old[:56]}'
    src = src.replace(old, new)
    print('OK:', old.strip().splitlines()[0][:52])

# 1) 摘除 r61 诊断
rep("""            if i % 40 == 0:  # r61 诊断
                _ax, _ay = (self._amcl_xy or (float('nan'),) * 2)
                self.get_logger().info(
                    f'[r61dbg] i={i} rem={dist:.2f} cmd=({_cx:.2f},{_cy:.2f}) '
                    f'amcl=({_ax:.2f},{_ay:.2f})')
""", "")

# 2) 抽出方法 _mid2north（放在 _goto 定义前）
rep("    def _goto(self, spot, label, retries=2):",
    """    def _mid2north(self, spot, anchor):
        \"\"\"r62: 中→北直驱链（自 r42 块抽出前置）。北腿先走链(全段碰撞盒已核)，
        Nav2 迷宫腿 66~129s 只留作兜底。\"\"\"
        if not (self._amcl_xy and -8.5 < self._amcl_xy[1] < -3.0
                and spot.get('y', 0.0) > -3.0):
            return False
        chain_mn = [{'x': -2.6, 'y': -7.8, 'yaw': 90.0},
                    {'x': -1.0, 'y': -6.5, 'yaw': 90.0},
                    {'x': -1.0, 'y': -2.0, 'yaw': 90.0},
                    {'x': -0.8, 'y': 0.8, 'yaw': 90.0}]
        if self._seg_hits_box(-0.8, 0.8, spot['x'], spot['y'], self._W31M):
            chain_mn.append({'x': 3.9, 'y': 2.5, 'yaw': 0.0})
        chain_mn.append(spot)
        ok_mn = True
        for wp in chain_mn:
            segd = math.hypot(wp['x'] - self._amcl_xy[0],
                              wp['y'] - self._amcl_xy[1])
            if segd < 0.35:
                continue
            if not self._autopilot(wp, timeout=max(
                    15.0, min(90.0, segd / 0.2 + 15.0)), tol=0.4):
                ok_mn = False
                break
        time.sleep(0.6)
        if ok_mn and self._near(spot, 0.5):
            self._state('中→北直驱链到达')
            self._check_localization(anchor)
            return True
        if ok_mn and self._autopilot(spot, timeout=8.0, tol=0.4):
            self._state('中→北直驱链到达(直驱收尾)')
            self._check_localization(anchor)
            return True
        return False

    def _goto(self, spot, label, retries=2):""")

# 3) 尾段直驱失败后、dip 分支前 → 前置调用
rep("""                    self._state('漏斗尾段直驱到达')
                    self._check_localization(anchor)
                    return True
                if -8.5 < spot.get('y', 0.0) < -3.0:""",
    """                    self._state('漏斗尾段直驱到达')
                    self._check_localization(anchor)
                    return True
                if self._mid2north(spot, anchor):
                    return True
                if -8.5 < spot.get('y', 0.0) < -3.0:""")

open(F, 'w', encoding='utf-8').write(src)
print('R62-PATCHED')
