#!/usr/bin/env python3
# 满负载轮阶段画像：从 mission.log 最后一轮窗口提取关键事件时间线
import re, sys

path = '/home/ros/comp_logs/mission.log'
lines = open(path, errors='ignore').read().splitlines()
# 定位最后一次"方块已复位"→"轮完成"窗口
start = max(i for i, l in enumerate(lines) if '复位' in l)
end = next(i for i in range(start, len(lines)) if '轮完成' in lines[i])
win = lines[start:end + 1]

KEY = re.compile(
    r'\[(\d{10})\.\d+\].*(贪心\] 选 \S+|前往 \S+ 方块 \d+|正在抓取 (\S+)|'
    r'张开夹爪|探向方块|闭合夹爪|收臂至随行位|启动随行搬运|前往 (\w) 区放置|'
    r'落地槽位|返回出发点|轮完成|复位|穿行|直驱到达|直驱完成|同带|dip|'
    r'\[ap\] OK|看门狗重置|主目标未达|级联|绕行点|HOME)')
ev = []
for l in win:
    m = KEY.search(l)
    if m:
        t = int(m.group(1))
        ev.append((t, m.group(2) or m.group(0)[-24:]))
t0 = ev[0][0]
prev = t0
for t, tag in ev:
    print(f'{t - t0:6d}s (+{t - prev:4d}) {tag}')
    prev = t
print('TOTAL', prev - t0, 's')
