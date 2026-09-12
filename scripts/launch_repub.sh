#!/bin/bash
# 干净启动 v2 转发器 + 清残留 scan_filte
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

# 清残留 scan_filter（保留最新的 21855，杀旧的）
for pid in 4992 5938 7406 12802; do
  kill $pid 2>/dev/null && echo "killed stale scan_filter $pid"
done

# 启动 v2
setsid nohup python3 /home/ros/map_repub.py >/tmp/mr2.log 2>&1 &
sleep 4
tail -2 /tmp/mr2.log
