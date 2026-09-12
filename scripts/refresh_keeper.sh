#!/bin/bash
# 重启 keeper（新 40 分钟窗口）→ 新 node
for p in $(pgrep -f "[c]ast_keeper"); do kill $p 2>/dev/null; done
sleep 2
setsid nohup python3 /home/ros/cast_keeper.py >/tmp/ck3.log 2>&1 &
sleep 7
cat /tmp/ck3.log
cat /tmp/cast_node 2>/dev/null && echo KEEPER_REFRESHED
