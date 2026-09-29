#!/bin/bash
# 三个常驻辅助进程统一由本脚本拉起（任一崩溃由 systemd 重启整个组）
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
export PYTHONUNBUFFERED=1

python3 /home/ros/truth_odom.py &
P1=$!
python3 /home/ros/loc_shim.py &
P2=$!
python3 /home/ros/cam_jpeg.py &
python3 /home/ros/map_repub.py &
P4=$!
P3=$!

# 任一子进程退出即退出（systemd 会重启全部）
wait -n $P1 $P2 $P3 $P4
exit 1
