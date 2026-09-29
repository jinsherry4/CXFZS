#!/bin/bash
# 重启任务链（不带移动障碍），供避障实证手动投障用
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
source ~/competition_env.sh 2>/dev/null
pkill -f '[m]ission.launch' 2>/dev/null
pkill -f '[m]ission_node' 2>/dev/null; pkill -f '[l]lm_parser' 2>/dev/null
pkill -f '[c]arry_follower' 2>/dev/null; pkill -f '[c]md_vel_watchdog' 2>/dev/null
pkill -f '[o]bstacle_mover' 2>/dev/null; pkill -f '[s]can_filter' 2>/dev/null
sleep 2
setsid nohup ros2 launch competition_bringup mission.launch.py \
  api_key:"$DEEPSEEK_API_KEY" obstacles:=false \
  >> /home/ros/comp_logs/mission_noobs.log 2>&1 </dev/null &
echo relaunched-noobs
