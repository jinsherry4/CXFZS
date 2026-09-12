#!/bin/bash
# 任务链启动器（VM 端执行，避免 SSH 引号转义坑）
# 用法: bash ~/relaunch_mission.sh
set +u
export PYTHONUNBUFFERED=1
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
source ~/competition_env.sh 2>/dev/null || true

pkill -f '[m]ission.launch'; pkill -f '[m]ission_node'; pkill -f '[o]bstacle_mover'
pkill -f '[l]lm_parser'; pkill -f '[c]md_vel_watchdog'; pkill -f '[q]uestion_bridge'
sleep 2

if [ -n "${DEEPSEEK_API_KEY:-}" ]; then
  nohup ros2 launch competition_bringup mission.launch.py \
    api_key:="$DEEPSEEK_API_KEY" obstacles:=true \
    > ~/comp_logs/mission_$(date +%H%M%S).log 2>&1 &
  echo "任务链已启动 (真实 API)"
else
  nohup ros2 launch competition_bringup mission.launch.py \
    obstacles:=true > ~/comp_logs/mission_$(date +%H%M%S).log 2>&1 &
  echo "任务链已启动 (MOCK: 无 API key)"
fi
sleep 5
pgrep -f '[m]ission_node' >/dev/null && echo MISSION_UP || echo MISSION_FAIL
pgrep -f '[o]bstacle_mover' >/dev/null && echo MOVER_UP || echo MOVER_FAIL
