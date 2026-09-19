#!/bin/bash
# ============================================
#  比赛一键启动（虚拟机终端执行：bash ~/go.sh）
# ============================================
# 用法：
#   bash ~/go.sh        → 无头模式（最快，配合网页面板/CoStudio 观看）
#   bash ~/go.sh gui    → 同时弹出 Gazebo 3D + RViz 窗口（现场演示用）
#
# 启动内容：
#   Gazebo 场景 → Nav2 导航 → 定位垫片 → 相机压缩 → rosbridge(9090)
#   → foxglove_bridge(8765) → 任务链（DeepSeek 真实解析）
# 注：方块复位/真值里程计/定位垫片/图像压缩由 systemd 常驻服务自动管理。
# ============================================
set +u
MODE=${1:-headless}

# LLM 真实解析模式（读取 competition_env.sh 的 DeepSeek key）
unset MISSION_MOCK
source /home/ros/competition_env.sh 2>/dev/null

export GAZEBO_HEADLESS=1
if [ "$MODE" = "gui" ]; then
  unset GAZEBO_HEADLESS
fi

bash /home/ros/start_all.sh
RC=$?

if [ "$MODE" = "gui" ]; then
  echo '[GUI] 等待仿真就绪后弹出 Gazebo/RViz 窗口…'
  sleep 20
  bash /home/ros/show_gui.sh
fi

echo ''
echo '========================================'
echo ' 启动完成。观看入口：'
echo '  裁判输入台 : http://192.168.30.131:8080/input.html'
echo '  指挥面板   : http://192.168.30.131:8080/dashboard.html'
echo '  CoStudio   : ws://192.168.30.131:8765 (Foxglove WebSocket)'
echo ' 出任务     : 输入台点按钮 / 发布 {"data": "任务1"}'
echo ' 状态查看   : tail -f ~/comp_logs/mission.log'
echo ' 一键停止   : bash ~/comp_stop.sh'
echo '========================================'
exit $RC
