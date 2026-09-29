#!/bin/bash
# ============================================================
#  比赛一键启动（systemd 版 v5, 2026-09-29）
#  虚拟机终端执行:  bash ~/go.sh          # 无头（最快）
#                   bash ~/go.sh gui      # 额外弹出 RViz 导航视图
#                   bash ~/go.sh stop     # 一键停止全部服务
#                   bash ~/go.sh status   # 查看各服务状态
#
#  架构说明（与旧 start_all.sh 的区别）：
#    09-22 起全栈由 systemd 管理（9 个服务, 开机自启 + 崩溃自愈）。
#    旧 start_all.sh 会 pkill 后手动拉起 → 与 systemd 自愈冲突产生双实例，
#    已废弃；本脚本改为「按序重启服务 + 健康门等待」。
#
#  启动顺序: gazebo → nav → helpers → mission → referee
#            → rosbridge → dashboard → viz
# ============================================================
set +u
MODE=${1:-start}

SERVICES="competition-gazebo competition-nav competition-helpers \
competition-mission competition-referee competition-rosbridge \
competition-dashboard competition-viz"
STOP_ORDER="competition-mission competition-referee competition-viz \
competition-dashboard competition-rosbridge competition-helpers \
competition-nav competition-gazebo"

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ros/dev_ws/install/setup.bash 2>/dev/null

# ---------- 子命令 ----------
if [ "$MODE" = "stop" ]; then
  echo "[stop] 停止全部比赛服务..."
  echo ros | sudo -S -v >/dev/null 2>&1
  for s in $STOP_ORDER; do
    sudo systemctl stop "$s" 2>/dev/null && echo "  已停: $s"
  done
  # 兜底清理旧脚本残留的孤儿进程（不含 systemd 管理的）
  pkill -f '[r]viz2' 2>/dev/null
  echo "[stop] 完成。重新启动: bash ~/go.sh"
  exit 0
fi

if [ "$MODE" = "status" ]; then
  printf "%-34s %s\n" "服务" "状态"
  for s in $SERVICES competition-foxglove; do
    printf "%-34s %s\n" "$s" "$(systemctl is-active "$s" 2>/dev/null)"
  done
  echo ""
  echo "日志: journalctl -u competition-mission -f   (或 ~/comp_logs/*.log)"
  exit 0
fi

# ---------- 启动 ----------
echo ros | sudo -S -v >/dev/null 2>&1 || { echo "sudo 凭证失败"; exit 1; }

gate() {  # gate <名称> <超时秒> <检测命令...>
  local name=$1 tmo=$2; shift 2
  local t=0
  while [ $t -lt "$tmo" ]; do
    if "$@" >/dev/null 2>&1; then
      echo "  [OK] $name 就绪 (${t}s)"
      return 0
    fi
    sleep 3; t=$((t+3))
  done
  echo "  [警告] $name 等待超时 (${tmo}s)，继续后续步骤"
  return 1
}

wait_gazebo()   { pgrep -f 'gz[s]erver' >/dev/null && timeout 6 ros2 node list 2>/dev/null | grep -qi gazebo; }
wait_map()      { timeout 6 ros2 lifecycle get /map_server 2>/dev/null | grep -q active; }
wait_node()     { timeout 6 ros2 node list 2>/dev/null | grep -q "$1"; }
wait_port()     { ss -ltn 2>/dev/null | grep -q ":$1 "; }

echo "[1/4] 停止现有服务链（逆序）..."
for s in $STOP_ORDER; do
  sudo systemctl stop "$s" 2>/dev/null
done
sleep 3

echo "[2/4] 按序启动（含健康门）..."
sudo systemctl restart competition-gazebo
echo "  等待 Gazebo 世界加载..."
gate "Gazebo" 90 wait_gazebo

sudo systemctl restart competition-nav
echo "  等待 Nav2 map_server 激活..."
gate "Nav2" 90 wait_map

sudo systemctl restart competition-helpers
gate "helpers(定位垫片)" 30 wait_node truth_odom

for s in competition-mission competition-referee competition-rosbridge competition-dashboard competition-viz; do
  sudo systemctl restart "$s"
  sleep 1
done
echo "  等待任务链与可视化..."
gate "mission_node" 40 wait_node mission_node
gate "rosbridge:9090" 20 wait_port 9090
gate "dashboard:8080" 15 wait_port 8080
gate "viz_scene" 20 wait_node viz_scene

echo "[3/4] 服务状态:"
for s in $SERVICES; do
  printf "  %-34s %s\n" "$s" "$(systemctl is-active "$s" 2>/dev/null)"
done

if [ "$MODE" = "gui" ]; then
  echo "[4/4] 弹出 RViz 导航视图（Gazebo 界面随服务常开）..."
  bash /home/ros/show_gui.sh
else
  echo "[4/4] 无头模式（Gazebo 界面本身在服务里常开；RViz 需要时: bash ~/show_gui.sh）"
fi

echo ''
echo '========================================'
echo ' 启动完成。观看/操作入口：'
echo '  裁判输入台 : http://192.168.30.131:8080/input.html'
echo '  指挥面板   : http://192.168.30.131:8080/dashboard.html'
echo '  CoStudio   : ws://192.168.30.131:9090 (rosbridge)'
echo '  Foxglove   : ws://192.168.30.131:8765 (foxglove_bridge, 需: sudo systemctl start competition-foxglove)'
echo '  出任务     : 输入台点按钮 / 发布 {"data": "任务1"}'
echo '  状态查看   : bash ~/go.sh status'
echo '  一键停止   : bash ~/go.sh stop'
echo '========================================'