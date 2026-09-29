#!/bin/bash
# ============================================================
#  task.sh — 一键发题并执行（正式链路）
#  /referee/text → 大模型解析 → /mission/command → 执行
#
#  用法（虚拟机终端）:
#    bash ~/task.sh '把4个红色方块放到A区，1个蓝色方块放到C区'   # 自然语言直述
#    bash ~/task.sh '小爱需要2只猫和5只狗…（应用题原文）'          # 应用题→大模型解题
#    bash ~/task.sh 1          # 快捷任务1..4（演示旁路, 跳过大模型）
#    bash ~/task.sh            # 无参数: 显示用法 + 当前状态
#
#  内置栈健康预检：仿真被关/服务掉线时自动恢复后再发题
# ============================================================
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ros/dev_ws/install/setup.bash 2>/dev/null

if [ -z "$1" ]; then
  echo "用法: bash ~/task.sh '题目文本'   或   bash ~/task.sh 1..4"
  echo "快捷任务: 1=red1→A+blue2→C  2=blue2→C+red1→B  3=red3→B  4=blue2→A"
  echo "---- 当前状态（最近 3 条）----"
  journalctl -u competition-mission -n 3 --no-pager 2>/dev/null | tail -3
  exit 0
fi

# ---- 预检 1：仿真世界（gzserver 缺失 = Gazebo 窗口被关 → 自动恢复）----
if ! pgrep -f 'gz[s]erver' >/dev/null; then
  echo "[预检] 仿真未运行（Gazebo 窗口可能被关闭）→ 自动执行 go.sh 恢复（约1-2分钟）..."
  bash /home/ros/go.sh > /tmp/go_restore.log 2>&1
  if ! pgrep -f 'gz[s]erver' >/dev/null; then
    echo "[错误] 仿真恢复失败，详见 /tmp/go_restore.log"; exit 1
  fi
  echo "[预检] 仿真已恢复"
fi

# ---- 预检 2：mission 服务 ----
if [ "$(systemctl is-active competition-mission 2>/dev/null)" != "active" ]; then
  echo "[预检] mission 服务未运行 → 启动中..."
  echo ros | sudo -S systemctl start competition-mission 2>/dev/null
  sleep 6
fi

echo "---- 发布前状态 ----"
journalctl -u competition-mission -n 1 --no-pager 2>/dev/null | tail -1

ros2 topic pub --once /referee/text std_msgs/msg/String "data: '$1'"
RC=$?
if [ $RC -ne 0 ]; then
  echo "[错误] 发布失败(rc=$RC)。检查: systemctl is-active competition-referee"
  exit $RC
fi

echo "已发布，等待解析(最多 10s)..."
sleep 8
echo "---- 裁判回执 ----"
journalctl -u competition-referee -n 3 --no-pager 2>/dev/null | grep '裁判回执' | tail -2
echo "---- 任务状态 ----"
journalctl -u competition-mission -n 4 --no-pager 2>/dev/null | grep '\[状态\]' | tail -3
echo ""
echo "实时观看: journalctl -u competition-mission -f | grep --line-buffered '\[状态\]'"