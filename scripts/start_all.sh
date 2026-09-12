#!/bin/bash
# 比赛一键启动 v3（2026-09-07 AMCL 毒化事故复盘版）
# 链路: Gazebo(v2地图) → Nav2(map_server 自愈型, 无AMCL) → 定位垫片 → rosbridge → 任务链
#
# v3 关键变更（对照 v2）：
#   1. 彻底删除 `ros2 lifecycle set /amcl shutdown` 循环——那会把 AMCL 打入
#      finalized 终态；此后任何 bond 心跳风暴（2 vCPU 饥荒 4s 即可触发）引发的
#      全量 reset 在 re-bringup 时必然卡死在 amcl configure → "Aborting bringup"
#      → map_server 永不激活 → global costmap 退化为 5x5 空图 → 整轮零抓取。
#      现在 nav_bringup_gazebo2.launch.py 根本不启动 AMCL（定位由
#      truth_odom + loc_shim 承担），reset 后 re-bringup 全链可成功，自愈闭环。
#   2. 固定 sleep 40 改为健康门：等 map_server 激活 + /map 真实尺寸（≥400 宽，
#      真图 658，空图默认 100）双条件，再放行定位垫片与任务链。
# 用法: bash ~/start_all.sh
set +u
export PYTHONUNBUFFERED=1   # nohup 重定向下日志实时可见
mkdir -p ~/comp_logs
source ~/competition_env.sh 2>/dev/null || true   # 可放 export DEEPSEEK_API_KEY=sk-xxx
[ "${MISSION_MOCK:-0}" = "1" ] && unset DEEPSEEK_API_KEY   # MOCK 联调模式：禁用真实API
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
if [ "${GAZEBO_HEADLESS:-0}" = "1" ]; then
  # 无桌面环境仍需 GL 上下文（摄像头传感器必需，否则 Rendering is disabled）。
  # 优先真 Xorg：vmwgfx→SVGA3D 硬件渲染走宿主GPU；Xvfb 的 GLX 只有 llvmpipe 软渲染，会吃满 2 个 vCPU。
  if ! DISPLAY=:0 timeout 8 glxinfo -B 2>/dev/null | grep -q SVGA3D; then
    ( nohup sudo -n Xorg :0 -noreset > /dev/null 2>&1 ) &
    sleep 5
  fi
  if DISPLAY=:0 timeout 8 glxinfo -B 2>/dev/null | grep -q SVGA3D; then
    export DISPLAY=:0
  else
    # Xorg 起不来（无3D/驱动缺失）才退回 Xvfb 软渲染
    if ! pgrep -x Xvfb > /dev/null; then
      ( nohup Xvfb :99 -screen 0 1280x1024x24 > /dev/null 2>&1 ) &
      sleep 2
    fi
    export DISPLAY=:99
  fi
  export RVIZ_ENABLE=0   # RViz 软件渲染吃掉大量 CPU，无头模式跳过
else
  export DISPLAY=${DISPLAY:-:0}
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

if [ "${SKIP_GAZEBO:-0}" != "1" ]; then
echo '[1/6] 关闭残留仿真与任务链进程(全量)...'
# 2026-09-07 round2 教训：只杀 gazebo 不杀任务链 → 双 mission_node 竞争；
# 旧 gzserver 未死透 → 新实例 bind 失败死亡 → 机器人/方块停在上一轮位置，
# 整轮在污染世界里执行。因此：全量杀 + 端口释放确认。
pkill -f 'ros2 [l]aunch' 2>/dev/null; sleep 1
pkill -f 'gz[s]erver' 2>/dev/null; pkill -f 'gz[c]lient' 2>/dev/null; pkill -f '[r]viz2' 2>/dev/null
pkill -f '[m]ission_node' 2>/dev/null; pkill -f '[l]lm_parser' 2>/dev/null; pkill -f '[o]bstacle_mover' 2>/dev/null
pkill -f '[c]md_vel_watchdog' 2>/dev/null; pkill -f '[c]arry_follower' 2>/dev/null; pkill -f '[q]uestion_bridge' 2>/dev/null
pkill -f '[t]ruth_odom' 2>/dev/null; pkill -f '[l]oc_shim' 2>/dev/null
pkill -f '[r]osbridge' 2>/dev/null; pkill -f '[r]osapi' 2>/dev/null
pkill -f '[c]omponent_container' 2>/dev/null; pkill -f '[r]obot_state_publisher' 2>/dev/null; pkill -f '[s]pawner' 2>/dev/null
sleep 1
# r10 复盘：rosbridge SIGTERM 后偶发不死（busy loop），r7-r10 实测每轮
# 残留一个实例累积吃 CPU（最多 4 个并存）。-9 补刀。
pkill -9 -f '[r]osbridge' 2>/dev/null; pkill -9 -f '[r]osapi' 2>/dev/null
sleep 2
# SIGTERM 后最多等 6s 让 gzserver 优雅退出（保存世界/断开传输），未退则 -9
for i in 1 2 3 4 5 6; do pgrep -f 'gz[s]erver' >/dev/null || break; sleep 1; done
pkill -9 -f 'gz[s]erver' 2>/dev/null
for i in 1 2 3 4 5; do pgrep -f 'gz[s]erver' >/dev/null || break; sleep 1; done
sleep 1

echo '[2/6] 启动 Gazebo(新地图)+机械臂控制器...'
nohup ros2 launch mybot gazebo_world2.launch.py > ~/comp_logs/gazebo.log 2>&1 &
sleep 20
# 新实例存活确认：bind 失败(端口被占)会在几秒内 process has died
if ! pgrep -f 'gz[s]erver' >/dev/null; then
  echo '  错误: gzserver 未存活——疑似 Gazebo master 端口被占，见 ~/comp_logs/gazebo.log'; exit 1
fi
GZCNT=$(pgrep -f 'gz[s]erver' | wc -l)
echo "  gzserver 存活 x${GZCNT} (应=1)"

if [ "${START_GAZEBO_ONLY:-0}" = "1" ]; then echo 'GAZEBO_ONLY: 机器人插入窗口完成，退出'; exit 0; fi
fi

echo '[3/6] 启动 Nav2(map_server+规划, AMCL已移除)...'
nohup ros2 launch bot_navigation nav_bringup_gazebo2.launch.py > ~/comp_logs/nav.log 2>&1 &

echo '[4/6] /map 健康门(map_server激活+真实尺寸) → 定位垫片 truth_odom+loc_shim...'
MAP_OK=0; ST=''; MW=''
for i in $(seq 1 45); do
  ST=$(timeout 6 ros2 lifecycle get /map_server 2>/dev/null | tail -1)
  if echo "$ST" | grep -q active; then
    # transient_local 探针读 latched /map 宽度（echo 无 --qos 参数，用 rclpy）
    MW=$(timeout 10 python3 -c "
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
rclpy.init()
n = Node('map_probe')
got = {}
n.create_subscription(OccupancyGrid, '/map',
    lambda m: got.update(w=m.info.width),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
import time
t0 = time.time()
while 'w' not in got and time.time() - t0 < 7:
    rclpy.spin_once(n, timeout_sec=200)
print(got.get('w', ''))
" 2>/dev/null | tail -1)
    if [ -n "$MW" ] && [ "$MW" -ge 400 ] 2>/dev/null; then
      MAP_OK=1; echo "  健康门通过: map_server=active, /map宽=${MW} (真实地图)"; break
    fi
  fi
  sleep 2
done
if [ "$MAP_OK" != "1" ]; then
  echo "  警告: /map 健康门 90s 未通过 (map_server=${ST:-无响应}, /map宽=${MW:-无})，继续启动但需人工核查 nav.log"
fi
( nohup python3 ~/truth_odom.py > /tmp/truth_odom.log 2>&1 ) &
( nohup python3 ~/loc_shim.py > /tmp/loc_shim.log 2>&1 ) &
sleep 3
# 出生点校验：非全新世界（机器人停在上一轮终点）会让整轮任务在错误前提执行
OX=''; OY=''
for oi in 1 2 3; do
  OX=$(timeout 8 ros2 topic echo /odom --once --field pose.pose.position.x 2>/dev/null | grep -oE '^-?[0-9.e-]+$' | head -1)
  OY=$(timeout 8 ros2 topic echo /odom --once --field pose.pose.position.y 2>/dev/null | grep -oE '^-?[0-9.e-]+$' | head -1)
  [ -n "$OX" ] && [ -n "$OY" ] && break
  sleep 2
done
if [ -n "$OX" ] && [ -n "$OY" ]; then
  ODM=$(python3 -c "import math;print(round(math.hypot($OX,$OY),2))" 2>/dev/null)
  if python3 -c "exit(0 if float('${ODM:-99}') < 2.0 else 1)" 2>/dev/null; then
    echo "  出生点校验通过 (|odom|=${ODM}m)"
  else
    echo "  警告: 机器人偏离出生点 ${ODM}m——世界可能未重置，本轮结果不可信！"
  fi
fi

echo '[5/6] 启动 rosbridge(coStudio 连 ws://本机IP:9090)...'
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > ~/comp_logs/rosbridge.log 2>&1 &

echo '[6/6] 启动任务链(调度+LLM+移动障碍)...'
if [ -n "${DEEPSEEK_API_KEY:-}" ]; then
  nohup ros2 launch competition_bringup mission.launch.py api_key:="$DEEPSEEK_API_KEY" obstacles:=true \
    > ~/comp_logs/mission.log 2>&1 &
else
  nohup ros2 launch competition_bringup mission.launch.py obstacles:=true \
    > ~/comp_logs/mission.log 2>&1 &
fi

sleep 3
echo '完成。日志: ~/comp_logs/*.log'
echo '自检: bash ~/comp_tools/boot_verify.sh'
echo '出题: ros2 run competition_bringup question_bridge'
echo '状态: ros2 topic echo /mission/work_state'
