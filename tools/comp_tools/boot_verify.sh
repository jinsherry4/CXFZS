#!/usr/bin/env bash
# 启动自检 v2（2026-09-07 AMCL 移除版）：等待并验证关键节点/话题/机器人状态。
# v2 变更：AMCL 已从系统移除（truth_odom+loc_shim 承担定位）——
#   - 节点表: amcl → map_server
#   - 第5项: AMCL 定位检查 → 定位链健康（/map 真实尺寸 ≥400 宽，防 5x5 空图
#     事故复发；loc_shim /amcl_pose 与 /odom 一致性）。/initialpose 补发逻辑
#     一并删除（AMCL 不存在，补发无意义）。
# 用法: bash ~/comp_tools/boot_verify.sh [最大等待秒数，默认180]
MAX_WAIT="${1:-180}"
t0=$(date +%s)
pass() { echo "[VERIFY][OK] $1"; }
fail() { echo "[VERIFY][FAIL] $1"; }

source /opt/ros/humble/setup.bash 2>/dev/null
while true; do
    ok=1
    # 1. gzserver 进程
    if pgrep -f 'gz[s]erver' >/dev/null; then pass "gzserver 运行中"; else ok=0; fail "gzserver 未运行"; fi
    # 2. 关键节点（map_server 替代 amcl）
    NODES=$(timeout 12 ros2 node list 2>/dev/null)
    for n in map_server bt_navigator planner_server controller_server mission_node llm_parser; do
        if echo "$NODES" | grep -q "^/$n$"; then pass "节点 /$n 在线"; else ok=0; fail "节点 /$n 缺失"; fi
    done
    # 3. /scan 恰好 1 个发布者（防双重 spawn）
    NPUB=$(timeout 12 ros2 topic info /scan 2>/dev/null | grep -o 'Publisher count: [0-9]*' | grep -o '[0-9]*$')
    if [ "${NPUB:-0}" = "1" ]; then pass "/scan 发布者=1"; else ok=0; fail "/scan 发布者=${NPUB:-无}（应为1）"; fi
    # 4. 机器人本体完好：odom 必须在出生点附近且贴地（防出生瞬间被弹飞）
    ODOM=$(timeout 12 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null)
    OX=$(echo "$ODOM" | awk '/^x:/{print $2}')
    OY=$(echo "$ODOM" | awk '/^y:/{print $2}')
    OZ=$(echo "$ODOM" | awk '/^z:/{print $2}')
    if [ -n "$OX" ] && [ -n "$OY" ] && [ -n "$OZ" ]; then
        MAG=$(python3 -c "import math;print(math.hypot($OX,$OY))" 2>/dev/null)
        PASSZ=$(python3 -c "print($OZ < 0.5)" 2>/dev/null)
        if python3 -c "exit(0 if float('$MAG') < 2.0 else 1)" 2>/dev/null && [ "$PASSZ" = "True" ]; then
            pass "机器人本体完好 (odom=$OX,$OY,$OZ)"
        else
            ok=0; fail "机器人异常弹飞 (odom=$OX,$OY,$OZ)，需要重启"
        fi
    else
        ok=0; fail "/odom 无数据"
    fi
    # 5a. /map 真实尺寸（transient_local 探针；真图宽658，空图默认100——
    #     2026-09-06 事故中 map_server 未激活导致 costmap 5x5 空图整轮零抓取）
    MW=$(timeout 12 python3 -c "
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
while 'w' not in got and time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=200)
print(got.get('w', ''))
" 2>/dev/null | tail -1)
    if [ -n "$MW" ] && [ "$MW" -ge 400 ] 2>/dev/null; then
        pass "/map 真实地图已发布 (宽=${MW})"
    else
        ok=0; fail "/map 异常 (宽=${MW:-无发布})——疑似 map_server 未激活/空图，检查 nav.log"
    fi
    # 5b. loc_shim 定位一致性（/amcl_pose 由 loc_shim 从 /odom 转发，应恒一致）
    APUB=$(timeout 12 ros2 topic info /amcl_pose 2>/dev/null | grep -o 'Publisher count: [0-9]*' | grep -o '[0-9]*$')
    if [ "${APUB:-0}" -ge 1 ] 2>/dev/null; then
        AP=$(timeout 10 ros2 topic echo /amcl_pose --once --field pose.pose.position 2>/dev/null)
        AX=$(echo "$AP" | awk '/^x:/{print $2}')
        AY=$(echo "$AP" | awk '/^y:/{print $2}')
        if [ -n "$AX" ] && [ -n "$OX" ]; then
            DIFF=$(python3 -c "import math; print(round(math.hypot(($AX or 0)-($OX or 0), ($AY or 0)-($OY or 0)),2))" 2>/dev/null)
            if python3 -c "exit(0 if (${DIFF:-99}) < 1.0 else 1)" 2>/dev/null; then
                pass "loc_shim 定位与里程计一致 ($AX, $AY, 偏差${DIFF}m)"
            else
                ok=0; fail "loc_shim 与里程计偏差 ${DIFF}m（truth_odom/loc_shim 异常）"
            fi
        else
            ok=0; fail "/amcl_pose 无数据（loc_shim 未发布）"
        fi
    else
        ok=0; fail "/amcl_pose 无发布者（loc_shim 未运行）"
    fi
    if [ "$ok" = "1" ]; then
        echo "[VERIFY] 全部通过，用时 $(( $(date +%s) - t0 ))s"
        exit 0
    fi
    if [ $(( $(date +%s) - t0 )) -ge "$MAX_WAIT" ]; then
        echo "[VERIFY] 超时 $MAX_WAIT s，仍有失败项"
        exit 1
    fi
    sleep 5
done
