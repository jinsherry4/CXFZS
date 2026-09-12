#!/usr/bin/env bash
# 导航栈自愈：检查关键 nav2 节点，缺失的以组件形式补载进运行中的容器，
# 加载参数并手动完成生命周期转换（绕开 load_node 响应超时竞态）。
# 用法: bash ~/comp_tools/boot_repair.sh
PARAMS="$HOME/dev_ws/src/yzbot/bot_navigation/param/originbot_nav2_2.yaml"

source /opt/ros/humble/setup.bash
source "$HOME/dev_ws/install/setup.bash"

# 生成 FQN 键版本的参数文件（ros2 param load 要求 /node 形式键）
python3 - "$PARAMS" <<'PY'
import io, re, sys
src, dst = sys.argv[1], '/tmp/nav_params_fqn.yaml'
s = io.open(src, encoding='utf-8').read()
s = re.sub(r'^([a-z_][a-z0-9_]*):', r'/\1:', s, flags=re.M)
io.open(dst, 'w', encoding='utf-8').write(s)
PY

have() { timeout 8 ros2 node list 2>/dev/null | grep -qx "/$1"; }

load_comp() { # $1 节点名  $2 包名  $3 插件类
    if have "$1"; then return 0; fi
    echo "[repair] 补载 /$1"
    timeout 90 ros2 component load /nav2_container "$2" "$3" -s -n "$1" >/dev/null 2>&1 \
        || { echo "[repair] /$1 组件加载失败"; return 1; }
    sleep 1
    timeout 60 ros2 param load "/$1" /tmp/nav_params_fqn.yaml >/dev/null 2>&1 \
        || echo "[repair] /$1 参数加载失败（继续）"
}

wait_state() { # $1 节点名  $2 期望态前缀  $3 轮询次数
    for _ in $(seq 1 "${3:-5}"); do
        st=$(timeout 30 ros2 lifecycle get "/$1" 2>/dev/null)
        case "$st" in "$2"*) return 0;; esac
        sleep 2
    done
    return 1
}

transition() { # $1 节点名  $2 目标态(configure|activate)，确认到达否则重发一次
    timeout 90 ros2 lifecycle set "/$1" "$2" >/dev/null 2>&1
    wait_state "$1" "$2" 5 || {
        timeout 90 ros2 lifecycle set "/$1" "$2" >/dev/null 2>&1
        wait_state "$1" "$2" 5 || echo "[repair] /$1 未能到达 $2"
    }
}

load_comp map_server nav2_map_server nav2_map_server::MapServer
load_comp amcl nav2_amcl nav2_amcl::AmclNode
load_comp controller_server nav2_controller nav2_controller::ControllerServer
load_comp smoother_server nav2_smoother nav2_smoother::SmootherServer
load_comp planner_server nav2_planner nav2_planner::PlannerServer
load_comp behavior_server nav2_behaviors nav2_behaviors::BehaviorServer
load_comp bt_navigator nav2_bt_navigator nav2_bt_navigator::BtNavigator
load_comp waypoint_follower nav2_waypoint_follower nav2_waypoint_follower::WaypointFollower
load_comp velocity_smoother nav2_velocity_smoother nav2_velocity_smoother::VelocitySmoother

# 顺序生命周期转换（map/amcl → 控制规划行为 → bt 等）
for n in map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do
    if have "$n"; then
        st=$(timeout 30 ros2 lifecycle get "/$n" 2>/dev/null)
        case "$st" in
            unconfigured*) transition "$n" configure; transition "$n" activate ;;
            inactive*)     transition "$n" activate ;;
        esac
    fi
done

# AMCL 已激活但未必定位：补发初始位姿（0,0）
sleep 2
if have amcl && [ "$(timeout 15 ros2 topic info /amcl_pose 2>/dev/null | grep -o '[0-9]*$')" = "0" ]; then
    echo "[repair] 重发 /initialpose"
    timeout 20 ros2 topic pub -t 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}' >/dev/null 2>&1
fi

echo "[repair] 完成"
