#!/bin/bash
# 强化版全量停止：覆盖新增节点，先杀 launch 父进程再杀孤儿，-9 兜底
pkill -f "ros2 launch" 2>/dev/null
sleep 1
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
pkill -f component_container 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f spawner 2>/dev/null
pkill -f mission_node 2>/dev/null
pkill -f llm_parser 2>/dev/null
pkill -f obstacle_mover 2>/dev/null
pkill -f cmd_vel_watchdog 2>/dev/null
pkill -f carry_follower 2>/dev/null
pkill -f question_bridge 2>/dev/null
pkill -f scan_filter 2>/dev/null
pkill -f fine_monitor 2>/dev/null
pkill -f set_initial_pose 2>/dev/null
pkill -f rosbridge 2>/dev/null
pkill -f rosapi_node 2>/dev/null
pkill -f rviz2 2>/dev/null
sleep 2
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f component_container 2>/dev/null
pkill -9 -f carry_follower 2>/dev/null
pkill -9 -f mission_node 2>/dev/null
pkill -9 -f obstacle_mover 2>/dev/null
pkill -9 -f cmd_vel_watchdog 2>/dev/null
pkill -9 -f llm_parser 2>/dev/null
pkill -9 -f fine_monitor 2>/dev/null
pkill -9 -f rosbridge 2>/dev/null
pkill -9 -f scan_filter 2>/dev/null
sleep 1
LEFT=$(ps aux | grep -cE 'gzserve[r]|mission_nod[e]|carry_follo[wer]|rosbridg[e]|fine_monito[r]|obstacle_move[r]|cmd_vel_watchdo[g]|llm_parse[r]')
echo "remaining: $LEFT"
