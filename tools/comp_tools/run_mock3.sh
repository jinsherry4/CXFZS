#!/bin/bash
# MOCK 出题：等任务链就绪后发布题目。用法: bash ~/comp_tools/run_mock3.sh
set +u
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash 2>/dev/null
pkill -f fine_monito[r] 2>/dev/null; sleep 1
nohup python3 ~/comp_tools/fine_monitor.py > /tmp/fine_mon.txt 2>&1 &
READY=0
for i in $(seq 1 24); do
  NODES=$(ros2 node list 2>/dev/null)
  echo "$NODES" | grep -q mission_node && echo "$NODES" | grep -q llm_parser && { READY=1; break; }
  sleep 5
done
echo "NODES_READY=$READY"
sleep 5
ros2 topic pub -w 1 --once /mission/question std_msgs/msg/String "{data: '需要搬运1个红色包裹到A区，2个蓝色包裹到C区'}" > /dev/null 2>&1
echo QUESTION_SENT
