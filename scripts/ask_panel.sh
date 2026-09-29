#!/bin/bash
# 经裁判文本通道发题（走 systemd referee_node → DeepSeek llm_parser）
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
MSG="$1"
timeout 15 ros2 topic pub --once /referee/text std_msgs/msg/String "{data: \"$MSG\"}"
