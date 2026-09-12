#!/bin/bash
# 三窗口布局：面板左半 / Gazebo 右半 / RViz 移出屏
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.L15AV3

EPI=$(wmctrl -l | grep "指挥面板" | awk '{print $1}')
GZ=$(wmctrl -l | grep "Gazebo" | awk '{print $1}')
RV=$(wmctrl -l | grep "RViz" | awk '{print $1}')
echo "epi=$EPI gz=$GZ rv=$RV"

if [ -n "$RV" ]; then
  wmctrl -i -r "$RV" -b remove,maximized_vert,maximized_horz
  sleep 0.5
  wmctrl -i -r "$RV" -e 0,2600,2600,600,400
  sleep 0.5
fi
if [ -n "$GZ" ]; then
  wmctrl -i -r "$GZ" -b remove,maximized_vert,maximized_horz
  sleep 0.5
  wmctrl -i -r "$GZ" -e 0,956,27,962,851
  sleep 0.5
fi
if [ -n "$EPI" ]; then
  wmctrl -i -r "$EPI" -b remove,maximized_vert,maximized_horz
  sleep 0.5
  wmctrl -i -r "$EPI" -e 0,0,27,956,851
  sleep 0.5
fi
wmctrl -l -G | grep -E "Gazebo|RViz|指挥"
