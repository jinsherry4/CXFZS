#!/bin/bash
# 面板全新实例（清会话恢复）+ 布局 + 关通知
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.L15AV3

pkill -x epiphany 2>/dev/null
for i in $(seq 1 20); do pgrep -x epiphany >/dev/null || break; sleep 0.5; done
pkill -9 -x epiphany 2>/dev/null
rm -rf ~/.local/share/epiphany
echo "epiphany reset"

GDK_BACKEND=x11 setsid nohup epiphany file:///home/ros/dashboard/dashboard.html >/tmp/epi_v3.log 2>&1 &
sleep 18
pgrep -x epiphany >/dev/null && echo EPI_ALIVE || { echo EPI_DEAD; cat /tmp/epi_v3.log; exit 1; }

# 关闭卡住的 colcon 通知（点空白处）
python3 /home/ros/inject_click.py 400 780 2>/dev/null || echo inject_click_failed
sleep 2

EPI=$(wmctrl -l | grep "指挥面板" | awk '{print $1}')
GZ=$(wmctrl -l | grep "Gazebo" | awk '{print $1}')
echo "epi=$EPI gz=$GZ"
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
fi
sleep 1
wmctrl -l -G | grep -E "Gazebo|指挥"
