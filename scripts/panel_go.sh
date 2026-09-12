#!/bin/bash
# 真面板启动（WebKit 禁合成模式修复白屏）+ 布局
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.L15AV3
export WEBKIT_DISABLE_COMPOSITING_MODE=1

# 等 epiphany 死透
for i in $(seq 1 20); do pgrep -f "[e]piphany" >/dev/null || break; sleep 0.5; done

GDK_BACKEND=x11 setsid nohup /usr/bin/epiphany-browser http://localhost:8080/dashboard.html >/tmp/epi_ok.log 2>&1 &
sleep 15
pgrep -f "[e]piphany" >/dev/null && echo EPI_ALIVE || echo EPI_DEAD
wmctrl -l
bash ~/layout_all.sh 2>&1 | tail -3
