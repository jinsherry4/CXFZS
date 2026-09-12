#!/bin/bash
# 面板终极清场版：彻底杀 + 清 profile + HTTP 单窗口
export DISPLAY=:0
# Xwayland auth 文件每次会话随机变化，动态获取（获取失败则不设，实测不设也能用）
_XAUTH=$(pgrep -af Xwayland | grep -oP '(?<=-auth )\S+' | head -1)
[ -n "$_XAUTH" ] && [ -f "$_XAUTH" ] && export XAUTHORITY=$_XAUTH

# 1. 确认 HTTP 服务活着
curl -s -o /dev/null -m 3 http://localhost:8080/dashboard.html || {
  cd ~/dashboard && setsid nohup python3 -m http.server 8080 >/tmp/httpd.log 2>&1 &
  sleep 2
}
curl -s -o /dev/null -m 3 -w "http=%{http_code}\n" http://localhost:8080/dashboard.html

# 1.5 杀 RViz（nav launch 默认启动，演示布局不需要）
pkill -f '[r]viz2' 2>/dev/null; sleep 1; pkill -9 -f '[r]viz2' 2>/dev/null
pgrep -f '[r]viz2' >/dev/null && echo RVIZ_STILL_ALIVE || echo RVIZ_DEAD

# 2. 彻底杀 epiphany（等死透）
pkill -x epiphany 2>/dev/null
for i in $(seq 1 30); do pgrep -x epiphany >/dev/null || break; sleep 0.5; done
pkill -9 -x epiphany 2>/dev/null
sleep 1
pgrep -x epiphany >/dev/null && echo EPI_STILL_ALIVE || echo EPI_DEAD

# 3. 清 profile（去 session 恢复）
rm -rf ~/.local/share/epiphany

# 4. 单窗口启动
GDK_BACKEND=x11 setsid nohup epiphany --new-window http://localhost:8080/dashboard.html >/tmp/epi_z.log 2>&1 &
sleep 16
pgrep -x epiphany >/dev/null && echo EPI_ALIVE || { echo EPI_DEAD; cat /tmp/epi_z.log; exit 1; }
DISPLAY=:0 wmctrl -l | grep -v -E "Gazebo|RViz"

# 5. 布局
bash ~/layout_all.sh 2>&1 | tail -3
