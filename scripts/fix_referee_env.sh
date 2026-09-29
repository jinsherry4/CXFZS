#!/bin/bash
set -e
# 修复 referee 服务拿不到 API Key：EnvironmentFile 不支持 export 前缀写法
sed -E 's/^export //' /home/ros/competition_env.sh > /home/ros/competition_referee.env
chmod 600 /home/ros/competition_referee.env
grep -c '^DEEPSEEK_API_KEY=sk' /home/ros/competition_referee.env >/dev/null && echo "env-file-ok"
echo ros | sudo -S sed -i 's|EnvironmentFile=.*|EnvironmentFile=/home/ros/competition_referee.env|' /etc/systemd/system/competition-referee.service
echo ros | sudo -S systemctl daemon-reload
echo ros | sudo -S systemctl restart competition-referee
sleep 4
PID=$(pgrep -f referee_entry.py | head -1)
echo ros | sudo -S tr '\0' '\n' < /proc/$PID/environ | grep -q '^DEEPSEEK_API_KEY=sk' && echo "service-has-key" || echo "service-missing-key"
systemctl is-active competition-referee
