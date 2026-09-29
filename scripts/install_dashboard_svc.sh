#!/bin/bash
set -e
# dashboard http.server systemd 化：开机自启+崩溃自愈（09-22 遗留③）
# heredoc 占住 tee 的 stdin，sudo -S 无法再读密码——先 -v 缓存凭证
echo ros | sudo -S -v
pkill -f '[h]ttp.server 8080' 2>/dev/null || true
sleep 1
sudo tee /etc/systemd/system/competition-dashboard.service > /dev/null <<'EOF'
[Unit]
Description=Competition dashboard http server (port 8080)
After=network.target
[Service]
User=ros
WorkingDirectory=/home/ros/dashboard
ExecStart=/usr/bin/python3 -m http.server 8080
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF
echo ros | sudo -S systemctl daemon-reload
echo ros | sudo -S systemctl enable --now competition-dashboard
sleep 3
systemctl is-active competition-dashboard
curl -s -o /dev/null -w "http:%{http_code}\n" http://127.0.0.1:8080/dashboard.html
