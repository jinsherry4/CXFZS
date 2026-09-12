#!/bin/bash
# 快照：录 2 秒 → 抽帧 → scp 用
rm -f /tmp/snap.mp4 /tmp/snap_frame.png
NODE=$(cat /tmp/cast_node)
gst-launch-1.0 -e pipewiresrc path="$NODE" num-buffers=20 ! videoconvert ! videorate ! video/x-raw,framerate=10/1 ! x264enc speed-preset=ultrafast tune=zerolatency bitrate=2500 ! mp4mux ! filesink location=/tmp/snap.mp4 > /tmp/snap_gst.log 2>&1
ffmpeg -y -i /tmp/snap.mp4 -ss 1.5 -frames:v 1 /tmp/snap_frame.png 2>/dev/null
ls -la /tmp/snap_frame.png
