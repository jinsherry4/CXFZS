#!/bin/bash
# 分阶段重启(MOCK 模式)：Gazebo 先行，等机器人插入且物理稳定（必要时传送回原点），
# 再启动 Nav/任务链。避免导航容器在机器人插入窗口抢 CPU 导致生成竞态/弹飞。
# 用法: bash ~/comp_tools/boot_mock.sh
bash ~/comp_stop.sh
sleep 1
export GAZEBO_HEADLESS=1
export MISSION_ROUND_SEC=420   # MOCK 验证模式放宽到 420s,正式比赛默认 280s

# 阶段1：仅启动 Gazebo+机器人（start_all 走完 [1/6][2/6] 后自行退出）
START_GAZEBO_ONLY=1 bash ~/start_all.sh > ~/comp_logs/start_gazebo.log 2>&1

# 阶段2：等机器人出现并静止在出生点；弹飞则 reset_simulation 清速度（只设位姿
# 不清 twist，残余速度会让弹飞传送死循环），连续 3 次异常则整段重开 Gazebo
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
stable=0
anom=0
for i in $(seq 1 90); do
  ODOM=$(timeout 8 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null)
  OX=$(echo "$ODOM" | awk '/^x:/{print $2}')
  OY=$(echo "$ODOM" | awk '/^y:/{print $2}')
  OZ=$(echo "$ODOM" | awk '/^z:/{print $2}')
  if [ -n "$OX" ] && [ -n "$OY" ] && [ -n "$OZ" ]; then
    BAD=$(python3 -c "import math;print(1 if math.hypot($OX,$OY)>1.5 or abs($OZ)>0.5 else 0)" 2>/dev/null)
    if [ "$BAD" = "1" ]; then
      anom=$((anom+1))
      echo "[boot] 机器人异常(odom=$OX,$OY,$OZ)，第${anom}次恢复"
      if [ "$anom" -ge 3 ]; then
        echo '[boot] 连续3次异常，整段重开 Gazebo'
        bash ~/comp_stop.sh
        sleep 2
        START_GAZEBO_ONLY=1 bash ~/start_all.sh > ~/comp_logs/start_gazebo.log 2>&1
        anom=0
        continue
      fi
      timeout 10 ros2 service call /gazebo/reset_simulation std_srvs/srv/Empty \
        >/dev/null 2>&1
      sleep 2
      timeout 10 ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
        "{state: {name: six_arm, reference_frame: world, pose: {position: {x: 0.0, y: 0.0, z: 0.05}}}}" >/dev/null 2>&1
      sleep 2
      continue
    fi
    echo "[boot] 机器人稳定 (odom=$OX,$OY,$OZ)"
    stable=1
    break
  fi
  sleep 2
done
if [ "$stable" != "1" ]; then
  echo '[boot] 机器人 180s 内未稳定，中止启动'
  exit 1
fi

# 阶段3：Nav2 + AMCL 初始位姿 + rosbridge + 任务链，随后自动自愈+自检
SKIP_GAZEBO=1 nohup bash ~/start_all.sh > ~/comp_logs/start_all.log 2>&1 &
SKIP_GAZEBO=1 nohup bash -c 'sleep 50; bash ~/comp_tools/boot_repair.sh; bash ~/comp_tools/boot_verify.sh 150' \
  > ~/comp_logs/boot_repair.log 2>&1 &
echo BOOTING_STAGED
