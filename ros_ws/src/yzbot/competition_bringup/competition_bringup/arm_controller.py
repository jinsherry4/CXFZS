# -*- coding: utf-8 -*-
"""机械臂控制器：关节轨迹 + 夹爪 + 虚拟搬运。

关节数值来源：官方脚本 my_send_goal.sh / pick_on.sh / pick_off.sh。
搬运逻辑：不再使用 gazebo 链接绑定——物理焊接(fixed joint)与底盘
planar_move 插件的每步 SetLinearVel 速度覆写相冲突，起步瞬间即令 ODE
约束求解发散（实测整机被抛飞百米）。改为通知 carry_follower 以 20Hz
直接设置方块位姿跟随底盘，方块不参与约束求解，搬运全程物理确定。
时序针对 5 分钟赛制压缩：夹爪单点 0.6s，臂段 1.0~2.0s。
"""
import time

from control_msgs.action import FollowJointTrajectory
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIP_JOINTS = ['finger_joint1']

# 关节预设（来自官方脚本）
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
REACH = [0.0, 0.9, 1.17, 0.0, -0.3, 0.0]     # 探向方块
GRASP = [0.0, 1.05, 1.17, 0.0, -0.3, 0.0]    # 下压抓取(不宜过深,夹爪会触地)
CARRY = [0.0, 0.55, 0.9, 0.0, -0.3, 0.0]     # 随行搬运位:夹爪离地,严禁拄地行驶
GRIP_OPEN = 0.3
GRIP_CLOSED = 0.0


class ArmController:
    """阻塞式封装：在 mission 线程里顺序调用，内部用事件等待，不卡 executor。"""

    def __init__(self, node: Node):
        self.node = node
        self._arm_ac = None
        self._grip_ac = None

    def wait_servers(self, timeout_sec=30.0):
        """等 action 服务器上线（在调用前执行一次）。"""
        import rclpy
        end = time.time() + timeout_sec
        while time.time() < end and rclpy.ok():
            self._arm_ac = self._arm_ac or self._make_arm()
            self._grip_ac = self._grip_ac or self._make_grip()
            if (self._arm_ac.wait_for_server(timeout_sec=0.5)
                    and self._grip_ac.wait_for_server(timeout_sec=0.5)):
                return True
        return not (self._arm_ac is None)

    def _make_arm(self):
        from rclpy.action import ActionClient
        return ActionClient(self.node, FollowJointTrajectory,
                            '/arm_controller/follow_joint_trajectory')

    def _make_grip(self):
        from rclpy.action import ActionClient
        return ActionClient(self.node, FollowJointTrajectory,
                            '/gripper_controller/follow_joint_trajectory')

    # ---------- 内部工具 ----------
    @staticmethod
    def _wait_future(node, future, timeout):
        import threading
        done = threading.Event()

        def _cb(_):
            done.set()

        future.add_done_callback(_cb)
        return done.wait(timeout)

    def _send_traj(self, client, joint_names, positions, duration):
        from builtin_interfaces.msg import Duration
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points.append(pt)
        if not client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('action server 未上线')
            return False
        fut = client.send_goal_async(goal)
        if not self._wait_future(self.node, fut, 10.0):
            return False
        handle = fut.result()
        if not handle.accepted:
            return False
        res_fut = handle.get_result_async()
        ok = self._wait_future(self.node, res_fut, duration + 15.0)
        return ok and res_fut.result().status == 4  # SUCCEEDED

    # ---------- 对外接口 ----------
    def move_arm(self, positions, duration=2.0):
        return self._send_traj(self._arm_ac, ARM_JOINTS, positions, duration)

    def move_gripper(self, position, duration=0.6):
        """夹爪开（0.3）/ 闭（0.0），单点轨迹。"""
        from builtin_interfaces.msg import Duration
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIP_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points.append(pt)
        if not self._grip_ac.wait_for_server(timeout_sec=5.0):
            return False
        fut = self._grip_ac.send_goal_async(goal)
        if not self._wait_future(self.node, fut, 10.0):
            return False
        handle = fut.result()
        if not handle.accepted:
            return False
        res_fut = handle.get_result_async()
        ok = self._wait_future(self.node, res_fut, duration + 10.0)
        return ok

    # ---------- 组合动作 ----------
    def stow(self):
        """收臂到 HOME。开机后机械臂因无轨迹指令会重力下垂进激光面，
        激光扫到自身部件会在代价地图上把机器人自身标成致命障碍
        （规划起点直接落入 lethal），导航因此全面失败——启动必须先收臂。"""
        return self.move_arm(HOME, 2.0)

    def pick(self, cube_name: str):
        """张开→探下→下压→闭合→收臂到随行位→启动虚拟搬运。
        必须先收臂再行车：GRASP 深位夹爪会触地，拄地行驶会触发 ODE 弹射。"""
        log = self.node.get_logger()
        log.info(f'[arm] 张开夹爪准备抓取 {cube_name}')
        self.move_gripper(GRIP_OPEN)
        log.info('[arm] 探向方块')
        self.move_arm(REACH, 1.2)
        self.move_arm(GRASP, 0.9)
        log.info('[arm] 闭合夹爪')
        self.move_gripper(GRIP_CLOSED)
        log.info('[arm] 收臂至随行位')
        self.move_arm(CARRY, 0.8)
        log.info('[arm] 启动随行搬运')
        msg = String()
        msg.data = cube_name
        self.node.carry_pub.publish(msg)
        time.sleep(0.3)
        return True

    def place(self, cube_name: str, x: float = None, y: float = None):
        """停随行(方块传送至区内槽位绝对坐标)→张开→收臂。"""
        log = self.node.get_logger()
        log.info(f'[arm] 释放 {cube_name}')
        msg = String()
        if x is not None and y is not None:
            msg.data = f'drop|{x:.3f}|{y:.3f}'
        else:
            msg.data = 'stop'
        self.node.carry_pub.publish(msg)
        time.sleep(0.3)
        self.move_gripper(GRIP_OPEN)
        self.move_arm(HOME, 1.6)
        return True
