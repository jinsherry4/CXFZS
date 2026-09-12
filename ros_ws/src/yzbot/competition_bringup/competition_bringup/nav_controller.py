# -*- coding: utf-8 -*-
"""Nav2 导航封装：发送 /navigate_to_pose 目标，阻塞等待结果。"""
import math
import time


class NavController:
    def __init__(self, node):
        self.node = node
        self._ac = None
        self._ac_through = None
        self._handle = None
        self._pending_fut = None

    def cancel_active(self):
        """取消在执行的导航目标（弹飞恢复等场景：恢复期间必须停掉 DWB，
        否则控制器继续下发速度，传送回家后机器人立刻再次被驱动/弹飞）。"""
        h = self._handle
        self._handle = None
        if h is not None:
            try:
                h.cancel_goal_async()
            except Exception:
                pass

    def wait_servers(self, timeout_sec=180.0):
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        if self._ac is None:
            self._ac = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        end = time.time() + timeout_sec
        while time.time() < end:
            if self._ac.wait_for_server(timeout_sec=1.0):
                return True
        return False

    def _wait_future(self, future, timeout):
        import threading
        done = threading.Event()

        def _cb(_):
            done.set()

        future.add_done_callback(_cb)
        return done.wait(timeout)

    def _cleanup_orphan(self):
        """上一次 send 的 goal 请求若被服务器迟到受理，取消该孤儿目标：
        否则它会占住 bt_navigator，后续所有目标都被拒绝（未受理 3 连）。"""
        fut = self._pending_fut
        self._pending_fut = None
        if fut is not None and fut.done():
            try:
                h = fut.result()
                if h.accepted:
                    self.node.get_logger().warn('清理迟到受理的孤儿导航目标')
                    h.cancel_goal_async()
            except Exception:
                pass

    def goto(self, x, y, yaw_deg=0.0, frame='map', timeout_sec=50.0):
        """导航到 (x, y, yaw°)，返回是否成功。"""
        from nav2_msgs.action import NavigateToPose
        self._cleanup_orphan()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        yaw = math.radians(float(yaw_deg))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Nav2 action server 未上线')
            return False
        # 上一目标尚在 abort 收尾时，send_goal 会被瞬时拒绝/丢单——在
        # mission_node 的重试预算外先自行快速重发，避免白白烧掉一次尝试。
        handle = None
        for attempt in range(3):
            fut = self._ac.send_goal_async(goal)
            if self._wait_future(fut, 30.0):
                h = fut.result()
                if h.accepted:
                    handle = h
                    break
            else:
                # 受理响应超时：future 可能已被服务器迟到受理，留待清理
                self._pending_fut = fut
            time.sleep(2.0)
        if handle is None:
            self.node.get_logger().warn('Nav2 目标连续 3 次未受理')
            return False
        self._handle = handle
        res_fut = handle.get_result_async()
        ok = self._wait_future(res_fut, timeout_sec)
        self._handle = None
        if not ok:
            self.node.get_logger().warn('Nav2 导航超时，取消目标')
            handle.cancel_goal_async()
            return False
        status = res_fut.result().status
        return status == 4  # SUCCEEDED

    def goto_through(self, poses, timeout_sec=240.0):
        """poses: [{'x','y','yaw'}, ...] 中间点穿行不停靠，末位为终点。

        r14 复盘：漏斗绕行点逐个 NavigateToPose 停靠（减速-对航向-再加速）
        把绕行腿均速拖到 0.28m/s（长腿 0.5m/s）。NavigateThroughPoses 一次
        穿行全部中间点；失败/超时由调用方退回逐点级联。"""
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateThroughPoses
        from geometry_msgs.msg import PoseStamped
        if self._ac_through is None:
            self._ac_through = ActionClient(
                self.node, NavigateThroughPoses, '/navigate_through_poses')
        if not self._ac_through.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().warn('NavigateThroughPoses 服务未上线')
            return False
        goal = NavigateThroughPoses.Goal()
        for p in poses:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.node.get_clock().now().to_msg()
            ps.pose.position.x = float(p['x'])
            ps.pose.position.y = float(p['y'])
            yaw = math.radians(float(p.get('yaw', 0.0)))
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.append(ps)
        fut = self._ac_through.send_goal_async(goal)
        if not self._wait_future(fut, 30.0):
            self.node.get_logger().warn('穿行目标受理超时')
            return False
        h = fut.result()
        if not h.accepted:
            return False
        self._handle = h
        res_fut = h.get_result_async()
        ok = self._wait_future(res_fut, timeout_sec)
        self._handle = None
        if not ok:
            self.node.get_logger().warn('穿行导航超时，取消目标')
            h.cancel_goal_async()
            return False
        return res_fut.result().status == 4  # SUCCEEDED
