# -*- coding: utf-8 -*-
"""出题器桥接节点：运行 TMSCQtest 出题程序，捕获题目文本发布到 /mission/question。

用法：
    ros2 run competition_bringup question_bridge
    ros2 run competition_bringup question_bridge --ros-args -p binary_path:=/home/ros/TMSCQtest_x86_x64.bin

参数：
    binary_path  出题程序路径（默认 ~/TMSCQtest_x86_x64.bin）
    cwd          出题程序工作目录（默认其所在目录）
"""
import os
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QuestionBridge(Node):

    def __init__(self):
        super().__init__('question_bridge')
        self.declare_parameter('binary_path', os.path.expanduser('~/TMSCQtest_x86_x64.bin'))
        self.pub = self.create_publisher(String, '/mission/question', 10)
        binary = self.get_parameter('binary_path').value
        if not os.path.isfile(binary):
            self.get_logger().error(f'出题程序不存在: {binary}')
            raise SystemExit(1)
        self.get_logger().info(f'运行出题程序: {binary}')
        try:
            proc = subprocess.run([binary], capture_output=True, timeout=30,
                                  cwd=os.path.dirname(binary) or None)
        except subprocess.TimeoutExpired:
            self.get_logger().error('出题程序超时（30s）')
            raise SystemExit(2)
        text = self._decode(proc.stdout) or self._decode(proc.stderr)
        if not text:
            self.get_logger().error('出题程序无输出')
            raise SystemExit(3)
        question = self._extract(text)
        self.get_logger().info(f'题目原文:\n{question}')
        msg = String()
        msg.data = question
        # 新节点与订阅者的 DDS 匹配在 UDP-only 传输下需要一点时间，
        # 单发即退会让消息在匹配完成前丢失——重复发布确保送达
        import time
        for _ in range(10):
            self.pub.publish(msg)
            time.sleep(0.3)
        self.get_logger().info('已发布到 /mission/question')

    @staticmethod
    def _decode(raw: bytes) -> str:
        for enc in ('utf-8', 'gbk'):
            try:
                return raw.decode(enc)
            except UnicodeDecodeError:
                continue
        return raw.decode('utf-8', errors='replace')

    @staticmethod
    def _extract(text: str) -> str:
        lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
        return '\n'.join(lines)


def main(args=None):
    rclpy.init(args=args)
    try:
        QuestionBridge()
    except SystemExit as e:
        rclpy.shutdown()
        raise e
    rclpy.shutdown()


if __name__ == '__main__':
    main()
