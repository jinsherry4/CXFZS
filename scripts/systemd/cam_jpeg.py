# -*- coding: utf-8 -*-
"""相机 JPEG 压缩转发：/camera/image_raw (rgb8) → /camera/image_raw/compressed
(sensor_msgs/CompressedImage, jpeg)。
coStudio/Foxglove 的 Image 面板对 raw rgb8 渲染存在白屏缺陷，compressed
jpeg 路径渲染可靠。压缩开销 ~15ms/帧（320x240），5fps 下 CPU 可忽略。"""
import io
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CamJpeg(Node):
    def __init__(self):
        super().__init__('cam_jpeg')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(CompressedImage,
                                         '/camera/image_raw/compressed', qos)
        self.sub = self.create_subscription(Image, '/camera/image_raw',
                                            self.on_img, qos)
        self.last = 0.0
        self.get_logger().info('cam_jpeg 就绪: raw rgb8 → jpeg compressed')

    def on_img(self, m: Image):
        now = time.time()
        if now - self.last < 0.2:   # 最多 5fps
            return
        self.last = now
        try:
            from PIL import Image as PILImage
            arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(
                m.height, m.width, 3)
            buf = io.BytesIO()
            PILImage.fromarray(arr).save(buf, format='JPEG', quality=70)
            out = CompressedImage()
            out.header = m.header
            out.format = 'jpeg'
            out.data = buf.getvalue()
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'压缩失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    n = CamJpeg()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
