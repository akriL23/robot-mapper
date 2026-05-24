#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
import numpy as np
import subprocess
import tempfile
import os

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.2, self.publish_frame)  # 5 FPS
        
        # Создаём временный файл для кадров
        self.tmp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
        self.tmp_path = self.tmp_file.name
        self.tmp_file.close()
        
        # Тестовый захват через v4l2-ctl
        self.get_logger().info('🔍 Testing camera via v4l2-ctl...')
        if not self._capture_frame():
            self.get_logger().error('❌ Camera test failed! Check: 1) ls /dev/video0, 2) sudo modprobe bcm2835-v4l2')
            self.cap_failed = True
        else:
            self.get_logger().info('✅ Camera ready via v4l2-ctl')
            self.cap_failed = False

    def _capture_frame(self):
        """Захват одного кадра через v4l2-ctl + OpenCV"""
        try:
            # v4l2-ctl захватывает 1 кадр в формате MJPG
            cmd = [
                'v4l2-ctl',
                '--device', '/dev/video0',
                '--set-fmt-video=width=640,height=480,pixelformat=MJPG',
                '--stream-mmap=3',
                '--stream-count=1',
                '--stream-to', self.tmp_path
            ]
            result = subprocess.run(cmd, capture_output=True, timeout=3)
            if result.returncode != 0 or not os.path.exists(self.tmp_path):
                return False
            
            # Читаем JPEG через OpenCV
            frame = cv2.imread(self.tmp_path)
            if frame is None:
                return False
            
            # Конвертируем BGR -> RGB
            self.last_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return True
        except Exception:
            return False
        finally:
            if os.path.exists(self.tmp_path):
                try: os.remove(self.tmp_path)
                except: pass

    def publish_frame(self):
        if self.cap_failed:
            return
        if self._capture_frame() and hasattr(self, 'last_frame'):
            try:
                rgb = self.last_frame
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                msg.height, msg.width = rgb.shape[:2]
                msg.encoding = "rgb8"
                msg.is_bigendian = 0
                msg.step = msg.width * 3
                msg.data = rgb.tobytes()
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'Publish error: {e}')

    def destroy_node(self):
        if os.path.exists(self.tmp_path):
            try: os.remove(self.tmp_path)
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
