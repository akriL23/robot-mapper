#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import glob

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # 1. Находим камеру автоматически (игнорируем видео10-31, это кодеки Pi)
        self.device_path = self._find_camera()
        
        if not self.device_path:
            self.get_logger().error('❌ NO CAMERA FOUND! (Tried /dev/video[0-9])')
            self.cap_failed = True
            self.timer = self.create_timer(5.0, self._recheck) # Пробовать каждые 5 сек
            return

        self.get_logger().info(f'📸 Camera found at: {self.device_path}')
        
        # 2. Инициализация OpenCV
        self.cap = cv2.VideoCapture(self.device_path)
        
        # Настройки для Logitech C920 (MJPG поток самый надежный)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        if not self.cap.isOpened():
            self.get_logger().error(' Failed to open video capture via OpenCV')
            self.cap_failed = True
        else:
            self.get_logger().info('✅ Camera ready (15 FPS)')
            self.cap_failed = False
            self.timer = self.create_timer(0.066, self.publish_frame) # ~15 FPS

    def _find_camera(self):
        """Ищет первое устройство /dev/video[0-9], доступное для чтения/записи"""
        devices = sorted(glob.glob('/dev/video[0-9]'))
        for dev in devices:
            # Проверяем права доступа
            if os.access(dev, os.R_OK | os.W_OK):
                return dev
        return None

    def _recheck(self):
        """Повторная попытка найти камеру (если подключили позже)"""
        self.get_logger().info(' Rechecking for camera...')
        path = self._find_camera()
        if path:
            self.device_path = path
            self.cap = cv2.VideoCapture(path)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if self.cap.isOpened():
                self.get_logger().info('✅ Camera re-connected!')
                self.cap_failed = False
                self.timer = self.create_timer(0.066, self.publish_frame)

    def publish_frame(self):
        if self.cap_failed: return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # Конвертация BGR (OpenCV) -> RGB (ROS Image)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        
        self.publisher_.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
