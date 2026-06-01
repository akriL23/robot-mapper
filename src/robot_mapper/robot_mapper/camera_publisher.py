#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import glob
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # ПАРАМЕТРЫ
        self.WIDTH = 320
        self.HEIGHT = 240
        self.FPS_TARGET = 30
        
        # 1. Находим камеру
        self.device_path = self._find_camera()
        
        if not self.device_path:
            self.get_logger().error('❌ NO CAMERA FOUND!')
            self.cap_failed = True
            self.timer = self.create_timer(5.0, self._recheck)
            return

        self.get_logger().info(f'📸 Camera: {self.device_path}')
        
        # 2. OpenCV — только базовые настройки, без V4L2
        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error('❌ Failed to open camera')
            self.cap_failed = True
            return
        
        # Минимальные настройки
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, self.FPS_TARGET)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'✅ {actual_w:.0f}x{actual_h:.0f} @{actual_fps:.0f}fps')
        
        self.cap_failed = False
        self.frame_count = 0
        self.last_log = time.time()
        
        self.timer = self.create_timer(1.0/self.FPS_TARGET, self.publish_frame)

    def _find_camera(self):
        """Ищет первое доступное видеоустройство"""
        for dev in sorted(glob.glob('/dev/video[0-9]')):
            dev_num = int(dev.replace('/dev/video', ''))
            if dev_num < 10 and os.access(dev, os.R_OK | os.W_OK):
                # Проверяем что это реальная камера
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    cap.release()
                    return dev
        return None

    def _recheck(self):
        """Повторный поиск камеры"""
        self.get_logger().info('🔄 Rechecking...')
        path = self._find_camera()
        if path:
            self.device_path = path
            self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, self.FPS_TARGET)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            if self.cap.isOpened():
                self.get_logger().info('✅ Camera re-connected!')
                self.cap_failed = False
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0/self.FPS_TARGET, self.publish_frame)

    def publish_frame(self):
        if self.cap_failed:
            return
        
        # Очищаем буфер — единственная оптимизация для задержки
        for _ in range(3):
            self.cap.grab()
        
        ret, frame = self.cap.read()
        
        if not ret or frame is None:
            return
        
        # Изменяем размер если нужно
        if frame.shape[1] != self.WIDTH or frame.shape[0] != self.HEIGHT:
            frame = cv2.resize(frame, (self.WIDTH, self.HEIGHT), 
                             interpolation=cv2.INTER_NEAREST)
        
        # Просто конвертируем BGR -> RGB, без фильтров
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.height = rgb.shape[0]
        msg.width = rgb.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        
        self.publisher_.publish(msg)
        
        # Статистика
        self.frame_count += 1
        now = time.time()
        if now - self.last_log >= 5.0:
            fps = self.frame_count / (now - self.last_log)
            data_size = len(msg.data) / 1024
            self.get_logger().info(f'📊 {fps:.0f}fps | {data_size:.0f}KB/frame')
            self.frame_count = 0
            self.last_log = now

    def destroy_node(self):
        self.get_logger().info('🛑 Shutting down...')
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
