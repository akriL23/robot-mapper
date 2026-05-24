#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        # Пины [TRIG, ECHO] для 4 датчиков (без конфликтов с моторами)
        self.sensors = {
            'front_left':  [4, 7],
            'front_right': [16, 26],
            'back_left':   [27, 14],
            'back_right':  [15, 10]
        }
        
        # 🔄 ИСПРАВЛЕНО: us_pubs вместо publishers (избегаем конфликта имён)
        self.us_pubs = {}
        for name in self.sensors:
            self.us_pubs[name] = self.create_publisher(
                Range, f'/ultrasonic/{name}', 10)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for trig, echo in self.sensors.values():
            GPIO.setup(trig, GPIO.OUT)
            GPIO.setup(echo, GPIO.IN)
            GPIO.output(trig, GPIO.LOW)
        
        self.timer = self.create_timer(0.1, self.measure_all)
        self.get_logger().info('Ultrasonic node started (4 sensors)')

    def _measure(self, trig, echo):
        GPIO.output(trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, GPIO.LOW)
        
        # Ждём начала эха
        t0 = time.time()
        timeout = t0 + 0.04
        while GPIO.input(echo) == 0:
            if time.time() > timeout:
                return -1.0
            t0 = time.time()
        
        # Ждём конца эха
        start = time.time()
        timeout = start + 0.04
        while GPIO.input(echo) == 1:
            if time.time() > timeout:
                break
        
        duration = time.time() - start
        # Конвертируем в метры: (время * 343 м/с) / 2
        distance = (duration * 343.0) / 2.0
        return round(distance, 3) if distance < 4.0 else 4.0

    def measure_all(self):
        for name, (trig, echo) in self.sensors.items():
            dist = self._measure(trig, echo)
            if dist < 0:
                continue
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'ultrasonic_{name}'
            msg.radiation_type = Range.ULTRASOUND
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = dist
            # 🔄 ИСПРАВЛЕНО: us_pubs вместо publishers
            self.us_pubs[name].publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
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
