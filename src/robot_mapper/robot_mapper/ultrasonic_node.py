#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.sensors = {
            'front_left':  [23, 24], 'front_right': [17, 27],
            'back_left':   [22, 25], 'back_right':  [5, 6]
        }
        self.publishers = {name: self.create_publisher(Range, f'/ultrasonic/{name}', 10) for name in self.sensors}
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for trig, echo in self.sensors.values():
            GPIO.setup(trig, GPIO.OUT)
            GPIO.setup(echo, GPIO.IN)
            GPIO.output(trig, False)
        self.timer = self.create_timer(0.1, self.measure_all)

    def _measure(self, trig, echo):
        GPIO.output(trig, True); time.sleep(0.00001); GPIO.output(trig, False)
        t0 = time.time()
        while GPIO.input(echo) == 0 and time.time() - t0 < 0.1: pass
        start = time.time()
        while GPIO.input(echo) == 1 and time.time() - t0 < 0.1: pass
        dur = time.time() - start
        return min(round(dur * 17150 / 100.0, 2), 4.0)

    def measure_all(self):
        for name, (trig, echo) in self.sensors.items():
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'ultrasonic_{name}'
            msg.radiation_type = Range.ULTRASOUND
            msg.min_range, msg.max_range = 0.02, 4.0
            msg.range = self._measure(trig, echo)
            self.publishers[name].publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
