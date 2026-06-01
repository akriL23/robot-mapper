#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
JoystickControl v2 – unchanged except keepalive_hz default 50 Hz for minimal delay.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.5)
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('require_enable', False)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('keepalive_hz', 50.0)   # было 20, теперь 50 Гц (20 мс)

        self._update_params()
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/manual', 10)
        self.act_pub = self.create_publisher(Bool, '/joy_active', 10)

        self._last_twist = Twist()
        self._joy_moving = False
        self._last_joy_t = 0.0
        self._joy_timeout = 0.5

        period = 1.0 / self.get_parameter('keepalive_hz').value
        self._timer = self.create_timer(period, self._keepalive_cb)

    def _update_params(self):
        self.linear_scale   = self.get_parameter('linear_scale').value
        self.angular_scale  = self.get_parameter('angular_scale').value
        self.deadzone       = self.get_parameter('deadzone').value
        self.require_enable = self.get_parameter('require_enable').value
        self.enable_button  = int(self.get_parameter('enable_button').value)
        self.axis_linear    = int(self.get_parameter('axis_linear').value)
        self.axis_angular   = int(self.get_parameter('axis_angular').value)

    def _apply_deadzone(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _joy_cb(self, msg: Joy):
        self._update_params()
        self._last_joy_t = time.time()
        if self.require_enable:
            if not (len(msg.buttons) > self.enable_button and msg.buttons[self.enable_button]):
                self._stop_motion()
                return
        raw_lin = msg.axes[self.axis_linear]  if len(msg.axes) > self.axis_linear  else 0.0
        raw_ang = msg.axes[self.axis_angular] if len(msg.axes) > self.axis_angular else 0.0
        lin = self._apply_deadzone(-raw_lin) * self.linear_scale
        ang = self._apply_deadzone(-raw_ang) * self.angular_scale
        moving = abs(lin) > 0.001 or abs(ang) > 0.001
        self._joy_moving = moving
        if moving:
            self._last_twist.linear.x = lin
            self._last_twist.angular.z = ang
        else:
            self._last_twist = Twist()
            self._joy_moving = False
        b = Bool(); b.data = moving
        self.act_pub.publish(b)

    def _keepalive_cb(self):
        now = time.time()
        if self._last_joy_t > 0 and (now - self._last_joy_t) > self._joy_timeout:
            if self._joy_moving:
                self.get_logger().warn('Joy timeout – emergency stop')
                self._stop_motion()
            return
        if self._joy_moving:
            self.cmd_pub.publish(self._last_twist)

    def _stop_motion(self):
        self._joy_moving = False
        self._last_twist = Twist()
        self.cmd_pub.publish(Twist())
        b = Bool(); b.data = False
        self.act_pub.publish(b)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
