#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CMD_VEL Multiplexer v4 – Strict mode selection.

Subscribes:
  /cmd_vel/manual   (joystick)
  /cmd_vel/web      (web D-pad)
  /cmd_vel/auto     (auto_explorer)
  /web_control_mode (String) – desired mode: WEB, JOY, AUTO
  /auto_enable      (Bool)    – only used in AUTO mode (extra safety)

Publishes:
  /cmd_vel          (Twist)
  /control_mode     (String)  – current active source
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('timeout', 0.15)   # 150 ms
        self.declare_parameter('loop_hz', 50.0)

        self.TIMEOUT = self.get_parameter('timeout').value
        hz = self.get_parameter('loop_hz').value

        # Subscribers
        self.sub_manual = self.create_subscription(Twist, '/cmd_vel/manual', self._cb_manual, 10)
        self.sub_web    = self.create_subscription(Twist, '/cmd_vel/web',    self._cb_web,    10)
        self.sub_auto   = self.create_subscription(Twist, '/cmd_vel/auto',   self._cb_auto,   10)
        self.sub_mode   = self.create_subscription(String, '/web_control_mode', self._cb_mode, 10)
        self.sub_auto_enable = self.create_subscription(Bool, '/auto_enable', self._cb_auto_enable, 10)

        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mode = self.create_publisher(String, '/control_mode', 10)

        # State
        self.cmd_manual = Twist()
        self.cmd_web    = Twist()
        self.cmd_auto   = Twist()
        self.t_manual   = 0.0
        self.t_web      = 0.0
        self.t_auto     = 0.0
        self.auto_enabled = False

        self.desired_mode = 'JOY'   # default
        self.last_published_mode = ''

        self._zero_flush = 0

        self.timer = self.create_timer(1.0/hz, self._loop)
        self.get_logger().info(f'CmdVelMux v4: timeout={self.TIMEOUT}s, mode=JOY')

    def _cb_manual(self, msg): self.cmd_manual = msg; self.t_manual = time.time()
    def _cb_web(self, msg):    self.cmd_web    = msg; self.t_web    = time.time()
    def _cb_auto(self, msg):   self.cmd_auto   = msg; self.t_auto   = time.time()

    def _cb_mode(self, msg: String):
        new_mode = msg.data
        if new_mode in ('WEB','JOY','AUTO'):
            if new_mode != self.desired_mode:
                self.desired_mode = new_mode
                self._zero_flush = 5
                self.get_logger().info(f'Desired mode → {new_mode}')

    def _cb_auto_enable(self, msg: Bool):
        self.auto_enabled = msg.data

    def _loop(self):
        if self._zero_flush > 0:
            self.pub_cmd.publish(Twist())
            self._zero_flush -= 1
            self._publish_mode('STOP')
            return

        now = time.time()
        mode = self.desired_mode

        if mode == 'WEB':
            fresh = (now - self.t_web) < self.TIMEOUT
            if fresh:
                self.pub_cmd.publish(self.cmd_web)
                self._publish_mode('WEB')
            else:
                self.pub_cmd.publish(Twist())
                self._publish_mode('STOP')
        elif mode == 'JOY':
            fresh = (now - self.t_manual) < self.TIMEOUT
            if fresh:
                self.pub_cmd.publish(self.cmd_manual)
                self._publish_mode('MANUAL')
            else:
                self.pub_cmd.publish(Twist())
                self._publish_mode('STOP')
        elif mode == 'AUTO':
            fresh = self.auto_enabled and ((now - self.t_auto) < self.TIMEOUT)
            if fresh:
                self.pub_cmd.publish(self.cmd_auto)
                self._publish_mode('AUTO')
            else:
                self.pub_cmd.publish(Twist())
                self._publish_mode('STOP')

    def _publish_mode(self, mode: str):
        if mode != self.last_published_mode:
            self.last_published_mode = mode
            m = String(); m.data = mode
            self.pub_mode.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
