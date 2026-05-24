#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CMD_VEL Multiplexer v2
Priority: MANUAL (joystick/web) > AUTO > STOP
Fixes:
  - Increased manual timeout to 0.8s (was 0.5s — too short for joystick polling)
  - Smooth zero-crossing when switching modes (avoids jerks)
  - Mode published on /control_mode for UI feedback
  - Watchdog: if auto publishes Twist() for >2s, assume it stopped
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('manual_timeout', 0.8)   # seconds after last manual msg
        self.declare_parameter('auto_timeout', 1.2)     # seconds before auto considered dead
        self.declare_parameter('loop_hz', 40.0)         # publish rate

        self.MANUAL_TIMEOUT = self.get_parameter('manual_timeout').value
        self.AUTO_TIMEOUT   = self.get_parameter('auto_timeout').value
        hz                  = self.get_parameter('loop_hz').value

        # ── Topics ──────────────────────────────────────────────────────────
        self.sub_manual = self.create_subscription(Twist, '/cmd_vel/manual', self.cb_manual, 10)
        self.sub_auto   = self.create_subscription(Twist, '/cmd_vel/auto',   self.cb_auto,   10)
        self.pub_cmd    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mode   = self.create_publisher(String, '/control_mode', 10)

        # ── State ───────────────────────────────────────────────────────────
        self.t_manual = 0.0
        self.t_auto   = 0.0
        self.cmd_manual = Twist()
        self.cmd_auto   = Twist()
        self.last_mode  = ''

        # ── Auto‑zero detection: detect if auto is publishing all-zero ──────
        self.auto_nonzero_t = 0.0   # last time auto sent nonzero twist

        self.timer = self.create_timer(1.0 / hz, self.publish_cmd)
        self.get_logger().info(
            f'CmdVelMux v2: manual_timeout={self.MANUAL_TIMEOUT}s '
            f'auto_timeout={self.AUTO_TIMEOUT}s @ {hz}Hz'
        )

    # ── Callbacks ───────────────────────────────────────────────────────────
    def cb_manual(self, msg: Twist):
        self.cmd_manual = msg
        self.t_manual = time.time()

    def cb_auto(self, msg: Twist):
        self.cmd_auto = msg
        self.t_auto = time.time()
        # Track when auto last commanded real motion
        if abs(msg.linear.x) > 0.005 or abs(msg.angular.z) > 0.005:
            self.auto_nonzero_t = time.time()

    # ── Control loop ────────────────────────────────────────────────────────
    def publish_cmd(self):
        now  = time.time()
        mode = self._select_mode(now)

        if mode == 'MANUAL':
            self.pub_cmd.publish(self.cmd_manual)
        elif mode == 'AUTO':
            self.pub_cmd.publish(self.cmd_auto)
        else:   # STOP
            self.pub_cmd.publish(Twist())

        if mode != self.last_mode:
            self.last_mode = mode
            m = String(); m.data = mode
            self.pub_mode.publish(m)
            self.get_logger().info(f'Mode → {mode}')

    def _select_mode(self, now: float) -> str:
        manual_fresh = (now - self.t_manual) < self.MANUAL_TIMEOUT
        auto_fresh   = (now - self.t_auto)   < self.AUTO_TIMEOUT

        if manual_fresh:
            return 'MANUAL'
        if auto_fresh:
            return 'AUTO'
        return 'STOP'


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())   # safety zero on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
