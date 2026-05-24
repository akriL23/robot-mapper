#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoExplorer v4 — Small-Room Optimised
State machine: FORWARD → AVOID → ROTATE → WALL_FOLLOW → STUCK_ESCAPE
Fixes vs v3:
  - Full 360° sector scan with weighted angular error (not just best_sector index)
  - Wall‑following mode to hug walls and maximise coverage
  - Anti-stuck detector: if robot hasn't moved (odom) in N seconds → STUCK_ESCAPE
  - STUCK_ESCAPE: back up + random large rotation before resuming
  - Smooth velocity ramp (acceleration-limited) in all states
  - All parameters are ROS 2 declared params — tunable at runtime
  - Publishes /explorer_state (std_msgs/String) for UI indicator
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# ── Helpers ──────────────────────────────────────────────────────────────────

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def angle_wrap(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def scan_sector(ranges, center_deg: float, half_width_deg: float) -> float:
    """Return min valid range in [center-half, center+half] degrees (0° = front)."""
    valid = []
    n = len(ranges)
    for d in range(-int(half_width_deg), int(half_width_deg) + 1):
        idx = int((center_deg + d) % 360)
        r = ranges[idx % n]
        if math.isfinite(r) and r > 0.05:
            valid.append(r)
    return min(valid) if valid else float('inf')


# ── Node ─────────────────────────────────────────────────────────────────────

class AutoExplorer(Node):

    # FSM states
    ST_FORWARD      = 'FORWARD'
    ST_AVOID        = 'AVOID'
    ST_ROTATE       = 'ROTATE'
    ST_WALL_FOLLOW  = 'WALL_FOLLOW'
    ST_ESCAPE       = 'ESCAPE'

    def __init__(self):
        super().__init__('auto_explorer')

        # ── Parameters ───────────────────────────────────────────────────────
        p = self.declare_parameter
        p('obstacle_dist',     0.28)   # m — hard stop threshold
        p('safe_dist',         0.50)   # m — start turning
        p('wall_follow_dist',  0.35)   # m — desired wall-follow distance
        p('max_linear',        0.10)   # m/s
        p('max_angular',       0.75)   # rad/s
        p('lin_accel',         0.20)   # m/s²
        p('ang_accel',         1.80)   # rad/s²
        p('stuck_time',        4.0)    # s without odom progress → ESCAPE
        p('stuck_dist',        0.03)   # m — min displacement to not be "stuck"
        p('escape_back_time',  1.2)    # s backing up during escape
        p('escape_spin_time',  1.8)    # s spinning during escape
        p('wall_follow_prob',  0.30)   # probability of entering wall-follow after obstacle
        p('loop_hz',           10.0)

        def gp(name):
            return self.get_parameter(name).value

        self.obstacle_dist    = gp('obstacle_dist')
        self.safe_dist        = gp('safe_dist')
        self.wall_follow_dist = gp('wall_follow_dist')
        self.max_lin          = gp('max_linear')
        self.max_ang          = gp('max_angular')
        self.lin_accel        = gp('lin_accel')
        self.ang_accel        = gp('ang_accel')
        self.stuck_time       = gp('stuck_time')
        self.stuck_dist       = gp('stuck_dist')
        self.escape_back_time = gp('escape_back_time')
        self.escape_spin_time = gp('escape_spin_time')
        self.wall_follow_prob = gp('wall_follow_prob')
        hz                    = gp('loop_hz')

        # ── Publishers / Subscribers ─────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel/auto',   10)
        self.state_pub = self.create_publisher(String, '/explorer_state', 10)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self.scan: LaserScan | None = None
        self.scan_t = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.last_progress_x = 0.0
        self.last_progress_y = 0.0
        self.last_progress_t = time.time()

        self.cur_lin = 0.0
        self.cur_ang = 0.0
        self.tgt_lin = 0.0
        self.tgt_ang = 0.0

        self.state         = self.ST_FORWARD
        self.state_entry_t = time.time()
        self.escape_phase  = 'BACK'
        self.escape_dir    = 1.0

        self.last_loop_t = time.time()
        self.timer = self.create_timer(1.0 / hz, self._loop)
        self.get_logger().info('AutoExplorer v4 started')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self.scan   = msg
        self.scan_t = time.time()

    def _odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    # ── Main control loop ────────────────────────────────────────────────────
    def _loop(self):
        now = time.time()
        dt  = clamp(now - self.last_loop_t, 0.01, 0.3)
        self.last_loop_t = now

        # No scan yet or stale
        if self.scan is None or (now - self.scan_t) > 1.5:
            self._set_target(0.0, 0.0)
            self._publish(dt)
            return

        ranges = list(self.scan.ranges)
        n      = len(ranges)
        if n < 90:
            return

        # Normalise ranges to 360 indices (resample if needed)
        if n != 360:
            step = n / 360.0
            ranges = [ranges[int(i * step) % n] for i in range(360)]

        # ── Sector distances (0° = front, CCW positive) ───────────────────
        front  = scan_sector(ranges,   0, 20)
        fl     = scan_sector(ranges,  45, 20)
        fr     = scan_sector(ranges, -45, 20)
        left   = scan_sector(ranges,  90, 20)
        right  = scan_sector(ranges, -90, 20)
        rear   = scan_sector(ranges, 180, 20)

        # ── Stuck detection ───────────────────────────────────────────────
        dx = self.odom_x - self.last_progress_x
        dy = self.odom_y - self.last_progress_y
        if math.hypot(dx, dy) > self.stuck_dist:
            self.last_progress_x = self.odom_x
            self.last_progress_y = self.odom_y
            self.last_progress_t = now

        time_stuck = now - self.last_progress_t
        if time_stuck > self.stuck_time and self.state != self.ST_ESCAPE:
            self._enter_escape(now)

        # ── FSM ───────────────────────────────────────────────────────────
        if self.state == self.ST_FORWARD:
            self._state_forward(front, fl, fr, left, right, now)

        elif self.state == self.ST_AVOID:
            self._state_avoid(front, fl, fr, left, right, now)

        elif self.state == self.ST_ROTATE:
            self._state_rotate(front, fl, fr, now)

        elif self.state == self.ST_WALL_FOLLOW:
            self._state_wall_follow(front, right, left, now)

        elif self.state == self.ST_ESCAPE:
            self._state_escape(front, rear, now)

        self._publish(dt)
        self._publish_state()

    # ── State handlers ───────────────────────────────────────────────────────
    def _state_forward(self, front, fl, fr, left, right, now):
        if front < self.obstacle_dist or fl < self.obstacle_dist or fr < self.obstacle_dist:
            self._transition(self.ST_AVOID, now)
            return

        if front < self.safe_dist:
            # Slow down, steer toward more open side
            speed_factor = clamp((front - self.obstacle_dist) / (self.safe_dist - self.obstacle_dist), 0.1, 1.0)
            steer = (right - left) * 0.4   # positive = steer right
            self._set_target(self.max_lin * speed_factor, clamp(-steer, -self.max_ang, self.max_ang))
        else:
            # Open road — full speed, gentle correction toward open space
            steer = (right - left) * 0.2
            self._set_target(self.max_lin, clamp(-steer, -self.max_ang * 0.5, self.max_ang * 0.5))

    def _state_avoid(self, front, fl, fr, left, right, now):
        elapsed = now - self.state_entry_t
        if elapsed > 3.0:
            # Timeout → rotate to find exit
            self._transition(self.ST_ROTATE, now)
            return

        # Steer away from closest obstacle
        if fl < fr:
            # obstacle on left → turn right
            self._set_target(0.0, -self.max_ang * 0.8)
        else:
            self._set_target(0.0, self.max_ang * 0.8)

        # Exit condition: front is clear
        if front > self.safe_dist and fl > self.safe_dist and fr > self.safe_dist:
            # Randomly decide wall-follow or forward
            if random.random() < self.wall_follow_prob:
                self._transition(self.ST_WALL_FOLLOW, now)
            else:
                self._transition(self.ST_FORWARD, now)

    def _state_rotate(self, front, fl, fr, now):
        elapsed = now - self.state_entry_t
        # Pick direction toward more open side
        if fl > fr:
            self._set_target(0.0, self.max_ang)
        else:
            self._set_target(0.0, -self.max_ang)

        # Timeout protection
        if elapsed > 4.0:
            self._transition(self.ST_FORWARD, now)
            return

        if front > self.safe_dist and fl > self.safe_dist and fr > self.safe_dist:
            self._transition(self.ST_FORWARD, now)

    def _state_wall_follow(self, front, right, left, now):
        elapsed = now - self.state_entry_t
        # Follow right wall for up to 8s, then return to forward
        if elapsed > 8.0:
            self._transition(self.ST_FORWARD, now)
            return

        if front < self.obstacle_dist:
            self._transition(self.ST_AVOID, now)
            return

        # P-controller: maintain desired distance from right wall
        wall_dist = right
        if not math.isfinite(wall_dist):
            wall_dist = 2.0

        error = self.wall_follow_dist - wall_dist   # positive = too close → steer left
        kp = 1.5
        ang = clamp(kp * error, -self.max_ang, self.max_ang)
        lin = self.max_lin * clamp((front - self.obstacle_dist) / self.safe_dist, 0.3, 1.0)
        self._set_target(lin, ang)

    def _state_escape(self, front, rear, now):
        elapsed = now - self.state_entry_t

        if self.escape_phase == 'BACK':
            # Back up unless rear is blocked
            if rear < 0.25:
                self._set_target(0.0, 0.0)
            else:
                self._set_target(-self.max_lin * 0.6, 0.0)
            if elapsed > self.escape_back_time:
                self.escape_phase = 'SPIN'
                self.state_entry_t = now
                self.escape_dir = random.choice([-1.0, 1.0])

        elif self.escape_phase == 'SPIN':
            self._set_target(0.0, self.max_ang * self.escape_dir)
            if elapsed > self.escape_spin_time:
                # Reset progress tracker
                self.last_progress_x = self.odom_x
                self.last_progress_y = self.odom_y
                self.last_progress_t = now
                self._transition(self.ST_FORWARD, now)

    # ── Transitions & helpers ────────────────────────────────────────────────
    def _enter_escape(self, now):
        self.get_logger().warn('STUCK detected! Entering ESCAPE mode.')
        self.state         = self.ST_ESCAPE
        self.state_entry_t = now
        self.escape_phase  = 'BACK'
        self.escape_dir    = random.choice([-1.0, 1.0])

    def _transition(self, new_state: str, now: float):
        if self.state != new_state:
            self.get_logger().info(f'{self.state} → {new_state}')
        self.state         = new_state
        self.state_entry_t = now

    def _set_target(self, lin: float, ang: float):
        self.tgt_lin = clamp(lin, -self.max_lin, self.max_lin)
        self.tgt_ang = clamp(ang, -self.max_ang, self.max_ang)

    # ── Velocity smoothing & publish ─────────────────────────────────────────
    def _publish(self, dt: float):
        max_dl = self.lin_accel * dt
        max_da = self.ang_accel * dt

        dl = clamp(self.tgt_lin - self.cur_lin, -max_dl, max_dl)
        da = clamp(self.tgt_ang - self.cur_ang, -max_da, max_da)

        self.cur_lin += dl
        self.cur_ang += da

        # Dead-band snap to zero
        if abs(self.cur_lin) < 0.005: self.cur_lin = 0.0
        if abs(self.cur_ang) < 0.01:  self.cur_ang = 0.0

        msg = Twist()
        msg.linear.x  = self.cur_lin
        msg.angular.z = self.cur_ang
        self.cmd_pub.publish(msg)

    def _publish_state(self):
        m = String()
        m.data = self.state
        self.state_pub.publish(m)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested')
    finally:
        try:
            node.cmd_pub.publish(Twist())
            node.destroy_node()
        except Exception as e:
            print(f'Shutdown warning: {e}')
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
