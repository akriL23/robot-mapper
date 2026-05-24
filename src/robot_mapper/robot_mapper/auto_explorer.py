#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        # ================= ПАРАМЕТРЫ (оптимизированы под малые комнаты) =================
        self.declare_parameter('obstacle_dist', 0.25)
        self.declare_parameter('safe_dist', 0.45)
        self.declare_parameter('max_linear', 0.08)
        self.declare_parameter('max_angular', 0.70)
        self.declare_parameter('lin_accel', 0.12)
        self.declare_parameter('ang_accel', 1.2)
        self.declare_parameter('sector_width_deg', 15.0)
        self.declare_parameter('commit_duration', 1.5)
        self.declare_parameter('hysteresis_time', 0.8)

        self.obstacle_dist = self.get_parameter('obstacle_dist').value
        self.safe_dist = self.get_parameter('safe_dist').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.lin_accel = self.get_parameter('lin_accel').value
        self.ang_accel = self.get_parameter('ang_accel').value
        self.sector_width = self.get_parameter('sector_width_deg').value
        self.commit_duration = self.get_parameter('commit_duration').value
        self.hysteresis_time = self.get_parameter('hysteresis_time').value

        # ================= ПУБЛИКАТОРЫ/ПОДПИСКИ =================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/auto', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)

        # ================= СОСТОЯНИЕ =================
        self.last_scan = None
        self.last_scan_time = 0.0
        self.current_twist = Twist()
        self.target_twist = Twist()
        self.last_cmd_time = 0.0

        # Анти-зацикливание
        self.committed_until = 0.0
        self.committed_angle = 0.0
        self.last_dir_change = 0.0
        self.prev_best_sector = -1
        self.state = 'SEARCH'  # SEARCH, AVOID, COMMIT, WALL_FOLLOW

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('AutoExplorer v3 (Small Room Optimized)')

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = time.time()

    def control_loop(self):
        now = time.time()
        dt = now - self.last_cmd_time if self.last_cmd_time > 0 else 0.1
        self.last_cmd_time = now
        dt = max(0.01, min(dt, 0.2))

        if self.last_scan is None or (now - self.last_scan_time) > 1.0:
            self.target_twist.linear.x = 0.0
            self.target_twist.angular.z = 0.0
            self.publish_smoothed(dt)
            return

        ranges = self.last_scan.ranges
        if len(ranges) < 360:
            return

        n_sectors = int(360 / self.sector_width)
        costs = [0.0] * n_sectors
        for i in range(n_sectors):
            angle_deg = i * self.sector_width - 180.0
            idx = int(angle_deg + 180) % 360
            d = ranges[idx]
            if not math.isfinite(d) or d <= 0.05:
                costs[i] = 1.0
            else:
                costs[i] = max(0.0, 1.0 - (d / self.safe_dist))

        # Поиск лучшего сектора
        best_sector = costs.index(min(costs))
        best_angle = best_sector * self.sector_width - 180.0

        # Анти-дрожание: игнорируем смену курса, если прошло мало времени
        can_change_dir = (now - self.last_dir_change) > self.hysteresis_time
        if not can_change_dir and self.prev_best_sector != -1:
            best_sector = self.prev_best_sector
            best_angle = best_sector * self.sector_width - 180.0

        self.prev_best_sector = best_sector

        # COMMIT: если крутимся долго → жёстко едем в лучшую сторону
        if self.state == 'COMMIT':
            if now < self.committed_until:
                self.target_twist.angular.z = max(-self.max_angular, min(self.max_angular, math.radians(best_angle) * 1.2))
                self.target_twist.linear.x = self.max_linear * 0.7
                self.publish_smoothed(dt)
                return
            else:
                self.state = 'SEARCH'

        # Оценка загромождённости
        front_cost = costs[int(180 / self.sector_width)]
        avg_cost = sum(costs) / len(costs)

        if front_cost > 0.8 and avg_cost > 0.6:
            # Все стороны заняты → режим COMMIT
            self.state = 'COMMIT'
            self.committed_until = now + self.commit_duration
            self.last_dir_change = now
            self.target_twist.angular.z = max(-self.max_angular, min(self.max_angular, math.radians(best_angle)))
            self.target_twist.linear.x = 0.05  # медленный рывок
            self.get_logger().info('Commit mode: breaking spin cycle')
            return

        if front_cost > 0.6:
            # Препятствие близко → плавный поворот
            self.state = 'AVOID'
            self.target_twist.linear.x = 0.0
            self.target_twist.angular.z = math.radians(best_angle) * 1.5
            if abs(self.target_twist.angular.z) > self.max_angular:
                self.target_twist.angular.z = self.max_angular * (1 if self.target_twist.angular.z > 0 else -1)
            self.last_dir_change = now
        else:
            # Чисто → едем вперёд с адаптивной скоростью
            self.state = 'SEARCH'
            lin_factor = max(0.2, 1.0 - front_cost)
            self.target_twist.linear.x = lin_factor * self.max_linear
            self.target_twist.angular.z = max(-self.max_angular, min(self.max_angular, math.radians(best_angle) * 0.8))

        self.publish_smoothed(dt)

    def publish_smoothed(self, dt: float):
        lin_diff = self.target_twist.linear.x - self.current_twist.linear.x
        ang_diff = self.target_twist.angular.z - self.current_twist.angular.z

        max_lin = self.lin_accel * dt
        max_ang = self.ang_accel * dt

        self.current_twist.linear.x += max(-max_lin, min(max_lin, lin_diff))
        self.current_twist.angular.z += max(-max_ang, min(max_ang, ang_diff))

        if abs(self.current_twist.angular.z) < 0.04:
            self.current_twist.angular.z = 0.0
        if abs(self.target_twist.linear.x) < 0.01 and abs(self.target_twist.angular.z) < 0.02:
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0

        self.cmd_pub.publish(self.current_twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received Ctrl+C')
    finally:
        try:
            node.cmd_pub.publish(Twist())
            node.destroy_node()
        except Exception as e:
            node.get_logger().warn(f'Shutdown warning: {e}')
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
