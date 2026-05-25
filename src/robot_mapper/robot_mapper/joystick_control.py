#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JoystickControl v2
Fixes vs v1:
  - Dead-zone filtering: стик в покое не публикует нули в /cmd_vel/manual
    (это было главной причиной блокировки авто-режима)
  - Keep-alive: пока стик активен, публикуем на 20 Гц даже между /joy пакетами
    (устраняет рывки при Bluetooth latency)
  - Кнопка enable: L1/LB (button[4]) должна быть зажата для движения (dead-man switch)
    Параметр require_enable=False по умолчанию — не ломает старое поведение
  - Параметры читаются динамически через ros2 param set без перезапуска
  - Публикует /joy_active (Bool) для UI индикации
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('linear_scale',  0.5)
        self.declare_parameter('angular_scale', 1.5)
        self.declare_parameter('deadzone',      0.10)   # ось < deadzone считается нулём
        self.declare_parameter('require_enable', False)  # True = нужно держать L1
        self.declare_parameter('enable_button',  4)      # индекс кнопки dead-man switch
        self.declare_parameter('axis_linear',    1)      # индекс оси линейной скорости
        self.declare_parameter('axis_angular',   0)      # индекс оси угловой скорости
        self.declare_parameter('keepalive_hz',  20.0)   # частота keep-alive публикации

        self._update_params()

        # ── Topics ───────────────────────────────────────────────────────────
        self.joy_sub  = self.create_subscription(Joy,  '/joy', self._joy_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel/manual', 10)
        self.act_pub  = self.create_publisher(Bool,  '/joy_active',     10)

        # ── State ────────────────────────────────────────────────────────────
        self._last_twist   = Twist()
        self._joy_moving   = False   # True = стик вне дедзоны
        self._last_joy_t   = 0.0
        self._joy_timeout  = 0.5    # секунд без /joy → считаем неактивным

        # Keep-alive таймер: пока стик активен, шлём команду на keepalive_hz
        period = 1.0 / self.get_parameter('keepalive_hz').value
        self._timer = self.create_timer(period, self._keepalive_cb)

        self.get_logger().info(
            f'JoystickControl v2: linear_scale={self.linear_scale} '
            f'angular_scale={self.angular_scale} deadzone={self.deadzone}'
        )

    def _update_params(self):
        self.linear_scale  = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone      = self.get_parameter('deadzone').value
        self.require_enable= self.get_parameter('require_enable').value
        self.enable_button = int(self.get_parameter('enable_button').value)
        self.axis_linear   = int(self.get_parameter('axis_linear').value)
        self.axis_angular  = int(self.get_parameter('axis_angular').value)

    def _apply_deadzone(self, v: float) -> float:
        if abs(v) < self.deadzone:
            return 0.0
        # rescale: [deadzone, 1] → [0, 1]
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _joy_cb(self, msg: Joy):
        self._update_params()  # подхватываем ros2 param set без перезапуска
        self._last_joy_t = time.time()

        # Dead-man switch
        if self.require_enable:
            enabled = (len(msg.buttons) > self.enable_button
                       and msg.buttons[self.enable_button])
            if not enabled:
                self._stop_motion()
                return

        # Безопасное чтение осей
        raw_lin = msg.axes[self.axis_linear]  if len(msg.axes) > self.axis_linear  else 0.0
        raw_ang = msg.axes[self.axis_angular] if len(msg.axes) > self.axis_angular else 0.0

        lin = self._apply_deadzone(-raw_lin) * self.linear_scale
        ang = self._apply_deadzone(-raw_ang) * self.angular_scale

        moving = abs(lin) > 0.001 or abs(ang) > 0.001
        self._joy_moving = moving

        if moving:
            self._last_twist.linear.x  = lin
            self._last_twist.angular.z = ang
        else:
            # Стик в покое — НЕ публикуем нули в cmd_vel/manual,
            # чтобы не блокировать авто-режим в мультиплексоре.
            # Просто замолкаем → мультиплексор таймаутит manual и переключается на авто.
            self._last_twist = Twist()
            self._joy_moving = False

        # Публикуем статус активности для UI
        b = Bool(); b.data = moving
        self.act_pub.publish(b)

    def _keepalive_cb(self):
        """Публикует последнюю команду пока стик активен."""
        now = time.time()

        # Джойстик пропал (Bluetooth обрыв) → экстренный стоп
        if self._last_joy_t > 0 and (now - self._last_joy_t) > self._joy_timeout:
            if self._joy_moving:
                self.get_logger().warn('Joy timeout — emergency stop')
                self._stop_motion()
            return

        if self._joy_moving:
            self.cmd_pub.publish(self._last_twist)

    def _stop_motion(self):
        self._joy_moving  = False
        self._last_twist  = Twist()
        # Публикуем один явный ноль чтобы мультиплексор сбросил таймер
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
        node.cmd_pub.publish(Twist())  # финальный стоп
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
