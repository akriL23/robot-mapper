#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CMD_VEL Multiplexer v3
Priority: MANUAL > AUTO > STOP

Главные исправления vs v2:
  - Топик /auto_enable (Bool) — явный флаг блокировки авто.
    web_control публикует False при нажатии «Стоп авто» → мультиплексор
    НЕМЕДЛЕННО перестаёт пропускать /cmd_vel/auto, не дожидаясь AUTO_TIMEOUT.
  - Джойстик в покое (стик в дедзоне) больше не блокирует авто:
    joystick_control v2 замолкает когда стик в дедзоне, поэтому manual_fresh
    становится False и авто получает управление.
  - MANUAL_TIMEOUT уменьшен обратно до 0.3с — достаточно для keep-alive 20 Гц
    (50мс между пакетами << 300мс таймаут).
  - AUTO_TIMEOUT уменьшен до 0.3с — после явного /auto_enable=False или
    смерти процесса робот останавливается быстро.
  - При смене режима публикуем 3 нулевых Twist подряд для надёжного стопа.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('manual_timeout', 0.30)  # 300мс — keep-alive джойстика 20 Гц
        self.declare_parameter('auto_timeout',   0.30)  # 300мс — быстрый стоп после смерти авто
        self.declare_parameter('loop_hz',        50.0)

        self.MANUAL_TIMEOUT = self.get_parameter('manual_timeout').value
        self.AUTO_TIMEOUT   = self.get_parameter('auto_timeout').value
        hz                  = self.get_parameter('loop_hz').value

        # ── Topics ───────────────────────────────────────────────────────────
        self.sub_manual      = self.create_subscription(Twist, '/cmd_vel/manual', self._cb_manual, 10)
        self.sub_auto        = self.create_subscription(Twist, '/cmd_vel/auto',   self._cb_auto,   10)
        self.sub_auto_enable = self.create_subscription(Bool,  '/auto_enable',    self._cb_enable, 10)
        self.pub_cmd         = self.create_publisher(Twist,  '/cmd_vel',       10)
        self.pub_mode        = self.create_publisher(String, '/control_mode',  10)

        # ── State ────────────────────────────────────────────────────────────
        self.t_manual      = 0.0
        self.t_auto        = 0.0
        self.cmd_manual    = Twist()
        self.cmd_auto      = Twist()
        self.auto_enabled  = False  # явный флаг от web_control
        self.last_mode     = ''
        self._zero_flush   = 0      # счётчик нулевых пакетов при смене режима

        self.timer = self.create_timer(1.0 / hz, self._loop)
        self.get_logger().info(
            f'CmdVelMux v3: manual={self.MANUAL_TIMEOUT}s '
            f'auto={self.AUTO_TIMEOUT}s auto_enabled={self.auto_enabled} @ {hz}Hz'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_manual(self, msg: Twist):
        self.cmd_manual = msg
        self.t_manual   = time.time()

    def _cb_auto(self, msg: Twist):
        self.cmd_auto = msg
        self.t_auto   = time.time()

    def _cb_enable(self, msg: Bool):
        prev = self.auto_enabled
        self.auto_enabled = msg.data
        if prev and not msg.data:
            # Авто выключили явно → немедленно сбросить t_auto
            self.t_auto = 0.0
            self._zero_flush = 5  # пять нулевых пакетов подряд
            self.get_logger().info('Auto disabled via /auto_enable → immediate stop')

    # ── Main loop ─────────────────────────────────────────────────────────────
    def _loop(self):
        now  = time.time()

        # Если нужен принудительный сброс — шлём нули
        if self._zero_flush > 0:
            self.pub_cmd.publish(Twist())
            self._zero_flush -= 1
            self._publish_mode('STOP')
            return

        mode = self._select_mode(now)

        if mode == 'MANUAL':
            self.pub_cmd.publish(self.cmd_manual)
        elif mode == 'AUTO':
            self.pub_cmd.publish(self.cmd_auto)
        else:
            self.pub_cmd.publish(Twist())

        self._publish_mode(mode)

    def _select_mode(self, now: float) -> str:
        manual_fresh = (now - self.t_manual) < self.MANUAL_TIMEOUT
        # Авто активно только если ЯВНО включён И топик свежий
        auto_fresh   = (self.auto_enabled
                        and (now - self.t_auto) < self.AUTO_TIMEOUT)

        if manual_fresh:
            return 'MANUAL'
        if auto_fresh:
            return 'AUTO'
        return 'STOP'

    def _publish_mode(self, mode: str):
        if mode != self.last_mode:
            self.last_mode = mode
            m = String(); m.data = mode
            self.pub_mode.publish(m)
            self.get_logger().info(f'Mode → {mode}')


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
