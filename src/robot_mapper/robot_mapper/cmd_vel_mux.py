#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.sub_manual = self.create_subscription(Twist, '/cmd_vel/manual', self.cb_manual, 10)
        self.sub_auto   = self.create_subscription(Twist, '/cmd_vel/auto',   self.cb_auto, 10)
        self.pub_cmd    = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_manual_time = 0.0
        self.last_auto_time   = 0.0
        self.cmd_manual = Twist()
        self.cmd_auto   = Twist()
        
        self.MANUAL_TIMEOUT = 0.5   # Приоритет ручного управления
        self.AUTO_TIMEOUT   = 1.0   # Если авто молчит >1с -> безопасно нулим

        self.timer = self.create_timer(0.05, self.publish_cmd)
        self.get_logger().info('🔄 CMD_VEL Mux: Manual > Auto | Auto Stale Protection ON')

    def cb_manual(self, msg):
        self.cmd_manual = msg
        self.last_manual_time = time.time()

    def cb_auto(self, msg):
        self.cmd_auto = msg
        self.last_auto_time = time.time()

    def publish_cmd(self):
        now = time.time()
        # 1. Если было ручное управление <0.5с назад -> публикуем manual
        if now - self.last_manual_time < self.MANUAL_TIMEOUT:
            self.pub_cmd.publish(self.cmd_manual)
        # 2. Если авто-нода молчит >1с (умерла/остановлена) -> принудительно ноль
        elif now - self.last_auto_time > self.AUTO_TIMEOUT:
            self.pub_cmd.publish(Twist())
        # 3. Иначе -> публикуем актуальную auto-команду
        else:
            self.pub_cmd.publish(self.cmd_auto)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
