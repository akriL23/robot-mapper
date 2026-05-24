#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.5)
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/manual', 10)
        self.get_logger().info('Joystick control ready')

    def joy_callback(self, msg: Joy):
        twist = Twist()
        twist.linear.x = -msg.axes[1] * self.linear_scale
        twist.angular.z = -msg.axes[0] * self.angular_scale
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
