#!/usr/bin/env python3
"""Publishes circular cmd_vel for Gazebo diff-drive demo."""

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node


class CmdVelDemoNode(Node):

    def __init__(self) -> None:
        super().__init__('cmd_vel_demo')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.15)
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.05, self._on_timer)

    def _on_timer(self) -> None:
        msg = Twist()
        msg.linear.x = float(self.get_parameter('linear_speed').value)
        msg.angular.z = float(self.get_parameter('angular_speed').value)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
