#!/usr/bin/env python3
"""Adds Gaussian noise to odometry pose as a learning GPS substitute."""

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

import rclpy
from rclpy.node import Node


class GazeboFakeGpsNode(Node):

    def __init__(self) -> None:
        super().__init__('gazebo_fake_gps')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', 'gps/pose')
        self.declare_parameter('gps_position_std', 2.0)
        self.declare_parameter('gps_yaw_std', 0.08)
        self.declare_parameter('frame_id', 'map')

        self._std = float(self.get_parameter('gps_position_std').value)
        self._rng = np.random.default_rng(7)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.create_subscription(Odometry, in_topic, self._cb, 10)
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, out_topic, 10
        )

    def _cb(self, msg: Odometry) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self.get_parameter('frame_id').value
        out.pose.pose.position.x = (
            msg.pose.pose.position.x + self._rng.normal(0.0, self._std)
        )
        out.pose.pose.position.y = (
            msg.pose.pose.position.y + self._rng.normal(0.0, self._std)
        )
        out.pose.pose.orientation = msg.pose.pose.orientation
        yaw_std = float(self.get_parameter('gps_yaw_std').value)
        var = self._std ** 2
        cov = [0.0] * 36
        cov[0] = var
        cov[7] = var
        cov[35] = yaw_std ** 2
        out.pose.covariance = cov
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboFakeGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
