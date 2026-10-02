#!/usr/bin/env python3
"""Logs position error (ground truth vs GPS vs EKF) for tuning analysis."""

import csv
import os
from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class ErrorLoggerNode(Node):

    def __init__(self) -> None:
        super().__init__('error_logger')

        self.declare_parameter('output_csv', '')
        self.declare_parameter('log_rate_hz', 10.0)

        self._gt: Optional[Odometry] = None
        self._gps_xy: Optional[tuple] = None
        self._ekf: Optional[Odometry] = None

        out = self.get_parameter('output_csv').value
        if not out:
            out = os.path.join(os.getcwd(), 'ekf_error_log.csv')
        self._csv_path = out
        self._file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                't_sec',
                'gt_x',
                'gt_y',
                'gps_x',
                'gps_y',
                'ekf_x',
                'ekf_y',
                'gps_err',
                'ekf_err',
            ]
        )

        self.create_subscription(Odometry, 'ground_truth', self._gt_cb, 10)
        self.create_subscription(
            Odometry, 'odometry/filtered', self._ekf_cb, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 'gps/pose', self._gps_cb, 10
        )

        period = 1.0 / self.get_parameter('log_rate_hz').value
        self.create_timer(period, self._log_timer)
        self.get_logger().info(f'Logging errors to {self._csv_path}')

    def _gt_cb(self, msg: Odometry) -> None:
        self._gt = msg

    def _ekf_cb(self, msg: Odometry) -> None:
        self._ekf = msg

    def _gps_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._gps_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def _stamp_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _log_timer(self) -> None:
        if self._gt is None:
            return
        gt_x = self._gt.pose.pose.position.x
        gt_y = self._gt.pose.pose.position.y
        t = self._stamp_sec(self._gt.header.stamp)

        gps_x = gps_y = float('nan')
        gps_err = float('nan')
        if self._gps_xy is not None:
            gps_x, gps_y = self._gps_xy
            gps_err = ((gps_x - gt_x) ** 2 + (gps_y - gt_y) ** 2) ** 0.5

        ekf_x = ekf_y = float('nan')
        ekf_err = float('nan')
        if self._ekf is not None:
            ekf_x = self._ekf.pose.pose.position.x
            ekf_y = self._ekf.pose.pose.position.y
            ekf_err = ((ekf_x - gt_x) ** 2 + (ekf_y - gt_y) ** 2) ** 0.5

        self._writer.writerow(
            [t, gt_x, gt_y, gps_x, gps_y, ekf_x, ekf_y, gps_err, ekf_err]
        )

    def destroy_node(self):
        self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ErrorLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
