#!/usr/bin/env python3
"""Publishes ground truth, noisy IMU, and noisy GPS from a 2D kinematic path."""

import math

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from vehicle_sim.path_profiles import state_at_time
from vehicle_sim.path_reset import PathRecorder


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class VehicleSimulatorNode(Node):

    def __init__(self) -> None:
        super().__init__('vehicle_simulator')

        self.declare_parameter('path_profile', 'circle')
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('circle_radius', 20.0)
        self.declare_parameter('figure8_scale', 15.0)
        self.declare_parameter('imu_rate_hz', 100.0)
        self.declare_parameter('gps_rate_hz', 5.0)
        self.declare_parameter('imu_gyro_std', 0.01)
        self.declare_parameter('imu_accel_std', 0.05)
        self.declare_parameter('gps_position_std', 2.0)
        self.declare_parameter('gps_outage_start', -1.0)
        self.declare_parameter('gps_outage_duration', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_path', True)
        self.declare_parameter('path_jump_reset_m', 50.0)

        self._start_time = self.get_clock().now()
        frame = self.get_parameter('frame_id').value
        jump = float(self.get_parameter('path_jump_reset_m').value)
        self._path_rec = PathRecorder(frame, jump_reset_m=jump)
        self._startup_path_cleared = False

        self._gt_pub = self.create_publisher(Odometry, 'ground_truth', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu', 50)
        self._gps_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'gps/pose', 10
        )
        self._gt_pose_pub = self.create_publisher(
            PoseStamped, 'ground_truth/pose', 10
        )
        self._path_pub = self.create_publisher(Path, 'ground_truth/path', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        imu_period = 1.0 / self.get_parameter('imu_rate_hz').value
        self._imu_timer = self.create_timer(imu_period, self._on_imu_timer)

        gps_period = 1.0 / self.get_parameter('gps_rate_hz').value
        self._gps_timer = self.create_timer(gps_period, self._on_gps_timer)
        self.create_timer(0.2, self._startup_clear_path_cb)

        self._rng = np.random.default_rng(seed=42)

    def _startup_clear_path_cb(self) -> None:
        if self._startup_path_cleared:
            return
        self._startup_path_cleared = True
        stamp = self.get_clock().now().to_msg()
        self._path_rec.clear(stamp)
        self._path_pub.publish(self._path_rec.message)

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _ground_truth(self):
        t = self._elapsed()
        p = self.get_parameter('path_profile').value
        speed = self.get_parameter('speed').value
        r = self.get_parameter('circle_radius').value
        s8 = self.get_parameter('figure8_scale').value
        return state_at_time(t, p, speed, r, s8)

    def _gps_active(self) -> bool:
        t = self._elapsed()
        start = self.get_parameter('gps_outage_start').value
        duration = self.get_parameter('gps_outage_duration').value
        if start < 0.0 or duration <= 0.0:
            return True
        return not (start <= t < start + duration)

    def _publish_tf_and_gt(self, stamp, x, y, yaw, v):
        frame = self.get_parameter('frame_id').value
        child = self.get_parameter('child_frame_id').value

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = yaw_to_quaternion(yaw)
        self._tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame
        odom.child_frame_id = child
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        odom.twist.twist.linear.x = v
        self._gt_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self._gt_pose_pub.publish(pose)

        if self.get_parameter('publish_path').value:
            self._path_rec.set_frame_id(frame)
            self._path_rec.append(stamp, pose, 5000)
            self._path_pub.publish(self._path_rec.message)

    def _on_imu_timer(self) -> None:
        x, y, yaw, v, yaw_rate, a_x = self._ground_truth()
        stamp = self.get_clock().now().to_msg()

        self._publish_tf_and_gt(stamp, x, y, yaw, v)

        gyro_std = self.get_parameter('imu_gyro_std').value
        accel_std = self.get_parameter('imu_accel_std').value

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.get_parameter('child_frame_id').value
        imu.angular_velocity.z = yaw_rate + self._rng.normal(0.0, gyro_std)
        imu.linear_acceleration.x = a_x + self._rng.normal(0.0, accel_std)
        self._imu_pub.publish(imu)

    def _on_gps_timer(self) -> None:
        if not self._gps_active():
            return

        x, y, yaw, _, _, _ = self._ground_truth()
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter('frame_id').value
        std = self.get_parameter('gps_position_std').value
        var = std * std

        gps = PoseWithCovarianceStamped()
        gps.header.stamp = stamp
        gps.header.frame_id = frame
        gps.pose.pose.position.x = x + self._rng.normal(0.0, std)
        gps.pose.pose.position.y = y + self._rng.normal(0.0, std)
        gps.pose.pose.orientation = yaw_to_quaternion(yaw)

        cov = [0.0] * 36
        cov[0] = var
        cov[7] = var
        cov[35] = 0.5
        gps.pose.covariance = cov
        self._gps_pub.publish(gps)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
