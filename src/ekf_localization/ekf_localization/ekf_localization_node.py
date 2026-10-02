#!/usr/bin/env python3
"""ROS 2 node fusing IMU (predict) and GPS pose (update) with Ekf2D."""

import math

from ekf_localization.ekf_core import Ekf2D
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


class EkfLocalizationNode(Node):

    def __init__(self) -> None:
        super().__init__('ekf_localization')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('process_noise', [0.01, 0.01, 0.001, 0.05])
        self.declare_parameter('gps_noise_std', 2.0)
        self.declare_parameter('gps_yaw_noise_std', 0.15)
        self.declare_parameter('fuse_gps_yaw', True)
        self.declare_parameter('use_imu_accel', False)
        self.declare_parameter('wheel_odom_topic', '')
        self.declare_parameter('wheel_speed_noise_std', 0.08)
        self.declare_parameter('predict_from_wheel_odom', False)
        self.declare_parameter('gps_measurement_scale', 1.0)
        self.declare_parameter('initial_position_std', 5.0)
        self.declare_parameter('publish_path', True)
        self.declare_parameter('path_jump_reset_m', 1.5)
        self.declare_parameter('path_min_step_m', 0.02)
        self.declare_parameter('path_min_start_motion_m', 0.04)
        self.declare_parameter('max_dt', 0.5)

        pn = np.array(self.get_parameter('process_noise').value, dtype=float)
        gps_std = float(self.get_parameter('gps_noise_std').value)
        self._ekf = Ekf2D(
            process_noise_diag=pn,
            gps_noise_var=gps_std ** 2,
        )

        self._last_imu_time = None
        self._last_wheel_time = None
        self._initialized = False
        self._path = Path()
        self._path.header.frame_id = self.get_parameter('frame_id').value
        self._last_path_t = None
        self._path_motion_anchor = None
        self._startup_path_cleared = False

        self._odom_pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self._pose_pub = self.create_publisher(PoseStamped, 'ekf/pose', 10)
        self._path_pub = self.create_publisher(Path, 'ekf/path', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, 'imu', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'gps/pose', self._gps_cb, 10)

        wheel_topic = self.get_parameter('wheel_odom_topic').value
        if wheel_topic:
            self.create_subscription(Odometry, wheel_topic, self._wheel_odom_cb, 10)

        self.create_timer(0.05, self._publish_timer_cb)
        self.create_timer(0.2, self._startup_clear_path_cb)

        self.get_logger().info('EKF localization node started')

    def _startup_clear_path_cb(self) -> None:
        if self._startup_path_cleared:
            return
        self._startup_path_cleared = True
        self._reset_path(self.get_clock().now().to_msg())

    def _reset_path(self, stamp) -> None:
        self._path.poses.clear()
        self._path.header.stamp = stamp
        self._path.header.frame_id = self.get_parameter('frame_id').value
        self._path_pub.publish(self._path)
        self._last_path_t = self._stamp_to_sec(stamp)
        self._path_motion_anchor = None

    def _maybe_reset_path(self, stamp, x: float, y: float) -> None:
        t = self._stamp_to_sec(stamp)
        if self._last_path_t is not None:
            if t < self._last_path_t - 0.5:
                self._reset_path(stamp)
            elif self._path.poses:
                last = self._path.poses[-1].pose.position
                jump = self.get_parameter('path_jump_reset_m').value
                if math.hypot(x - last.x, y - last.y) > jump:
                    self._reset_path(stamp)
        self._last_path_t = t

    def _publish_timer_cb(self) -> None:
        if not self._initialized:
            return
        self._publish_state(self.get_clock().now().to_msg(), append_path=True)

    def _stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _publish_state(self, stamp, append_path: bool = False) -> None:
        frame = self.get_parameter('frame_id').value
        child = self.get_parameter('child_frame_id').value
        x, y, yaw, v = self._ekf.x

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame
        t.child_frame_id = 'base_link_ekf'
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

        P = self._ekf.P
        pose_cov = [0.0] * 36
        pose_cov[0] = P[0, 0]
        pose_cov[1] = P[0, 1]
        pose_cov[6] = P[1, 0]
        pose_cov[7] = P[1, 1]
        pose_cov[35] = P[2, 2]
        odom.pose.covariance = pose_cov
        odom.twist.twist.linear.x = v
        self._odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self._pose_pub.publish(pose)

        if append_path and self.get_parameter('publish_path').value:
            self._maybe_reset_path(stamp, x, y)
            if self._append_path_pose(stamp, pose, x, y):
                self._path_pub.publish(self._path)

    def _append_path_pose(self, stamp, pose, x: float, y: float) -> bool:
        min_start = float(self.get_parameter('path_min_start_motion_m').value)
        min_step = float(self.get_parameter('path_min_step_m').value)
        if self._path_motion_anchor is None:
            self._path_motion_anchor = (x, y)
        if not self._path.poses:
            ax, ay = self._path_motion_anchor
            if math.hypot(x - ax, y - ay) < min_start:
                return False
        if self._path.poses:
            last = self._path.poses[-1].pose.position
            gap = math.hypot(x - last.x, y - last.y)
            jump = float(self.get_parameter('path_jump_reset_m').value)
            if gap > jump:
                self._reset_path(stamp)
                self._path_motion_anchor = (x, y)
                return False
            if gap < min_step:
                return False
        self._path.header.stamp = stamp
        self._path.poses.append(pose)
        if len(self._path.poses) > 5000:
            self._path.poses.pop(0)
        return True

    def _imu_cb(self, msg: Imu) -> None:
        t = self._stamp_to_sec(msg.header.stamp)
        if self._last_imu_time is None:
            self._last_imu_time = t
            return

        dt = t - self._last_imu_time
        self._last_imu_time = t
        max_dt = self.get_parameter('max_dt').value
        if dt <= 0.0 or dt > max_dt:
            self._last_imu_time = t
            return

        if not self._initialized:
            return

        if bool(self.get_parameter('predict_from_wheel_odom').value):
            return

        yaw_rate = msg.angular_velocity.z
        accel_x = msg.linear_acceleration.x
        integrate_accel = bool(self.get_parameter('use_imu_accel').value)
        self._ekf.predict(dt, yaw_rate, accel_x, integrate_accel=integrate_accel)
        self._ekf.normalize_yaw()
        self._publish_state(msg.header.stamp, append_path=False)

    def _gps_cb(self, msg: PoseWithCovarianceStamped) -> None:
        z = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y],
            dtype=float,
        )

        if not self._initialized:
            yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            self._ekf.x = np.array([z[0], z[1], yaw, 0.0])
            pos_std = self.get_parameter('initial_position_std').value
            self._ekf.P = np.diag([pos_std ** 2, pos_std ** 2, 0.5, 1.0])
            self._initialized = True
            self._reset_path(msg.header.stamp)
            self.get_logger().info('EKF initialized from first GPS fix')
            self._publish_state(msg.header.stamp, append_path=False)
            return

        fuse_yaw = bool(self.get_parameter('fuse_gps_yaw').value)
        gps_std = float(self.get_parameter('gps_noise_std').value)
        meas_scale = float(self.get_parameter('gps_measurement_scale').value)
        if msg.pose.covariance[0] > 0.0:
            var_x = msg.pose.covariance[0] * meas_scale
            var_y = msg.pose.covariance[7] * meas_scale
        else:
            var_x = var_y = (gps_std ** 2) * meas_scale

        if fuse_yaw:
            yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            yaw_std = float(self.get_parameter('gps_yaw_noise_std').value)
            if msg.pose.covariance[35] > 0.0:
                var_yaw = msg.pose.covariance[35] * meas_scale
            else:
                var_yaw = (yaw_std ** 2) * meas_scale
            meas_cov = np.diag([var_x, var_y, var_yaw])
            self._ekf.update_position_heading(
                np.array([z[0], z[1], yaw]), meas_cov
            )
        else:
            self._ekf.R = np.diag([var_x, var_y])
            self._ekf.update_gps(z)

        self._ekf.normalize_yaw()
        self._publish_state(msg.header.stamp, append_path=False)

    def _wheel_odom_cb(self, msg: Odometry) -> None:
        if not self._initialized:
            return

        t = self._stamp_to_sec(msg.header.stamp)
        v = float(msg.twist.twist.linear.x)
        omega = float(msg.twist.twist.angular.z)
        max_dt = self.get_parameter('max_dt').value

        if bool(self.get_parameter('predict_from_wheel_odom').value):
            if self._last_wheel_time is not None:
                dt = t - self._last_wheel_time
                if 0.0 < dt <= max_dt:
                    self._ekf.predict(
                        dt, omega, integrate_accel=False
                    )
            self._last_wheel_time = t

        std = float(self.get_parameter('wheel_speed_noise_std').value)
        self._ekf.update_wheel_speed(v, std ** 2)
        self._ekf.normalize_yaw()
        self._publish_state(msg.header.stamp, append_path=False)


def main(args=None):
    rclpy.init(args=args)
    node = EkfLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
