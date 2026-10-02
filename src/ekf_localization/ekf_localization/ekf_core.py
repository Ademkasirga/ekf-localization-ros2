"""2D Extended Kalman Filter: state [x, y, yaw, v], IMU predict, GPS position update."""

import math
from typing import Optional

import numpy as np


class Ekf2D:
    """EKF for planar vehicle with IMU propagation and GPS (x, y) measurements."""

    STATE_DIM = 4

    def __init__(
        self,
        x0: Optional[np.ndarray] = None,
        P0: Optional[np.ndarray] = None,
        process_noise_diag: Optional[np.ndarray] = None,
        gps_noise_var: float = 4.0,
    ) -> None:
        self.x = np.zeros(self.STATE_DIM) if x0 is None else x0.astype(float).copy()
        if P0 is None:
            self.P = np.diag([10.0, 10.0, 0.5, 1.0])
        else:
            self.P = P0.astype(float).copy()

        if process_noise_diag is None:
            self.Q = np.diag([0.01, 0.01, 0.001, 0.05])
        else:
            self.Q = np.diag(process_noise_diag.astype(float))

        self.R = np.eye(2) * float(gps_noise_var)
        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ]
        )

    def set_gps_noise_variance(self, var_xy: float) -> None:
        self.R = np.eye(2) * float(var_xy)

    def set_process_noise_diag(self, diag: np.ndarray) -> None:
        self.Q = np.diag(diag.astype(float))

    def predict(
        self,
        dt: float,
        yaw_rate: float,
        accel_x: float = 0.0,
        integrate_accel: bool = True,
    ) -> None:
        if dt <= 0.0:
            return

        x, y, yaw, v = self.x

        yaw_p = yaw + yaw_rate * dt
        if integrate_accel:
            v_p = v + accel_x * dt
        else:
            v_p = v
        x_p = x + v_p * math.cos(yaw_p) * dt
        y_p = y + v_p * math.sin(yaw_p) * dt

        self.x = np.array([x_p, y_p, yaw_p, v_p])

        sin_y = math.sin(yaw_p)
        cos_y = math.cos(yaw_p)

        F = np.array(
            [
                [1.0, 0.0, -v_p * sin_y * dt, cos_y * dt],
                [0.0, 1.0, v_p * cos_y * dt, sin_y * dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, z_xy: np.ndarray) -> np.ndarray:
        """Fuse GPS position measurement; returns innovation vector."""
        z = z_xy.astype(float).reshape(2)
        x_pred = self.x
        y_innov = z - self.H @ x_pred

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = x_pred + K @ y_innov
        identity = np.eye(self.STATE_DIM)
        self.P = (identity - K @ self.H) @ self.P

        return y_innov

    def update_position_heading(
        self, z_xy_yaw: np.ndarray, meas_cov: np.ndarray
    ) -> np.ndarray:
        """Fuse planar position and heading; returns innovation vector."""
        z = z_xy_yaw.astype(float).reshape(3)
        h = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
            ]
        )
        x_pred = self.x
        y_innov = z - h @ x_pred
        y_innov[2] = math.atan2(
            math.sin(y_innov[2]), math.cos(y_innov[2])
        )

        r = meas_cov.astype(float).reshape(3, 3)
        s = h @ self.P @ h.T + r
        k = self.P @ h.T @ np.linalg.inv(s)

        self.x = x_pred + k @ y_innov
        identity = np.eye(self.STATE_DIM)
        self.P = (identity - k @ h) @ self.P

        return y_innov

    def update_wheel_speed(self, v_meas: float, variance: float) -> float:
        """Fuse longitudinal speed (e.g. wheel odometry); returns innovation."""
        h = np.array([[0.0, 0.0, 0.0, 1.0]])
        x_pred = self.x
        innov = float(v_meas) - h @ x_pred

        r = np.array([[float(variance)]])
        s = h @ self.P @ h.T + r
        k = self.P @ h.T @ np.linalg.inv(s)

        self.x = x_pred + (k @ innov).reshape(4)
        identity = np.eye(self.STATE_DIM)
        self.P = (identity - k @ h) @ self.P

        return innov

    def normalize_yaw(self) -> None:
        self.x[2] = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))
