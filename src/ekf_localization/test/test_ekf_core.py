import math

from ekf_localization.ekf_core import Ekf2D
import numpy as np


def test_predict_straight_line_increases_x():
    ekf = Ekf2D(x0=np.array([0.0, 0.0, 0.0, 2.0]))
    ekf.predict(dt=0.1, yaw_rate=0.0, accel_x=0.0)
    assert ekf.x[0] > 0.19
    assert abs(ekf.x[1]) < 1e-6
    assert ekf.x[3] == 2.0


def test_gps_update_pulls_state_toward_measurement():
    ekf = Ekf2D(x0=np.array([0.0, 0.0, 0.0, 0.0]))
    ekf.P = np.diag([100.0, 100.0, 1.0, 1.0])
    ekf.set_gps_noise_variance(1.0)
    z = np.array([10.0, 0.0])
    ekf.update_gps(z)
    assert ekf.x[0] > 5.0
    assert ekf.x[0] < 10.0


def test_predict_turn_changes_yaw():
    ekf = Ekf2D(x0=np.array([0.0, 0.0, 0.0, 1.0]))
    ekf.predict(dt=0.5, yaw_rate=0.2, accel_x=0.0)
    assert abs(ekf.x[2] - 0.1) < 1e-9


def test_position_heading_update_wraps_yaw():
    ekf = Ekf2D(x0=np.array([0.0, 0.0, 3.0, 1.0]))
    ekf.P = np.diag([1.0, 1.0, 0.5, 0.5])
    z = np.array([0.0, 0.0, -3.1])
    r = np.diag([0.5, 0.5, 0.1])
    ekf.update_position_heading(z, r)
    ekf.normalize_yaw()
    yaw_err = math.atan2(
        math.sin(ekf.x[2] - z[2]), math.cos(ekf.x[2] - z[2])
    )
    assert abs(yaw_err) < 0.2


def test_circle_predict_drift_without_gps():
    ekf = Ekf2D(x0=np.array([0.0, 0.0, 0.0, 2.0]))
    omega = 2.0 / 20.0
    for _ in range(100):
        ekf.predict(dt=0.1, yaw_rate=omega, accel_x=0.0)
    assert math.hypot(ekf.x[0], ekf.x[1]) > 1.0
