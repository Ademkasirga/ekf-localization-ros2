"""2D reference paths for the kinematic vehicle simulator."""

import math
from typing import Tuple


def state_at_time(
    t: float,
    profile: str,
    speed: float = 2.0,
    circle_radius: float = 20.0,
    figure8_scale: float = 15.0,
) -> Tuple[float, float, float, float, float, float]:
    """
    Return ground-truth (x, y, yaw, v, yaw_rate, body_accel_x) at time t [s].

    yaw is heading in map frame; body_accel_x is forward acceleration in body frame.
    """
    profile = profile.lower()

    if profile == 'straight':
        v = speed
        x = v * t
        y = 0.0
        yaw = 0.0
        yaw_rate = 0.0
        a_x = 0.0
        return x, y, yaw, v, yaw_rate, a_x

    if profile == 'circle':
        omega = speed / circle_radius
        yaw = omega * t
        x = circle_radius * math.sin(yaw)
        y = circle_radius * (1.0 - math.cos(yaw))
        v = speed
        yaw_rate = omega
        a_x = 0.0
        return x, y, yaw, v, yaw_rate, a_x

    if profile == 'figure8':
        # Lemniscate-like parametric curve
        scale = figure8_scale
        omega = speed / scale
        theta = omega * t
        x = scale * math.sin(theta)
        y = scale * math.sin(theta) * math.cos(theta)
        dx = scale * math.cos(theta) * omega
        dy = scale * (math.cos(theta) ** 2 - math.sin(theta) ** 2) * omega
        yaw = math.atan2(dy, dx)
        v = math.hypot(dx, dy)
        # Approximate yaw rate from curvature
        if v > 1e-3:
            ddx = -scale * math.sin(theta) * omega ** 2
            ddy = -4.0 * scale * math.sin(theta) * math.cos(theta) * omega ** 2
            yaw_rate = (ddy * dx - ddx * dy) / (v * v)
            a_x = (dx * ddx + dy * ddy) / v
        else:
            yaw_rate = 0.0
            a_x = 0.0
        return x, y, yaw, v, yaw_rate, a_x

    raise ValueError(f'Unknown path profile: {profile}')
