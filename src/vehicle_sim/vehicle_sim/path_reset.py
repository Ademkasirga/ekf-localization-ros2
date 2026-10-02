"""Helpers to clear nav_msgs/Path when a new run or sim reset is detected."""

import math
from typing import Optional

from nav_msgs.msg import Path


class PathRecorder:
    """Accumulates poses and publishes Path; clears on time rewind or position jump."""

    def __init__(
        self,
        frame_id: str,
        jump_reset_m: float = 1.5,
        min_step_m: float = 0.02,
        min_start_motion_m: float = 0.04,
    ) -> None:
        self._path = Path()
        self._path.header.frame_id = frame_id
        self._jump_reset_m = float(jump_reset_m)
        self._min_step_m = float(min_step_m)
        self._min_start_motion_m = float(min_start_motion_m)
        self._last_t: Optional[float] = None
        self._motion_anchor: Optional[tuple[float, float]] = None

    @property
    def message(self) -> Path:
        return self._path

    def set_frame_id(self, frame_id: str) -> None:
        self._path.header.frame_id = frame_id

    def set_jump_reset_m(self, jump_reset_m: float) -> None:
        self._jump_reset_m = float(jump_reset_m)

    def clear(self, stamp) -> None:
        self._path.poses.clear()
        self._path.header.stamp = stamp
        self._last_t = _stamp_sec(stamp)
        self._motion_anchor = None

    def maybe_clear(self, stamp, x: float, y: float) -> bool:
        """Clear path if sim time went backwards or the pose jumped (respawn)."""
        t = _stamp_sec(stamp)
        if self._last_t is not None:
            if t < self._last_t - 0.5:
                self.clear(stamp)
                return True
            if self._path.poses:
                last = self._path.poses[-1].pose.position
                if math.hypot(x - last.x, y - last.y) > self._jump_reset_m:
                    self.clear(stamp)
                    return True
        self._last_t = t
        return False

    def append(self, stamp, pose_stamped, max_poses: int) -> bool:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        self.maybe_clear(stamp, x, y)
        if self._motion_anchor is None:
            self._motion_anchor = (x, y)
        if not self._path.poses:
            ax, ay = self._motion_anchor
            if math.hypot(x - ax, y - ay) < self._min_start_motion_m:
                return False
        if self._path.poses:
            last = self._path.poses[-1].pose.position
            gap = math.hypot(x - last.x, y - last.y)
            if gap > self._jump_reset_m:
                self.clear(stamp)
                self._motion_anchor = (x, y)
                return False
            if gap < self._min_step_m:
                return False
        self._path.header.stamp = stamp
        self._path.poses.append(pose_stamped)
        if len(self._path.poses) > max_poses:
            self._path.poses.pop(0)
        return True


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9
