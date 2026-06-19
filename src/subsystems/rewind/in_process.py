"""Record certified joint targets and rewind them along the same geometry.

The controller is intentionally hardware-free. ``game_controller`` owns all
bus I/O and calls this class in the same way that it calls the in-process
jogging controller: gameplay targets are recorded with :meth:`record_target`,
then reset ticks request a target from :meth:`next_target`.

Recorded ``time_from_start`` values preserve gameplay timing as metadata.
Rewind timing is generated independently from joint-space geometry and the
configured velocity limits; recorded timing never affects robot motion.
"""

from __future__ import annotations

import bisect
import math
from typing import Any

from compas_fab.robots import Duration, JointTrajectory, JointTrajectoryPoint
from compas_robots.model import Joint

from subsystems.robot.shared_compas_scene import UR10E_JOINT_NAMES


_AXES = 6
_JOINT_NAMES = list(UR10E_JOINT_NAMES)
_JOINT_TYPES = [Joint.REVOLUTE] * _AXES


class RewindController:
    """Own one team's recorded and velocity-retimed rewind trajectories.

    Parameters
    ----------
    enabled:
        Enables recording and reset-stage rewind for this team.
    max_velocity_rad_s:
        Per-joint configured maximum velocities in radians per second.
    speed_fraction:
        Fraction of each configured maximum used to retime the rewind path.
    arrival_tolerance_rad:
        Maximum absolute error allowed on every joint at completion.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        max_velocity_rad_s: list[float],
        speed_fraction: float,
        arrival_tolerance_rad: float,
    ) -> None:
        self.enabled = bool(enabled)
        self.max_velocity_rad_s = _six_floats(max_velocity_rad_s, 1.0)
        self.speed_fraction = max(1e-6, min(1.0, float(speed_fraction)))
        self.arrival_tolerance_rad = max(0.0, float(arrival_tolerance_rad))

        # The recorded trajectory keeps gameplay-relative timestamps for later
        # offline analysis; rewind_trajectory owns newly generated timing.
        self.recorded_trajectory = _empty_trajectory()
        self.rewind_trajectory = _empty_trajectory()
        self._recording_started_s: float | None = None
        self._rewind_elapsed_s = 0.0
        self._rewind_times_s: list[float] = []
        self._initial_q_rad: list[float] | None = None
        self._status = "disabled" if not self.enabled else "idle"
        self._current_index = 0
        self._max_error_rad: float | None = None

    def start_recording(
        self, initial_q_rad: list[float] | None, *, now_s: float
    ) -> None:
        """Clear the prior game and capture its measured play-entry pose.

        Called by the game stage-entry hook. A boot directly into play may not
        have robot telemetry yet; :meth:`ensure_recording_started` captures the
        first measured pose later in that case.
        """

        if not self.enabled:
            return
        self.recorded_trajectory = _empty_trajectory()
        self.rewind_trajectory = _empty_trajectory()
        self._recording_started_s = None
        self._initial_q_rad = None
        self._rewind_times_s = []
        self._rewind_elapsed_s = 0.0
        self._current_index = 0
        self._max_error_rad = None
        self._status = "awaiting_initial_pose"
        self.ensure_recording_started(initial_q_rad, now_s=now_s)

    def ensure_recording_started(
        self, q_actual_rad: list[float] | None, *, now_s: float
    ) -> bool:
        """Capture the measured initial pose once robot telemetry is available."""

        if not self.enabled:
            return False
        q = _valid_q(q_actual_rad)
        if self._initial_q_rad is not None:
            return True
        if q is None:
            return False
        self._initial_q_rad = q
        self._recording_started_s = float(now_s)
        self.recorded_trajectory.points.append(_point(q, 0.0))
        self._status = "recording"
        return True

    def record_target(self, q_target_rad: list[float], *, now_s: float) -> None:
        """Append one collision-certified target published during gameplay."""

        if self._status != "recording" or self._recording_started_s is None:
            return
        q = _valid_q(q_target_rad)
        if q is None:
            return
        elapsed_s = max(0.0, float(now_s) - self._recording_started_s)
        self.recorded_trajectory.points.append(_point(q, elapsed_s))

    def start_rewind(self) -> bool:
        """Reverse and velocity-retime the recorded path for reset execution.

        Segment duration is the slowest time required by any moving joint at
        its configured rewind velocity. This preserves straight interpolation
        between every pair of certified targets.

        TODO(rewind-acceleration): add forward/backward acceleration retiming
        after the end-to-end hardware workflow has been validated.
        """

        if not self.enabled:
            return False
        source_points = list(self.recorded_trajectory.points)
        if not source_points or self._initial_q_rad is None:
            self._status = "unavailable"
            return False

        reversed_q = [
            list(point.joint_values[:_AXES]) for point in reversed(source_points)
        ]
        # Always terminate at the measured play-entry pose, even when the first
        # certified command happened to duplicate or slightly differ from it.
        if reversed_q[-1] != self._initial_q_rad:
            reversed_q.append(list(self._initial_q_rad))

        points: list[JointTrajectoryPoint] = []
        elapsed_s = 0.0
        previous_q: list[float] | None = None
        rewind_limits = [
            max(1e-9, velocity * self.speed_fraction)
            for velocity in self.max_velocity_rad_s
        ]
        for q in reversed_q:
            if previous_q is not None:
                elapsed_s += max(
                    abs(q[axis] - previous_q[axis]) / rewind_limits[axis]
                    for axis in range(_AXES)
                )
            points.append(_point(q, elapsed_s))
            previous_q = q

        self.rewind_trajectory = JointTrajectory(
            trajectory_points=points,
            joint_names=list(_JOINT_NAMES),
        )
        self._rewind_times_s = [point.time_from_start.seconds for point in points]
        self._rewind_elapsed_s = 0.0
        self._current_index = 0
        self._max_error_rad = None
        # Robot actual may lag the final gameplay command. Hold that certified
        # endpoint first, then start moving backward only after measured arrival.
        self._status = "aligning_start" if elapsed_s > 0.0 else "settling"
        return True

    def next_target(
        self, *, dt_s: float, q_actual_rad: list[float] | None
    ) -> list[float] | None:
        """Advance rewind time and return the interpolated robot joint target.

        The caller invokes this only on active reset ticks. Pauses therefore
        freeze progress naturally, while robot faults and bus safety remain
        owned by the surrounding game controller.
        """

        points = list(self.rewind_trajectory.points)
        if self._status not in {
            "aligning_start",
            "rewinding",
            "settling",
            "complete",
        } or not points:
            return None
        if self._status == "complete":
            return list(points[-1].joint_values[:_AXES])

        if self._status == "aligning_start":
            start_q = list(points[0].joint_values[:_AXES])
            actual = _valid_q(q_actual_rad)
            if actual is None or max(
                abs(actual[axis] - start_q[axis]) for axis in range(_AXES)
            ) > self.arrival_tolerance_rad:
                return start_q
            self._status = "rewinding"

        duration_s = self._rewind_times_s[-1]
        self._rewind_elapsed_s = min(
            duration_s, self._rewind_elapsed_s + max(0.0, float(dt_s))
        )
        target = self._sample(self._rewind_elapsed_s)
        if self._rewind_elapsed_s >= duration_s:
            self._status = "settling"
            actual = _valid_q(q_actual_rad)
            if actual is not None and self._initial_q_rad is not None:
                self._max_error_rad = max(
                    abs(actual[axis] - self._initial_q_rad[axis])
                    for axis in range(_AXES)
                )
                if self._max_error_rad <= self.arrival_tolerance_rad:
                    self._status = "complete"
        return target

    @property
    def complete(self) -> bool:
        """Return whether measured joints reached the recorded initial pose."""

        return self._status == "complete"

    def snapshot(self) -> dict[str, Any]:
        """Build compact rewind state for ``state.full`` and player displays."""

        point_count = len(self.rewind_trajectory.points)
        duration_s = self._rewind_times_s[-1] if self._rewind_times_s else 0.0
        progress = 0.0
        if self._status == "complete":
            progress = 1.0
        elif duration_s > 0.0:
            progress = min(1.0, self._rewind_elapsed_s / duration_s)
        return {
            "enabled": self.enabled,
            "status": self._status,
            "recorded_point_count": len(self.recorded_trajectory.points),
            "point_count": point_count,
            "current_index": self._current_index,
            "progress": progress,
            "initial_q_rad": (
                list(self._initial_q_rad) if self._initial_q_rad is not None else None
            ),
            "max_error_deg": (
                math.degrees(self._max_error_rad)
                if self._max_error_rad is not None
                else None
            ),
        }

    def _sample(self, elapsed_s: float) -> list[float]:
        """Linearly interpolate the velocity-retimed trajectory at one time."""

        points = list(self.rewind_trajectory.points)
        if elapsed_s <= 0.0 or len(points) == 1:
            self._current_index = 0
            return list(points[0].joint_values[:_AXES])
        if elapsed_s >= self._rewind_times_s[-1]:
            self._current_index = len(points) - 1
            return list(points[-1].joint_values[:_AXES])

        upper = bisect.bisect_right(self._rewind_times_s, elapsed_s)
        lower = max(0, upper - 1)
        self._current_index = lower
        t0 = self._rewind_times_s[lower]
        t1 = self._rewind_times_s[upper]
        q0 = points[lower].joint_values
        q1 = points[upper].joint_values
        if t1 <= t0:
            return list(q1[:_AXES])
        alpha = (elapsed_s - t0) / (t1 - t0)
        return [
            float(q0[axis]) + alpha * (float(q1[axis]) - float(q0[axis]))
            for axis in range(_AXES)
        ]


def _empty_trajectory() -> JointTrajectory:
    """Create an empty native COMPAS FAB trajectory in UR10e joint order."""

    return JointTrajectory(trajectory_points=[], joint_names=list(_JOINT_NAMES))


def _point(q_rad: list[float], elapsed_s: float) -> JointTrajectoryPoint:
    """Create one fully labelled revolute trajectory point."""

    return JointTrajectoryPoint(
        joint_values=list(q_rad[:_AXES]),
        joint_types=list(_JOINT_TYPES),
        joint_names=list(_JOINT_NAMES),
        time_from_start=Duration(max(0.0, float(elapsed_s)), 0),
    )


def _valid_q(value: list[float] | None) -> list[float] | None:
    """Return a six-joint float copy, or ``None`` for malformed input."""

    if not isinstance(value, list) or len(value) < _AXES:
        return None
    return [float(v) for v in value[:_AXES]]


def _six_floats(value: list[float], fallback: float) -> list[float]:
    """Normalize a per-joint numeric setting to six positive values."""

    values = [abs(float(v)) for v in list(value)[:_AXES]]
    while len(values) < _AXES:
        values.append(float(fallback))
    return [v if v > 1e-9 else float(fallback) for v in values]
