"""Velocity-based joint-space retiming, sampling, and single-segment motion.

This module holds the pure (hardware-free) geometry/timing math shared by two
callers so the formula lives in exactly one place:

* :class:`subsystems.rewind.in_process.RewindController` retimes and samples a
  multi-waypoint rewind path during the reset stage.
* :class:`SegmentMover` (below) retimes and samples a single straight
  start->goal segment during the conclusion show choreography.

Both use the same idea: the duration of moving between two configurations is
the time the *slowest* joint needs at its retimed speed limit
(``speed_fraction * max_velocity``). Every joint then interpolates linearly
across that duration, so motion is a straight line in joint space.

Nothing here performs I/O. Callers own all bus traffic and clocks; they feed in
``dt`` (seconds advanced this tick) and measured/target configurations as plain
``list[float]`` in radians, bus joint order, six joints.
"""

from __future__ import annotations

import bisect

# Number of robot joints. The UR10e is a six-axis arm; every configuration and
# velocity-limit list is expected to carry at least this many entries.
AXES = 6


def retime_path(
    path_rad: list[list[float]],
    max_velocity_rad_s: list[float],
    speed_fraction: float,
) -> list[float]:
    """Return cumulative arrival times (seconds) for each waypoint in a path.

    The time to traverse one segment is ``max_axis(|Δq_axis| / limit_axis)``
    where ``limit_axis = max(1e-9, max_velocity_axis * speed_fraction)``. The
    returned list is the running sum of those segment times, so element ``i`` is
    the time at which waypoint ``i`` is reached (element ``0`` is always 0.0).

    Args:
        path_rad: Ordered waypoints, each a six-joint configuration in radians.
        max_velocity_rad_s: Per-joint configured maximum velocities (rad/s).
            Tune via the robot velocity-limit config; larger values shorten the
            move. Values are used in magnitude (sign ignored).
        speed_fraction: Fraction (0, 1] of each maximum actually used. This is
            the single knob that scales the whole move speed; e.g. 0.60 means
            "move at 60% of the per-axis maximum velocity". Clamped to a tiny
            positive floor so a zero never divides.

    Returns:
        ``len(path_rad)`` cumulative arrival times in seconds, starting at 0.0.
        An empty path yields an empty list; a single waypoint yields ``[0.0]``.
    """

    fraction = max(1e-9, float(speed_fraction))  # never zero -> never divide by 0
    limits = [
        max(1e-9, abs(float(velocity)) * fraction)
        for velocity in list(max_velocity_rad_s)[:AXES]
    ]
    times: list[float] = []
    elapsed_s = 0.0
    previous_q: list[float] | None = None
    for q in path_rad:
        if previous_q is not None:
            elapsed_s += max(
                abs(float(q[axis]) - float(previous_q[axis])) / limits[axis]
                for axis in range(AXES)
            )
        times.append(elapsed_s)
        previous_q = list(q)
    return times


def sample_path_with_index(
    path_rad: list[list[float]],
    times_s: list[float],
    elapsed_s: float,
) -> tuple[list[float], int]:
    """Linearly interpolate a retimed path at one time, returning the segment.

    This is the shared core of the rewind controller's ``_sample``: it both
    returns the interpolated six-joint configuration and the index of the lower
    waypoint of the active segment (useful for progress bookkeeping).

    Args:
        path_rad: Ordered waypoints (see :func:`retime_path`).
        times_s: Cumulative arrival times from :func:`retime_path`, same length.
        elapsed_s: Seconds elapsed along the path. Clamped to ``[0, times[-1]]``.

    Returns:
        ``(configuration, lower_index)`` where ``configuration`` is the
        interpolated six-joint position and ``lower_index`` is the index of the
        waypoint at or before ``elapsed_s``.
    """

    if elapsed_s <= 0.0 or len(path_rad) == 1:
        return [float(v) for v in path_rad[0][:AXES]], 0
    if elapsed_s >= times_s[-1]:
        return [float(v) for v in path_rad[-1][:AXES]], len(path_rad) - 1

    upper = bisect.bisect_right(times_s, elapsed_s)
    lower = max(0, upper - 1)
    t0 = times_s[lower]
    t1 = times_s[upper]
    q0 = path_rad[lower]
    q1 = path_rad[upper]
    if t1 <= t0:
        return [float(v) for v in q1[:AXES]], lower
    alpha = (elapsed_s - t0) / (t1 - t0)
    return (
        [float(q0[axis]) + alpha * (float(q1[axis]) - float(q0[axis])) for axis in range(AXES)],
        lower,
    )


def sample_path(
    path_rad: list[list[float]],
    times_s: list[float],
    elapsed_s: float,
) -> list[float]:
    """Interpolate a retimed path at one time (configuration only).

    Thin wrapper over :func:`sample_path_with_index` for callers that do not
    need the segment index. See that function for argument semantics.
    """

    return sample_path_with_index(path_rad, times_s, elapsed_s)[0]


class SegmentMover:
    """Drive one straight joint-space segment from a start to a goal pose.

    The conclusion show moves the robot between a handful of fixed poses one
    segment at a time. ``SegmentMover`` owns the timing for a single such
    segment: :meth:`begin` retimes ``start -> goal`` using the same formula as
    the rewind controller, and :meth:`advance` is called once per game tick with
    the seconds elapsed to return the next interpolated target.

    Motion is open-loop in time: the segment is considered :attr:`arrived` once
    the accumulated elapsed time reaches the retimed duration. Because the
    caller advances every active mover with the *same* per-tick ``dt`` (and
    skips advancing while the game is paused), two teams sharing one game loop
    stay phase-aligned and freeze together during a pause.

    Seed the segment with the *measured* current configuration as ``start`` so
    the first interpolated target equals where the robot already is (no jump).
    """

    def __init__(
        self,
        *,
        max_velocity_rad_s: list[float],
        speed_fraction: float,
    ) -> None:
        """Create an idle mover bound to one set of speed limits.

        Args:
            max_velocity_rad_s: Per-joint configured maximum velocities (rad/s).
            speed_fraction: Fraction (0, 1] of those maxima to move at. This is
                the conclusion ``conclusion_speed_fraction`` knob; raise it for
                a snappier show, lower it for a calmer one.
        """

        # Per-joint velocity ceiling used to retime each segment.
        self.max_velocity_rad_s = [float(v) for v in list(max_velocity_rad_s)[:AXES]]
        # Fraction of those ceilings the mover actually targets.
        self.speed_fraction = float(speed_fraction)
        # Two-waypoint [start, goal] path for the active segment (empty = idle).
        self._path: list[list[float]] = []
        # Cumulative arrival times for ``_path`` (``[0.0, duration]``).
        self._times: list[float] = [0.0]
        # Seconds advanced into the active segment so far.
        self._elapsed_s = 0.0

    def begin(self, start_rad: list[float], goal_rad: list[float]) -> None:
        """Arm a new straight segment from ``start_rad`` to ``goal_rad``.

        Resets the elapsed clock to zero and retimes the segment. A zero-length
        segment (start == goal) is immediately :attr:`arrived`.

        Args:
            start_rad: Six-joint start configuration in radians. Pass the
                measured current pose to avoid a step on the first tick.
            goal_rad: Six-joint goal configuration in radians.
        """

        self._path = [
            [float(v) for v in list(start_rad)[:AXES]],
            [float(v) for v in list(goal_rad)[:AXES]],
        ]
        self._times = retime_path(self._path, self.max_velocity_rad_s, self.speed_fraction)
        self._elapsed_s = 0.0

    def advance(self, dt_s: float) -> list[float]:
        """Advance the segment by ``dt_s`` seconds and return the next target.

        Args:
            dt_s: Seconds elapsed since the previous call. Negative or zero
                values do not move the segment forward.

        Returns:
            The interpolated six-joint target configuration in radians. When the
            mover is idle (never begun) the goal/last waypoint is returned, or an
            empty list if it has never been armed.
        """

        if not self._path:
            return []
        self._elapsed_s = min(self._times[-1], self._elapsed_s + max(0.0, float(dt_s)))
        return sample_path(self._path, self._times, self._elapsed_s)

    @property
    def goal_rad(self) -> list[float]:
        """The active segment's goal configuration, or an empty list when idle."""

        return list(self._path[-1]) if self._path else []

    @property
    def duration_s(self) -> float:
        """Total retimed duration of the active segment in seconds."""

        return float(self._times[-1]) if self._times else 0.0

    @property
    def remaining_s(self) -> float:
        """Seconds left before the active segment reaches its goal."""

        return max(0.0, self.duration_s - self._elapsed_s)

    @property
    def arrived(self) -> bool:
        """True once the elapsed clock has reached the segment duration."""

        return bool(self._path) and self._elapsed_s >= self._times[-1]
