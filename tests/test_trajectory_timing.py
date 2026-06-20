"""Unit tests for the shared trajectory timing helpers and SegmentMover.

Run (pytest is not installed in the env):
    python tests/test_trajectory_timing.py
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from subsystems.motion_planning.trajectory_timing import (  # noqa: E402
    SegmentMover,
    retime_path,
    sample_path,
    sample_path_with_index,
)

_VEL = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


def test_retime_duration_is_slowest_axis() -> None:
    """Segment duration = max(|Δq|/limit); limit = vel * speed_fraction."""

    path = [[0.0] * 6, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    times = retime_path(path, _VEL, 0.5)  # limit per axis = 0.5 rad/s
    assert times[0] == 0.0
    assert abs(times[1] - 2.0) < 1e-9  # 1.0 rad / 0.5 rad/s


def test_retime_multi_axis_uses_largest_move() -> None:
    """The slowest (largest) joint move sets the synchronized duration."""

    path = [[0.0] * 6, [0.5, 2.0, 0.0, 0.0, 0.0, 0.0]]
    times = retime_path(path, _VEL, 1.0)  # limit = 1.0 rad/s
    assert abs(times[1] - 2.0) < 1e-9  # driven by joint 1 (2.0 rad)


def test_sample_interpolates_and_clamps() -> None:
    """sample_path returns endpoints outside [0, duration] and lerps inside."""

    path = [[0.0] * 6, [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    times = retime_path(path, _VEL, 1.0)  # duration 2.0 s
    assert sample_path(path, times, -1.0)[0] == 0.0
    assert abs(sample_path(path, times, 1.0)[0] - 1.0) < 1e-9
    assert sample_path(path, times, 5.0)[0] == 2.0


def test_sample_with_index_reports_segment() -> None:
    """sample_path_with_index returns the lower waypoint index of the segment."""

    path = [[0.0] * 6, [1.0] + [0.0] * 5, [2.0] + [0.0] * 5]
    times = retime_path(path, _VEL, 1.0)  # [0, 1, 2]
    _, index = sample_path_with_index(path, times, 1.5)
    assert index == 1


def test_segment_mover_lifecycle() -> None:
    """begin -> advance interpolates, arrives exactly at the retimed duration."""

    mover = SegmentMover(max_velocity_rad_s=_VEL, speed_fraction=0.5)
    mover.begin([0.0] * 6, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert abs(mover.duration_s - 2.0) < 1e-9
    assert mover.arrived is False

    q = mover.advance(1.0)  # halfway
    assert abs(q[0] - 0.5) < 1e-9
    assert mover.arrived is False
    assert abs(mover.remaining_s - 1.0) < 1e-9

    q = mover.advance(1.0)  # reaches goal
    assert abs(q[0] - 1.0) < 1e-9
    assert mover.arrived is True
    assert mover.remaining_s == 0.0


def test_segment_mover_zero_length_is_immediately_arrived() -> None:
    """A start == goal segment has zero duration and arrives at once."""

    mover = SegmentMover(max_velocity_rad_s=_VEL, speed_fraction=0.6)
    mover.begin([0.1] * 6, [0.1] * 6)
    assert mover.duration_s == 0.0
    assert mover.arrived is True


def main() -> int:
    test_retime_duration_is_slowest_axis()
    test_retime_multi_axis_uses_largest_move()
    test_sample_interpolates_and_clamps()
    test_sample_with_index_reports_segment()
    test_segment_mover_lifecycle()
    test_segment_mover_zero_length_is_immediately_arrived()
    print("[test] trajectory timing tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
