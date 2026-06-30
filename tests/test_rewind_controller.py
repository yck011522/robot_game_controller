"""Focused tests for geometric play-trajectory recording and rewind.

Run:
    $env:PYTHONPATH = "src"
    C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe -m pytest tests\\test_rewind_controller.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

from compas_fab.robots import JointTrajectory

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller.context import _game_config, _tutorial_config  # noqa: E402
from apps.game_controller.stages import _tick_stage_state  # noqa: E402
from subsystems.rewind.in_process import RewindController  # noqa: E402


class _RewindCompletionStub:
    """Minimal reset-stage completion signal used by the pure stage test."""

    def __init__(self, complete: bool) -> None:
        self.complete = complete


def _controller(
    *,
    max_velocity_rad_s: list[float] | None = None,
    speed_fraction: float = 0.3,
    tolerance_deg: float = 0.5,
) -> RewindController:
    """Build an enabled six-axis controller with compact test tuning."""

    return RewindController(
        enabled=True,
        max_velocity_rad_s=max_velocity_rad_s or [1.0] * 6,
        speed_fraction=speed_fraction,
        arrival_tolerance_rad=math.radians(tolerance_deg),
    )


def test_records_native_compas_trajectory_with_gameplay_timestamps() -> None:
    """Recording keeps native points and gameplay-relative time metadata."""

    rewind = _controller()
    rewind.start_recording([0.0] * 6, now_s=10.0)
    rewind.record_target([0.1] * 6, now_s=12.5)

    assert isinstance(rewind.recorded_trajectory, JointTrajectory)
    assert len(rewind.recorded_trajectory.points) == 2
    assert rewind.recorded_trajectory.points[0].time_from_start.seconds == 0.0
    assert rewind.recorded_trajectory.points[1].time_from_start.seconds == 2.5


def test_rewind_reverses_geometry_and_ignores_recorded_timing() -> None:
    """Retiming uses path displacement and velocity limits, not gameplay time."""

    rewind = _controller(max_velocity_rad_s=[1.0] * 6, speed_fraction=0.3)
    rewind.start_recording([0.0] * 6, now_s=0.0)
    rewind.record_target([0.6, 0.0, 0.0, 0.0, 0.0, 0.0], now_s=100.0)

    assert rewind.start_rewind() is True
    points = rewind.rewind_trajectory.points
    assert list(points[0].joint_values) == [0.6, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert list(points[-1].joint_values) == [0.0] * 6
    # 0.6 rad / (1.0 rad/s * 0.3) = 2 seconds, independent of 100 s gameplay.
    assert math.isclose(points[-1].time_from_start.seconds, 2.0)

    # First call confirms measured arrival at the final gameplay target.
    rewind.next_target(dt_s=0.0, q_actual_rad=[0.6] + [0.0] * 5)
    halfway = rewind.next_target(dt_s=1.0, q_actual_rad=[0.6] + [0.0] * 5)
    assert halfway is not None
    assert math.isclose(halfway[0], 0.3, abs_tol=1e-9)


def test_slowest_joint_sets_synchronized_segment_duration() -> None:
    """All axes share timing determined by the most constrained moving axis."""

    rewind = _controller(
        max_velocity_rad_s=[1.0, 2.0, 1.0, 1.0, 1.0, 1.0],
        speed_fraction=0.5,
    )
    rewind.start_recording([0.0] * 6, now_s=0.0)
    rewind.record_target([0.5, 2.0, 0.0, 0.0, 0.0, 0.0], now_s=0.1)
    rewind.start_rewind()

    # J1 needs 1 s and J2 needs 2 s, so both interpolate over 2 s.
    assert math.isclose(
        rewind.rewind_trajectory.points[-1].time_from_start.seconds, 2.0
    )


def test_completion_waits_for_measured_half_degree_arrival() -> None:
    """Finishing generated timing alone cannot advance the reset stage."""

    rewind = _controller(speed_fraction=0.3, tolerance_deg=0.5)
    rewind.start_recording([0.0] * 6, now_s=0.0)
    rewind.record_target([0.3] + [0.0] * 5, now_s=1.0)
    rewind.start_rewind()

    rewind.next_target(dt_s=0.0, q_actual_rad=[0.3] + [0.0] * 5)
    rewind.next_target(dt_s=10.0, q_actual_rad=[math.radians(0.6)] + [0.0] * 5)
    assert rewind.complete is False
    assert rewind.snapshot()["status"] == "settling"

    rewind.next_target(dt_s=0.01, q_actual_rad=[math.radians(0.4)] + [0.0] * 5)
    assert rewind.complete is True
    assert rewind.snapshot()["max_error_deg"] <= 0.5


def test_rewind_config_defaults_disabled_and_parses_test_values() -> None:
    """Existing profiles retain timer reset unless they explicitly opt in."""

    assert _game_config({})["rewind_enabled"] is False
    config = _game_config(
        {
            "rewind_enabled": True,
            "rewind_speed_fraction": 0.3,
            "rewind_arrival_tolerance_deg": 0.5,
        }
    )
    assert config["rewind_enabled"] is True
    assert config["rewind_speed_fraction"] == 0.3
    assert config["rewind_arrival_tolerance_deg"] == 0.5


def test_enabled_reset_ignores_timer_and_waits_for_rewind_completion() -> None:
    """An enabled rewind cannot enter conclusion based on reset time alone."""

    config = _game_config(
        {"rewind_enabled": True, "reset_duration_s": 1.0}
    )
    completion = _RewindCompletionStub(False)
    team = {
        "rewind": completion,
        "bucket_values": [0, 0, 0],
        "score": 0,
        "summed_score": 0,
        "conclusion_phase": None,
        "conclusion_active_bucket_index": None,
        "conclusion_target_pose_name": None,
        "conclusion_target_pose_deg": None,
        "conclusion_bucket_open_triggered": False,
        "conclusion_phase_started_mono_ns": None,
        "conclusion_done": False,
        "conclusion_sum_remainder_units": 0.0,
    }
    stage_state = {
        "stage": "reset",
        "stage_entered_mono_ns": 0,
        "pause_started_mono_ns": None,
        "paused_total_ns": 0,
        "winner_team": None,
        "skip_requested": False,
    }

    _tick_stage_state(stage_state, {"a": team}, config, _tutorial_config({}), int(10e9))
    assert stage_state["stage"] == "reset"

    completion.complete = True
    _tick_stage_state(stage_state, {"a": team}, config, _tutorial_config({}), int(11e9))
    assert stage_state["stage"] == "conclusion"
