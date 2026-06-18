"""Tests for the random trajectory validation haptic source."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import load as load_profile  # noqa: E402
from subsystems.haptic.random_trajectory import RandomTrajectoryHaptic  # noqa: E402


class _Clock:
    """Deterministic monotonic clock for integration tests."""

    def __init__(self) -> None:
        self.now_s = 0.0

    def advance(self, dt_s: float) -> None:
        """Move test time forward by a controlled number of seconds."""

        self.now_s += float(dt_s)

    def __call__(self) -> float:
        """Return the current test time."""

        return self.now_s


def _make_profile() -> SimpleNamespace:
    """Build a small profile-shaped object for unit tests."""

    return SimpleNamespace(
        tuning={
            "haptic": {
                "gear_ratio": [0.1] * 6,
                "input_mode": "absolute",
            },
            "robot": {
                "q_limits_min_deg": [-180.0] * 6,
                "q_limits_max_deg": [180.0] * 6,
                "max_velocity_deg_s": [15.0, 20.0, 30.0, 40.0, 50.0, 50.0],
                "max_acceleration_deg_s2": [45.0, 65.0, 65.0, 115.0, 115.0, 115.0],
            },
            "jogging": {
                "path_cutoff_deg": 3.0,
            },
            "random_trajectory_validation": {
                "seed": 99,
                "enabled_on_start": False,
                "ui_enabled": False,
                "speed_scale": 1.0,
                "min_axis_speed_fraction": 0.2,
                "path_turnaround_distance_deg": 3.0,
                "proximity_flip_distance_deg": 3.0,
                "proximity_stale_ticks": 12,
            },
        }
    )


def _state_with_proximity(
    *,
    offsets_deg: list[float],
    hits: list[list[bool]],
    ages: list[int],
) -> dict:
    """Build a minimal state.full payload carrying proximity masks."""

    return {
        "teams": {
            "b": {
                "collision": {
                    "prox_probe_offsets_deg": offsets_deg,
                    "prox_hits": hits,
                    "prox_age_ticks": ages,
                }
            }
        }
    }


def test_paused_start_latches_robot_pose_once() -> None:
    """Paused startup should seed from robot actual without chasing updates."""

    clock = _Clock()
    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=clock)
    first_pose = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    second_pose = [0.6, 0.5, 0.4, 0.3, 0.2, 0.1]

    rig.update_robot_actual(first_pose)
    assert rig.sample() is not None
    rig.update_robot_actual(second_pose)
    assert rig.sample() is not None

    assert rig.robot_target_rad == first_pose
    assert rig.robot_velocity_rad_s == [0.0] * 6
    print("[test] random trajectory paused startup latch: OK")


def test_unchecking_latches_to_last_planner_target_once() -> None:
    """Disabling the checkbox should hold the latest planner servo target."""

    clock = _Clock()
    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=clock)
    planner_target = [0.2, 0.1, 0.0, -0.1, -0.2, -0.3]
    later_pose = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4]

    rig.update_robot_actual([0.0] * 6)
    rig.set_running(True)
    clock.advance(0.05)
    rig.sample()
    rig.update_state_full(
        {
            "teams": {
                "b": {
                    "robot": {
                        "q_target_rad": planner_target,
                    },
                    "collision": {
                        "first_hit": None,
                    },
                }
            }
        }
    )
    rig.update_robot_actual([0.05] * 6)
    rig.set_running(False)
    rig.sample()
    rig.update_robot_actual(later_pose)
    rig.sample()

    assert rig.robot_target_rad == planner_target
    assert rig.robot_velocity_rad_s == [0.0] * 6
    print("[test] random trajectory pause transition latch: OK")


def test_random_velocity_respects_profile_speed_limits() -> None:
    """Generated robot velocity should stay within configured per-axis max."""

    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=_Clock())
    rig.update_robot_actual([0.0] * 6)
    rig.set_running(True)
    limits = [15.0, 20.0, 30.0, 40.0, 50.0, 50.0]

    for velocity, limit_deg_s in zip(rig.robot_velocity_rad_s, limits):
        assert abs(velocity) <= math.radians(limit_deg_s) + 1e-12
        assert abs(velocity) >= math.radians(limit_deg_s * 0.2) - 1e-12
    print("[test] random trajectory velocity limits: OK")


def test_run_request_waits_for_robot_actual_before_motion() -> None:
    """Checking the UI before robot feedback should not publish motion yet."""

    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=_Clock())

    rig.set_running(True)
    assert rig.sample() is None
    assert rig.robot_target_rad == [0.0] * 6
    assert rig.robot_velocity_rad_s == [0.0] * 6

    actual_pose = [0.2, -0.1, 0.3, -0.2, 0.4, -0.3]
    rig.update_robot_actual(actual_pose)

    assert rig.robot_target_rad == actual_pose
    assert rig.sample() is not None
    assert any(abs(v) > 0.0 for v in rig.robot_velocity_rad_s)
    print("[test] random trajectory run waits for robot actual: OK")


def test_unchecking_before_robot_actual_cancels_pending_run() -> None:
    """A pre-seed run request should be cancelable before first robot pose."""

    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=_Clock())

    rig.set_running(True)
    rig.set_running(False)
    rig.update_robot_actual([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    sample = rig.sample()

    assert sample is not None
    assert rig.robot_velocity_rad_s == [0.0] * 6
    assert sample["validation"]["running"] is False
    print("[test] random trajectory pending run cancel: OK")


def test_path_collision_distance_randomizes_vector() -> None:
    """Forward-path first hit at the threshold should immediately reroll."""

    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=_Clock())
    rig.update_robot_actual([0.0] * 6)
    rig.set_running(True)
    before = rig.robot_velocity_rad_s

    rig.update_state_full(
        {
            "teams": {
                "b": {
                    "collision": {
                        "first_hit": {
                            "distance_deg": 3.0,
                        }
                    }
                }
            }
        }
    )

    assert rig.robot_velocity_rad_s != before
    print("[test] random trajectory path collision reroll: OK")


def test_path_collision_reroll_resets_to_planner_target() -> None:
    """Rerolling should discard stale blocked target offset."""

    clock = _Clock()
    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=clock)
    planner_target = [0.2, -0.1, 0.3, -0.2, 0.4, -0.3]

    rig.update_robot_actual([0.0] * 6)
    rig.set_running(True)
    clock.advance(0.1)
    rig.sample()
    assert rig.robot_target_rad != planner_target

    rig.update_state_full(
        {
            "teams": {
                "b": {
                    "robot": {
                        "q_target_rad": planner_target,
                    },
                    "collision": {
                        "first_hit": {
                            "distance_deg": 3.0,
                        }
                    },
                }
            }
        }
    )

    assert rig.robot_target_rad == planner_target
    print("[test] random trajectory collision reroll target reset: OK")


def test_proximity_bias_flips_away_from_near_positive_hit() -> None:
    """A generated positive direction should flip if positive prox is tight."""

    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=_Clock())
    rig.update_state_full(
        _state_with_proximity(
            offsets_deg=[-3.0, -1.0, 1.0, 3.0],
            hits=[
                [False, False, True, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
            ],
            ages=[0, 9999, 9999, 9999, 9999, 9999],
        )
    )

    assert rig._proximity_biased_sign(0, 1.0) == -1.0
    assert rig._proximity_biased_sign(0, -1.0) == -1.0
    print("[test] random trajectory proximity flips away from positive hit: OK")


def test_proximity_bias_ignores_stale_masks() -> None:
    """Old proximity samples should not steer newly generated velocity."""

    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=_Clock())
    rig.update_state_full(
        _state_with_proximity(
            offsets_deg=[-3.0, -1.0, 1.0, 3.0],
            hits=[
                [False, False, True, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
            ],
            ages=[13, 9999, 9999, 9999, 9999, 9999],
        )
    )

    assert rig._proximity_biased_sign(0, 1.0) == 1.0
    print("[test] random trajectory proximity stale mask ignored: OK")


def test_proximity_bias_keeps_sign_when_both_sides_free() -> None:
    """Free proximity masks should preserve the RNG-selected direction."""

    rig = RandomTrajectoryHaptic(team="b", profile=_make_profile(), now_fn=_Clock())
    rig.update_state_full(
        _state_with_proximity(
            offsets_deg=[-3.0, -1.0, 1.0, 3.0],
            hits=[
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
                [False, False, False, False],
            ],
            ages=[0, 0, 0, 0, 0, 0],
        )
    )

    assert rig._proximity_biased_sign(0, 1.0) == 1.0
    assert rig._proximity_biased_sign(0, -1.0) == -1.0
    print("[test] random trajectory proximity free mask preserves sign: OK")


def test_validation_profile_loads_requested_team_b_limits() -> None:
    """The launcher profile should select Team B real robot and requested limits."""

    profile = load_profile(REPO_ROOT / "config" / "profiles" / "random_trajectory_collision_validation.yaml")

    assert profile.subsystems["haptic_io"]["b"] == "random_trajectory"
    assert profile.subsystems["robot_io"]["b"] == "real_rtde"
    assert profile.tuning["haptic"]["input_mode"] == "absolute"
    assert profile.tuning["robot"]["max_velocity_deg_s"] == [15, 20, 30, 40, 50, 50]
    assert profile.tuning["robot"]["max_acceleration_deg_s2"] == [45, 65, 65, 115, 115, 115]
    assert profile.tuning["random_trajectory_validation"]["enabled_on_start"] is False
    assert profile.tuning["random_trajectory_validation"]["speed_scale"] == 1.0
    assert profile.tuning["random_trajectory_validation"]["proximity_flip_distance_deg"] > 0.0
    assert profile.tuning["random_trajectory_validation"]["proximity_stale_ticks"] == 12
    print("[test] random trajectory validation profile: OK")


def main() -> int:
    """Run this file directly without requiring pytest discovery."""

    test_paused_start_latches_robot_pose_once()
    test_unchecking_latches_to_last_planner_target_once()
    test_random_velocity_respects_profile_speed_limits()
    test_run_request_waits_for_robot_actual_before_motion()
    test_unchecking_before_robot_actual_cancels_pending_run()
    test_path_collision_distance_randomizes_vector()
    test_path_collision_reroll_resets_to_planner_target()
    test_proximity_bias_flips_away_from_near_positive_hit()
    test_proximity_bias_ignores_stale_masks()
    test_proximity_bias_keeps_sign_when_both_sides_free()
    test_validation_profile_loads_requested_team_b_limits()
    print("\n[test] RANDOM TRAJECTORY HAPTIC TESTS PASSED\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
