from __future__ import annotations

import json
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import __main__ as gc  # noqa: E402
from apps.game_controller import haptics as gc_haptics  # noqa: E402

gc._haptic_config = gc_haptics._haptic_config
gc._publish_hold_current_pose = gc_haptics._publish_hold_current_pose
gc._update_dynamic_haptic_bounds_from_prox = (
    gc_haptics._update_dynamic_haptic_bounds_from_prox
)


class _FakePub:
    def __init__(self) -> None:
        self.frames = []

    def send_multipart(self, frames) -> None:
        self.frames.append(frames)


def _base_haptic_cfg() -> dict:
    return {
        "gear_ratio": [2.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "bounds_min_rad": [-math.pi] * 6,
        "bounds_max_rad": [math.pi] * 6,
        "prox_bounds_stale_ticks": 12,
    }


def test_dynamic_bounds_use_nearest_hits_per_direction() -> None:
    cfg = _base_haptic_cfg()
    state = {
        "last_q": [0.0] * 6,
        "last_prox_probe_offsets_deg": [-10.0, -5.0, -1.0, 1.0, 5.0, 10.0],
        "last_prox_hits": [
            [False, True, True, False, True, False],
            [False] * 6,
            [False] * 6,
            [False] * 6,
            [False] * 6,
            [False] * 6,
        ],
        "last_prox_age_ticks": [0, 99, 99, 99, 99, 99],
    }

    gc._update_dynamic_haptic_bounds_from_prox(state, cfg)

    # Axis-0 gear=2.0, so +/-1deg and +5deg robot offsets become
    # -0.5deg and +2.5deg in dial space.
    expect_min = math.radians(-0.5)
    expect_max = math.radians(2.5)
    got_min = state["current_haptic_bounds_min_rad"][0]
    got_max = state["current_haptic_bounds_max_rad"][0]
    assert math.isclose(got_min, expect_min, rel_tol=0.0, abs_tol=1e-9)
    assert math.isclose(got_max, expect_max, rel_tol=0.0, abs_tol=1e-9)

    # Other axes are stale -> static fallback.
    for axis in range(1, 6):
        assert math.isclose(
            state["current_haptic_bounds_min_rad"][axis],
            -math.pi,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        assert math.isclose(
            state["current_haptic_bounds_max_rad"][axis],
            math.pi,
            rel_tol=0.0,
            abs_tol=1e-12,
        )


def test_profile_static_bounds_convert_from_robot_to_dial_space() -> None:
    cfg = gc._haptic_config(
        {
            "gear_ratio": [0.1] * 6,
            "bounds_deg_min": [-180.0] * 6,
            "bounds_deg_max": [180.0] * 6,
        }
    )

    assert math.isclose(
        cfg["bounds_min_rad"][0],
        math.radians(-1800.0),
        rel_tol=0.0,
        abs_tol=1e-9,
    )
    assert math.isclose(
        cfg["bounds_max_rad"][0],
        math.radians(1800.0),
        rel_tol=0.0,
        abs_tol=1e-9,
    )


def test_hold_current_pose_publishes_measured_robot_target() -> None:
    pub = _FakePub()
    state = {
        "last_q": [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
    }

    gc._publish_hold_current_pose(pub, "test_game_controller", "b", state)

    assert state["last_target"] == state["last_q"]
    assert state["last_path_scalar"] == 1.0
    assert len(pub.frames) == 1
    topic = pub.frames[0][0].decode("ascii")
    body = json.loads(pub.frames[0][1].decode("utf-8"))
    assert topic == "cmd.robot.target.b"
    assert body["q_target_rad"] == state["last_q"]
    assert body["clamps"] == {"path": 1.0, "prox": 1.0, "final": 1.0}


def test_state_full_planner_diagnostics_are_compact() -> None:
    info = {
        "input_mode": "absolute",
        "forward_certified": False,
        "v_cmd_rad_s": [1, 2, 3, 4, 5, 6],
        "v_out_rad_s": [0, 0, 0, 0, 0, 0],
        "prox_hits": [[True]],
    }

    out = gc._state_full_planner(info)

    assert out == {
        "input_mode": "absolute",
        "forward_certified": False,
        "v_cmd_rad_s": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        "v_out_rad_s": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }


def test_dynamic_bounds_fallback_when_axis_stale() -> None:
    cfg = _base_haptic_cfg()
    state = {
        "last_q": [0.0] * 6,
        "last_prox_probe_offsets_deg": [-3.0, -1.0, 1.0, 3.0],
        "last_prox_hits": [[True, True, True, True] for _ in range(6)],
        "last_prox_age_ticks": [13, 13, 13, 13, 13, 13],
    }

    gc._update_dynamic_haptic_bounds_from_prox(state, cfg)

    assert state["current_haptic_bounds_min_rad"] == [-math.pi] * 6
    assert state["current_haptic_bounds_max_rad"] == [math.pi] * 6


def test_dynamic_bounds_handle_negative_gear_ratio() -> None:
    cfg = _base_haptic_cfg()
    cfg["gear_ratio"][0] = -2.0
    state = {
        "last_q": [0.0] * 6,
        "last_prox_probe_offsets_deg": [-5.0, -1.0, 1.0, 5.0],
        "last_prox_hits": [
            [False, True, True, False],
            [False] * 4,
            [False] * 4,
            [False] * 4,
            [False] * 4,
            [False] * 4,
        ],
        "last_prox_age_ticks": [0, 99, 99, 99, 99, 99],
    }

    gc._update_dynamic_haptic_bounds_from_prox(state, cfg)

    # Negative gear flips sign; implementation must still output ordered bounds.
    lo = state["current_haptic_bounds_min_rad"][0]
    hi = state["current_haptic_bounds_max_rad"][0]
    assert lo <= hi
    assert lo >= -math.pi
    assert hi <= math.pi
