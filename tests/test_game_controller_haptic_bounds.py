from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import __main__ as gc  # noqa: E402


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
