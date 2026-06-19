"""Tests for repeatable planner seeding and play-entry haptic alignment.

Run:
    $env:PYTHONPATH = "src"
    C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe -m pytest tests\\test_jogging_reseed_and_play_sync.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import haptics  # noqa: E402
from subsystems.jogging.in_process import InProcessPlanner  # noqa: E402


def test_reseed_replaces_stale_pose_velocity_and_dial_history() -> None:
    """A new game must not retain the prior game's planner integrator state."""

    planner = InProcessPlanner.__new__(InProcessPlanner)
    planner._q_cur = [2.0] * 6
    planner._v_cur = [1.0] * 6
    planner._prev_dial_pos = [3.0] * 6
    planner._seeded = True

    q_actual = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    q_dial = [1.0, -2.0, 3.0, -4.0, 5.0, -6.0]
    assert planner.reseed(q_actual, dial_pos_rad=q_dial) is True

    assert planner.q_cur == q_actual
    assert planner._v_cur == [0.0] * 6
    assert planner._prev_dial_pos == q_dial
    assert planner._seeded is True


def test_play_sync_publishes_reseat_and_waits_for_telemetry_streak() -> None:
    """Jogging remains gated until dial telemetry proves reseat was applied."""

    state = {
        "last_q": [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        "last_dial": [0.0] * 6,
        "last_haptic_connected": [True] * 6,
        "play_sync": {
            "enabled": True,
            "requested": True,
            "pending": False,
            "attempts": 0,
        },
    }
    config = {
        "gear_ratio": [0.1] * 6,
        "startup_settle_tol_rad": math.radians(0.5),
        "startup_settle_streak_ticks": 2,
        "startup_reseat_timeout_s": 1.0,
    }
    published: list[tuple[str, dict]] = []
    original_publish = haptics.bus.publish
    haptics.bus.publish = lambda _pub, topic, body: published.append((topic, body))
    try:
        assert haptics._begin_play_sync(
            object(), "test", "a", state, config, now=10.0
        ) is True
        assert published[-1][0] == "cmd.haptic.reseat.a"
        target_dial = list(state["play_sync"]["target_dial_rad"])
        assert target_dial == [value / 0.1 for value in state["last_q"]]

        state["last_dial"] = target_dial
        assert haptics._tick_play_sync(
            object(), "test", "a", state, config, now=10.1
        ) is False
        assert haptics._tick_play_sync(
            object(), "test", "a", state, config, now=10.2
        ) is True
        assert state["play_sync"]["pending"] is False
    finally:
        haptics.bus.publish = original_publish


def test_play_sync_is_disabled_for_non_real_haptic_profiles() -> None:
    """Keyboard and scripted profiles must not wait for firmware acknowledgements."""

    state = {
        "last_q": [0.0] * 6,
        "play_sync": {"enabled": False, "requested": True, "pending": False},
    }
    assert haptics._begin_play_sync(
        object(), "test", "a", state, {"gear_ratio": [1.0] * 6}, now=0.0
    ) is False
    assert state["play_sync"]["requested"] is False
