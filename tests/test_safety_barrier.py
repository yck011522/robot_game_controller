"""Focused tests for safety barrier channel ordering, bypass, and guards."""

from __future__ import annotations

import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import load as load_profile  # noqa: E402
from subsystems.safety_barrier.common import apply_bypass, resolve_safety_barrier_config  # noqa: E402
from subsystems.safety_barrier.sim import SimOpenSafetyBarrier  # noqa: E402
from apps.game_controller import __main__ as game_controller  # noqa: E402
from apps.robot_io import __main__ as robot_io  # noqa: E402


LABELS = ["SBarr11", "SBarr12", "SBarr21", "SBarr22", "SBarr31", "SBarr32", "SBarr41", "SBarr42"]


def test_bypass_keeps_final_ok_true_while_raw_channel_is_false() -> None:
    """A bypassed raw-false channel must not pull down the final ok flag."""

    config = resolve_safety_barrier_config(
        channel_order=LABELS,
        bypass_channels={"SBarr11": True},
    )
    snapshot = apply_bypass([False, True, True, True, True, True, True, True], config)
    assert snapshot.channels[0] is False
    assert snapshot.effective_channels[0] is True
    assert snapshot.ok is True


def test_sim_open_reports_all_channels_ok() -> None:
    """The initial simulator reports a clean all-open safety barrier."""

    config = resolve_safety_barrier_config(channel_order=LABELS, bypass_channels={})
    snapshot = SimOpenSafetyBarrier(config).read()
    assert snapshot.ok is True
    assert snapshot.channels == [True] * 8
    assert snapshot.labels == LABELS


def test_profiles_carry_default_bypass_map() -> None:
    """Every run profile should expose all fixed barrier labels for bypass edits."""

    for profile_path in sorted((REPO_ROOT / "config" / "profiles").glob("*.yaml")):
        profile = load_profile(profile_path)
        bypass = profile.tuning.get("safety_barrier", {}).get("bypass_channels", {})
        assert set(bypass) == set(LABELS), profile_path.name
        assert all(value is False for value in bypass.values()), profile_path.name


def test_game_controller_latches_barrier_pause_until_resume() -> None:
    """GC blocks immediately on barrier false, then requires PLAY/RESUME after restore."""

    control_state = {"soft_pause": False, "safety_blocked": False, "safety_pause_latched": False}
    safety_state = game_controller._initial_safety_state(enabled=True)
    game_controller._update_safety_state(
        safety_state,
        {
            "ok": False,
            "channels": [False] + [True] * 7,
            "effective_channels": [False] + [True] * 7,
            "channel_labels": LABELS,
            "bypass_channels": {},
            "errors": [],
        },
    )
    game_controller._refresh_safety_block(control_state, safety_state, telem_age_max_s=1.0)
    assert control_state["soft_pause"] is True
    assert control_state["safety_blocked"] is True
    assert control_state["safety_pause_latched"] is True

    ok, error, _ = game_controller._apply_ui_game_control(
        control_state,
        {"stage": "play"},
        {},
        {"action": "play_resume"},
        time.perf_counter_ns(),
    )
    assert ok is False
    assert "safety barrier" in str(error)

    game_controller._update_safety_state(
        safety_state,
        {
            "ok": True,
            "channels": [True] * 8,
            "effective_channels": [True] * 8,
            "channel_labels": LABELS,
            "bypass_channels": {},
            "errors": [],
        },
    )
    game_controller._refresh_safety_block(control_state, safety_state, telem_age_max_s=1.0)
    assert control_state["safety_blocked"] is False
    assert control_state["soft_pause"] is True

    ok, error, _ = game_controller._apply_ui_game_control(
        control_state,
        {"stage": "play"},
        {},
        {"action": "play_resume"},
        time.perf_counter_ns(),
    )
    assert ok is True
    assert error is None
    assert control_state["soft_pause"] is False
    assert control_state["safety_pause_latched"] is False


def test_state_full_barrier_keeps_only_runtime_status() -> None:
    """state.full should omit static labels and profile bypass settings."""

    safety_state = game_controller._initial_safety_state(enabled=True)
    game_controller._update_safety_state(
        safety_state,
        {
            "ok": True,
            "channels": [True] * 8,
            "effective_channels": [True] * 8,
            "channel_labels": LABELS,
            "bypass_channels": {"SBarr11": False},
            "errors": [],
        },
    )

    payload = game_controller._state_full_safety_barrier(safety_state)
    assert payload == {
        "enabled": True,
        "ok": True,
        "channels": [True] * 8,
        "stale": False,
        "errors": [],
    }


def test_robot_io_safety_guard_fails_closed_on_missing_or_stale_state() -> None:
    """RobotIO should not move when enabled safety state is absent or stale."""

    state = {"ok": None, "last_state_recv_mono_s": None}
    assert robot_io._safety_allows_motion(state, enabled=True, stale_after_s=1.0) is False

    robot_io._update_barrier_state(state, {"safety": {"barrier": {"ok": True}}})
    assert robot_io._safety_allows_motion(state, enabled=True, stale_after_s=1.0) is True

    state["last_state_recv_mono_s"] = time.perf_counter() - 2.0
    assert robot_io._safety_allows_motion(state, enabled=True, stale_after_s=1.0) is False
    assert robot_io._safety_allows_motion(state, enabled=False, stale_after_s=1.0) is True


def main() -> int:
    test_bypass_keeps_final_ok_true_while_raw_channel_is_false()
    test_sim_open_reports_all_channels_ok()
    test_profiles_carry_default_bypass_map()
    test_game_controller_latches_barrier_pause_until_resume()
    test_state_full_barrier_keeps_only_runtime_status()
    test_robot_io_safety_guard_fails_closed_on_missing_or_stale_state()
    print("[test] safety barrier tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
