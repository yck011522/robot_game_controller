"""Unit tests for the game stage machine (daydreaming -> ... -> conclusion).

Exercises the pure transition logic in ``apps.game_controller.__main__`` without
any ZeroMQ bus, robots, or real hardware. Each test builds a stage_state via
``_enter_stage`` (so entry side-effects run), then drives ``_tick_stage_state``
with a synthetic ``now_ns`` to fast-forward timers, or by mutating ``last_dial``
to simulate dial movement.

Run:
    python -m pytest tests/test_game_state_machine.py -q
    # full env:
    C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe -m pytest tests/test_game_state_machine.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import __main__ as gc  # noqa: E402
from apps.game_controller import context as gc_context  # noqa: E402
from apps.game_controller import operator_inputs as gc_operator_inputs  # noqa: E402
from apps.game_controller import stages as gc_stages  # noqa: E402

# The stage machine + game-config construction moved out of __main__ into the
# stages / context modules during the P7 refactor. Re-bind the moved names onto
# the ``gc`` alias so the existing ``gc.*`` call sites below keep exercising the
# real (now relocated) functions. Operator-input handling now lives in its own
# sibling module and is rebound the same way for the focused tests below.
gc._game_config = gc_context._game_config
gc._handle_operator_input_request = gc_operator_inputs._handle_operator_input_request
gc._enter_stage = gc_stages._enter_stage
gc._tick_stage_state = gc_stages._tick_stage_state
gc._stage_countdown_s = gc_stages._stage_countdown_s



# Small, fast thresholds so tests are quick and deterministic.
# movement_window_s is tiny so a couple of ~0.1 s ticks fill a full window and
# arm detection; movement_glitch_trim=0 gives plain peak-to-peak for the basic
# transition tests (a dedicated test below exercises trim-based glitch reject).
GAME_CFG = gc._game_config(
    {
        "duration_s": 5,
        "tutorial_duration_s": 4,
        "reset_duration_s": 2,
        "idle_timeout_s": 3,
        "daydream_to_idle_dial_deg": 30,
        "idle_to_tutorial_dial_deg": 360,
        "movement_window_s": 0.15,
        "movement_glitch_trim": 0,
        "sim_bucket_values": {"a": [120, 80, 40]},
        "start_stage": "daydreaming",
    }
)


def _make_team() -> dict:
    """A per-team state with seeded haptic telemetry and no alignment.

    ``haptic_seeded`` True + no ``startup_align`` block => baselines capture
    immediately and ``_startup_alignment_active`` returns False. The bucket /
    conclusion fields mirror the runtime team dict so ``_enter_conclusion``
    (run on entering the conclusion stage) has the keys it reads.
    """
    return {
        "team": "a",
        "haptic_seeded": True,
        "last_dial": [0.0] * 6,
        "bucket_values": [120, 80, 40],
        "score": 240,
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


def _make_teams() -> dict[str, dict]:
    return {"a": _make_team()}


def _new_stage_state() -> dict:
    return {
        "stage": "(init)",
        "stage_entered_mono_ns": 0,
        "winner_team": None,
        "pause_started_mono_ns": None,
        "paused_total_ns": 0,
        "dial_window": {},
        "dial_arm": {},
        "skip_requested": False,
        "prev_paused": False,
    }


def _enter(stage: str, teams: dict, now_ns: int = 0) -> dict:
    ss = _new_stage_state()
    gc._enter_stage(ss, teams, stage, GAME_CFG, now_ns, reason="test")
    return ss


def _secs(n: float) -> int:
    return int(n * 1e9)


def _arm(ss: dict, teams: dict, start_s: float = 0.1, step_s: float = 0.1) -> float:
    """Tick with the dials still until detection arms; return next tick time (s).

    Feeds a clean (still) rolling window so the team becomes armed, mirroring a
    settled boot. Returns the timestamp (seconds) for the caller's next tick.
    """
    t = start_s
    for _ in range(8):
        gc._tick_stage_state(ss, teams, GAME_CFG, _secs(t))
        if ss["dial_arm"].get("a", {}).get("armed"):
            return round(t + step_s, 6)
        t = round(t + step_s, 6)
    raise AssertionError("movement detection never armed")


# --- daydreaming ----------------------------------------------------------


def test_daydreaming_to_idle_on_dial_movement() -> None:
    teams = _make_teams()
    ss = _enter("daydreaming", teams)

    # Collect a clean still window so detection arms; no movement -> daydreaming.
    t = _arm(ss, teams)
    assert ss["stage"] == "daydreaming"
    assert ss["dial_arm"]["a"]["armed"]  # detection armed

    # Move dial 0 past the wake threshold (30 deg) -> idle. The window now holds
    # both still (0) and moved (40) samples, so its range crosses the threshold.
    teams["a"]["last_dial"][0] = math.radians(40)
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(t))
    assert ss["stage"] == "idle"


def test_daydreaming_ignores_subthreshold_movement() -> None:
    teams = _make_teams()
    ss = _enter("daydreaming", teams)
    t = _arm(ss, teams)

    teams["a"]["last_dial"][0] = math.radians(10)  # below 30 deg
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(t))
    assert ss["stage"] == "daydreaming"


def test_daydreaming_ignores_slow_drift() -> None:
    """Slow drift around a set point rolls off the window and never wakes.

    The dial creeps a little each tick, but because detection looks at the
    peak-to-peak range *within* the rolling window (old samples expire), the
    accumulated long-run displacement never registers as a wake.
    """
    teams = _make_teams()
    ss = _enter("daydreaming", teams)
    _arm(ss, teams)
    # Creep ~2 deg per 0.1 s tick for ~1 s. Within any 0.15 s window the range
    # is only a few degrees, far under the 30 deg wake threshold.
    t = 1.0
    for k in range(1, 11):
        teams["a"]["last_dial"][0] = math.radians(2.0 * k)
        gc._tick_stage_state(ss, teams, GAME_CFG, _secs(t))
        t = round(t + 0.1, 6)
    assert ss["stage"] == "daydreaming"


def test_single_frame_glitch_does_not_wake() -> None:
    """A lone encoder glitch frame is trimmed out and does not wake the game.

    Reproduces the real-world false wake (a one-tick ~140 deg J6 spike). With
    movement_glitch_trim>0 the outlier sample is discarded from the window
    range, so a single glitch frame surrounded by still samples never crosses
    the wake threshold.
    """
    cfg = gc._game_config(
        {
            "daydream_to_idle_dial_deg": 30,
            "idle_to_tutorial_dial_deg": 360,
            "idle_timeout_s": 3,
            "movement_window_s": 0.6,
            "movement_glitch_trim": 3,
            "start_stage": "daydreaming",
        }
    )
    teams = _make_teams()
    ss = _new_stage_state()
    gc._enter_stage(ss, teams, "daydreaming", cfg, 0, reason="test")
    # Fill a still window (10 ticks @ 0.05 s spans 0.45 s -> a full 0.6 s window
    # after the next few ticks) so detection arms with clean data.
    t = 0.05
    for _ in range(20):
        gc._tick_stage_state(ss, teams, cfg, _secs(t))
        t = round(t + 0.05, 6)
        if ss["dial_arm"].get("a", {}).get("armed"):
            break
    assert ss["dial_arm"]["a"]["armed"]
    # Inject a single 140 deg glitch frame on J6, then return to still.
    teams["a"]["last_dial"][5] = math.radians(140)
    gc._tick_stage_state(ss, teams, cfg, _secs(t))
    teams["a"]["last_dial"][5] = 0.0
    gc._tick_stage_state(ss, teams, cfg, _secs(round(t + 0.05, 6)))
    assert ss["stage"] == "daydreaming"  # glitch trimmed -> no false wake


# --- idle -----------------------------------------------------------------


def test_idle_to_tutorial_on_scroll_up() -> None:
    teams = _make_teams()
    ss = _enter("idle", teams)
    t = _arm(ss, teams)

    teams["a"]["last_dial"][0] = math.radians(400)  # past 360 deg
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(t))
    assert ss["stage"] == "tutorial"


def test_idle_to_daydreaming_on_timeout() -> None:
    teams = _make_teams()
    ss = _enter("idle", teams)
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.1))  # no movement

    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(3.5))  # past idle_timeout 3s
    assert ss["stage"] == "daydreaming"


# --- tutorial -------------------------------------------------------------


def test_tutorial_to_play_on_timer() -> None:
    teams = _make_teams()
    ss = _enter("tutorial", teams)
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(1.0))
    assert ss["stage"] == "tutorial"
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(4.5))  # past 4s
    assert ss["stage"] == "play"


def test_tutorial_to_play_on_skip() -> None:
    teams = _make_teams()
    ss = _enter("tutorial", teams)
    ss["skip_requested"] = True
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.1))
    assert ss["stage"] == "play"
    assert ss["skip_requested"] is False  # cleared on stage entry


def test_tutorial_to_play_when_all_active_players_reach_97_percent() -> None:
    """One-team and two-team games advance once every active player is at 97%."""

    for team_count in (1, 2):
        teams = _make_teams()
        if team_count == 2:
            teams["b"] = _make_team()
            teams["b"]["team"] = "b"
        ss = _enter("tutorial", teams)
        for team_state in teams.values():
            team_state["tutorial_progress"] = [97.0] * 6

        gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.1))

        assert ss["stage"] == "play"


def test_tutorial_waits_when_any_active_player_is_below_97_percent() -> None:
    """A single incomplete player on either active team keeps the timer running."""

    teams = _make_teams()
    teams["b"] = _make_team()
    teams["b"]["team"] = "b"
    ss = _enter("tutorial", teams)
    teams["a"]["tutorial_progress"] = [100.0] * 6
    teams["b"]["tutorial_progress"] = [97.0] * 5 + [96.99]

    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.1))

    assert ss["stage"] == "tutorial"


# --- play -----------------------------------------------------------------


def test_play_to_reset_on_timer() -> None:
    teams = _make_teams()
    ss = _enter("play", teams)
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(1.0))
    assert ss["stage"] == "play"
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(5.5))  # past 5s
    assert ss["stage"] == "reset"


def test_play_to_reset_on_skip() -> None:
    teams = _make_teams()
    ss = _enter("play", teams)
    ss["skip_requested"] = True
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.1))
    assert ss["stage"] == "reset"


# --- reset ----------------------------------------------------------------


def test_reset_to_conclusion_on_timer() -> None:
    teams = _make_teams()
    ss = _enter("reset", teams)
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.5))
    assert ss["stage"] == "reset"
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(2.5))  # past 2s
    assert ss["stage"] == "conclusion"


def test_reset_is_not_skippable() -> None:
    teams = _make_teams()
    ss = _enter("reset", teams)
    ss["skip_requested"] = True  # skip should be ignored in reset
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.5))
    assert ss["stage"] == "reset"


# --- conclusion -----------------------------------------------------------


def test_conclusion_to_idle_when_all_done() -> None:
    teams = _make_teams()
    ss = _enter("conclusion", teams)
    # Not done yet -> stays.
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(0.5))
    assert ss["stage"] == "conclusion"
    # Mark every team finished -> idle.
    teams["a"]["conclusion_done"] = True
    gc._tick_stage_state(ss, teams, GAME_CFG, _secs(1.0))
    assert ss["stage"] == "idle"


# --- skip authorization (via the UI request handler) ----------------------


def _control_state() -> dict:
    return {"soft_pause": False, "last_action": None, "last_action_ts_mono_ns": None}


def test_skip_rejected_outside_play_or_tutorial() -> None:
    teams = _make_teams()
    ss = _enter("idle", teams)
    reply = gc._handle_operator_input_request(
        _control_state(),
        ss,
        teams,
        {"action": "skip"},
        0,
        producer="test_game_state_machine",
        recovery_timeout_s=gc.RECOVERY_TIMEOUT_S,
    )
    assert reply["ok"] is False
    assert reply["result"]["action"] == "skip"
    assert ss["skip_requested"] is False
    assert "idle" in str(reply["error"] or "")


def test_skip_accepted_in_play_and_tutorial() -> None:
    teams = _make_teams()
    for stage in ("play", "tutorial"):
        ss = _enter(stage, teams)
        reply = gc._handle_operator_input_request(
            _control_state(),
            ss,
            teams,
            {"action": "skip"},
            0,
            producer="test_game_state_machine",
            recovery_timeout_s=gc.RECOVERY_TIMEOUT_S,
        )
        assert reply["ok"] is True, stage
        assert reply["error"] is None
        assert reply["result"]["action"] == "skip"
        assert ss["skip_requested"] is True


def test_end_game_is_alias_for_skip() -> None:
    teams = _make_teams()
    ss = _enter("play", teams)
    reply = gc._handle_operator_input_request(
        _control_state(),
        ss,
        teams,
        {"action": "end_game"},
        0,
        producer="test_game_state_machine",
        recovery_timeout_s=gc.RECOVERY_TIMEOUT_S,
    )
    assert reply["ok"] is True
    assert reply["result"]["action"] == "skip"
    assert ss["skip_requested"] is True


# --- countdown reflects the active timed stage ----------------------------


def test_countdown_only_for_timed_stages() -> None:
    teams = _make_teams()
    assert gc._stage_countdown_s(_enter("play", teams), GAME_CFG, _secs(1.0)) == 4
    assert gc._stage_countdown_s(_enter("tutorial", teams), GAME_CFG, _secs(1.0)) == 3
    assert gc._stage_countdown_s(_enter("reset", teams), GAME_CFG, _secs(0.5)) == 2
    assert gc._stage_countdown_s(_enter("idle", teams), GAME_CFG, _secs(1.0)) == 0
    assert gc._stage_countdown_s(_enter("daydreaming", teams), GAME_CFG, _secs(1.0)) == 0
