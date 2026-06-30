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
gc._tutorial_config = gc_context._tutorial_config
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
        "reset_duration_s": 2,
        "idle_timeout_s": 3,
        "daydream_to_idle_error_deg": 900,
        "idle_to_tutorial_dial_deg": 360,
        "movement_window_s": 0.15,
        "movement_glitch_trim": 0,
        "sim_bucket_values": {"a": [120, 80, 40]},
        "start_stage": "daydreaming",
    }
)
TUTORIAL_CFG = gc._tutorial_config({"duration_s": 4})


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
        "last_tracking_target_dial_rad": [0.0] * 6,
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


def _tick(ss: dict, teams: dict, cfg: dict, now_ns: int, tutorial_cfg: dict | None = None) -> None:
    """Advance the stage machine with the matching tutorial config."""

    gc._tick_stage_state(
        ss,
        teams,
        cfg,
        tutorial_cfg if tutorial_cfg is not None else TUTORIAL_CFG,
        now_ns,
    )


def _countdown(ss: dict, cfg: dict, now_ns: int, tutorial_cfg: dict | None = None) -> int:
    """Return countdown using the split game/tutorial config shape."""

    return gc._stage_countdown_s(
        ss,
        cfg,
        tutorial_cfg if tutorial_cfg is not None else TUTORIAL_CFG,
        now_ns,
    )


def _secs(n: float) -> int:
    return int(n * 1e9)


def _arm(ss: dict, teams: dict, start_s: float = 0.1, step_s: float = 0.1) -> float:
    """Tick with the dials still until detection arms; return next tick time (s).

    Feeds a clean (still) rolling window so the team becomes armed, mirroring a
    settled boot. Returns the timestamp (seconds) for the caller's next tick.
    """
    t = start_s
    for _ in range(8):
        _tick(ss, teams, GAME_CFG, _secs(t))
        if ss["dial_arm"].get("a", {}).get("armed"):
            return round(t + step_s, 6)
        t = round(t + step_s, 6)
    raise AssertionError("movement detection never armed")


# --- daydreaming ----------------------------------------------------------


def test_daydreaming_dial_movement_goes_idle_without_player() -> None:
    teams = _make_teams()
    ss = _enter("daydreaming", teams)

    # Dial held on its tracking target -> still daydreaming.
    _tick(ss, teams, GAME_CFG, _secs(0.1))
    assert ss["stage"] == "daydreaming"

    # Push dial 0 past the residual wake threshold (900 deg) off its target.
    teams["a"]["last_dial"][0] = math.radians(950)
    _tick(ss, teams, GAME_CFG, _secs(0.2))
    assert ss["stage"] == "idle"


def test_daydreaming_ignores_subthreshold_movement() -> None:
    teams = _make_teams()
    ss = _enter("daydreaming", teams)

    teams["a"]["last_dial"][0] = math.radians(100)  # below 900 deg residual
    _tick(ss, teams, GAME_CFG, _secs(0.1))
    assert ss["stage"] == "daydreaming"


def test_daydreaming_skip_goes_idle_without_player() -> None:
    """SKIP exits daydream directly when there is no recorded playback."""

    teams = _make_teams()
    ss = _enter("daydreaming", teams)

    ss["skip_requested"] = True
    _tick(ss, teams, GAME_CFG, _secs(0.1))
    assert ss["stage"] == "idle"
    assert ss["skip_requested"] is False


def test_daydreaming_interrupt_recenters_still_goes_idle() -> None:
    """A latched wake enters interrupted mode and then finishes in idle."""

    class _StubPlayer:
        def start_forward(self) -> None:
            pass

    teams = _make_teams()
    teams["a"]["daydream_player"] = _StubPlayer()
    ss = _enter("daydreaming", teams)
    teams["a"]["last_dial"][0] = math.radians(950)  # > 900 deg residual -> wake
    _tick(ss, teams, GAME_CFG, _secs(0.1))
    assert ss["stage"] == "daydream_interrupted"
    assert teams["a"]["daydream_return_requested"] is True

    teams["a"]["last_dial"][0] = 0.0  # dial springs back below threshold
    _tick(ss, teams, GAME_CFG, _secs(0.2))
    assert ss["stage"] == "daydream_interrupted"

    teams["a"]["daydream_return_done"] = True
    _tick(ss, teams, GAME_CFG, _secs(0.3))
    assert ss["stage"] == "idle"


def test_daydream_interrupt_player_wait_blocks_straight_return(monkeypatch) -> None:
    """A recorded player owns interrupt motion while shortcut smoothing runs."""

    class _OptimizingPlayer:
        """Minimal player stub that has armed rewind but has no target yet."""

        rewind_complete = False

        def __init__(self) -> None:
            self.current_q_rad = None

        def begin_rewind(self, *, now_s: float, current_q_rad=None) -> bool:
            self.current_q_rad = current_q_rad
            return True

        def rewind_target(self, *, dt_s: float, q_actual_rad):
            return None

    player = _OptimizingPlayer()
    hold_calls = []
    state = {
        "daydream_player": player,
        "daydream_return_requested": True,
        "daydream_rewind_started": False,
        "daydream_return_done": False,
        "last_q": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    def _fake_hold(pub, producer: str, team: str, team_state: dict) -> None:
        hold_calls.append((producer, team, list(team_state["last_q"])))

    monkeypatch.setattr(gc, "_publish_hold_current_pose", _fake_hold)

    handled = gc._drive_daydream_playback(
        None, "game_controller", "a", state, 0.016, 42.0
    )

    assert handled is True
    assert state["daydream_rewind_started"] is True
    assert state["daydream_return_done"] is False
    assert player.current_q_rad == state["last_q"]
    assert hold_calls == [("game_controller", "a", state["last_q"])]


def test_daydream_loop_restart_waits_for_all_players() -> None:
    """One team cannot start the next playback loop before the other is ready."""

    class _LoopPlayer:
        def __init__(self, phase: str) -> None:
            self.phase = phase
            self.starts = 0

        def start_forward(self) -> None:
            self.phase = "forward"
            self.starts += 1

    player_a = _LoopPlayer("loop_waiting")
    player_b = _LoopPlayer("rewinding")
    teams = {
        "a": {"daydream_player": player_a},
        "b": {"daydream_player": player_b},
    }

    gc._restart_daydream_loop_if_ready(teams)
    assert player_a.starts == 0
    assert player_b.starts == 0

    player_b.phase = "loop_waiting"
    gc._restart_daydream_loop_if_ready(teams)
    assert player_a.starts == 1
    assert player_b.starts == 1


def test_single_frame_glitch_does_not_wake_idle() -> None:
    """A lone encoder glitch frame is trimmed out and does not wake idle.

    Idle uses the rolling peak-to-peak window. With movement_glitch_trim>0 a
    one-tick ~140 deg J6 spike is discarded from the range, so it never crosses
    the idle->tutorial threshold.
    """
    cfg = gc._game_config(
        {
            "idle_to_tutorial_dial_deg": 360,
            "idle_timeout_s": 30,
            "movement_window_s": 0.6,
            "movement_glitch_trim": 3,
            "start_stage": "idle",
        }
    )
    teams = _make_teams()
    ss = _new_stage_state()
    gc._enter_stage(ss, teams, "idle", cfg, 0, reason="test")
    # Fill a still window (10 ticks @ 0.05 s spans 0.45 s -> a full 0.6 s window
    # after the next few ticks) so detection arms with clean data.
    t = 0.05
    for _ in range(20):
        _tick(ss, teams, cfg, _secs(t), gc._tutorial_config({}))
        t = round(t + 0.05, 6)
        if ss["dial_arm"].get("a", {}).get("armed"):
            break
    assert ss["dial_arm"]["a"]["armed"]
    # Inject a single 140 deg glitch frame on J6, then return to still.
    teams["a"]["last_dial"][5] = math.radians(140)
    _tick(ss, teams, cfg, _secs(t), gc._tutorial_config({}))
    teams["a"]["last_dial"][5] = 0.0
    _tick(ss, teams, cfg, _secs(round(t + 0.05, 6)), gc._tutorial_config({}))
    assert ss["stage"] == "idle"  # glitch trimmed -> no false wake


# --- idle -----------------------------------------------------------------


def test_idle_to_tutorial_on_scroll_up() -> None:
    teams = _make_teams()
    ss = _enter("idle", teams)
    t = _arm(ss, teams)

    teams["a"]["last_dial"][0] = math.radians(400)  # past 360 deg
    _tick(ss, teams, GAME_CFG, _secs(t))
    assert ss["stage"] == "tutorial"


def test_idle_to_daydreaming_on_timeout() -> None:
    teams = _make_teams()
    ss = _enter("idle", teams)
    _tick(ss, teams, GAME_CFG, _secs(0.1))  # no movement

    _tick(ss, teams, GAME_CFG, _secs(3.5))  # past idle_timeout 3s
    assert ss["stage"] == "daydreaming"


def test_idle_to_tutorial_on_skip() -> None:
    teams = _make_teams()
    ss = _enter("idle", teams)
    ss["skip_requested"] = True

    _tick(ss, teams, GAME_CFG, _secs(0.1))

    assert ss["stage"] == "tutorial"
    assert ss["skip_requested"] is False


# --- tutorial -------------------------------------------------------------


def test_tutorial_to_play_on_timer() -> None:
    teams = _make_teams()
    ss = _enter("tutorial", teams)
    _tick(ss, teams, GAME_CFG, _secs(1.0))
    assert ss["stage"] == "tutorial"
    _tick(ss, teams, GAME_CFG, _secs(4.5))  # past 4s
    assert ss["stage"] == "play"


def test_tutorial_to_play_on_skip() -> None:
    teams = _make_teams()
    ss = _enter("tutorial", teams)
    ss["skip_requested"] = True
    _tick(ss, teams, GAME_CFG, _secs(0.1))
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

        _tick(ss, teams, GAME_CFG, _secs(0.1))

        assert ss["stage"] == "play"


def test_tutorial_waits_when_any_active_player_is_below_97_percent() -> None:
    """A single incomplete player on either active team keeps the timer running."""

    teams = _make_teams()
    teams["b"] = _make_team()
    teams["b"]["team"] = "b"
    ss = _enter("tutorial", teams)
    teams["a"]["tutorial_progress"] = [100.0] * 6
    teams["b"]["tutorial_progress"] = [97.0] * 5 + [96.99]

    _tick(ss, teams, GAME_CFG, _secs(0.1))

    assert ss["stage"] == "tutorial"


# --- play -----------------------------------------------------------------


def test_play_to_reset_on_timer() -> None:
    teams = _make_teams()
    ss = _enter("play", teams)
    _tick(ss, teams, GAME_CFG, _secs(1.0))
    assert ss["stage"] == "play"
    _tick(ss, teams, GAME_CFG, _secs(5.5))  # past 5s
    assert ss["stage"] == "reset"


def test_play_to_reset_on_skip() -> None:
    teams = _make_teams()
    ss = _enter("play", teams)
    ss["skip_requested"] = True
    _tick(ss, teams, GAME_CFG, _secs(0.1))
    assert ss["stage"] == "reset"


# --- reset ----------------------------------------------------------------


def test_reset_to_conclusion_on_timer() -> None:
    teams = _make_teams()
    ss = _enter("reset", teams)
    _tick(ss, teams, GAME_CFG, _secs(0.5))
    assert ss["stage"] == "reset"
    _tick(ss, teams, GAME_CFG, _secs(2.5))  # past 2s
    assert ss["stage"] == "conclusion"


def test_reset_is_not_skippable() -> None:
    teams = _make_teams()
    ss = _enter("reset", teams)
    ss["skip_requested"] = True  # skip should be ignored in reset
    _tick(ss, teams, GAME_CFG, _secs(0.5))
    assert ss["stage"] == "reset"


# --- conclusion -----------------------------------------------------------


def test_conclusion_to_idle_when_all_done() -> None:
    teams = _make_teams()
    ss = _enter("conclusion", teams)
    # Not done yet -> stays.
    _tick(ss, teams, GAME_CFG, _secs(0.5))
    assert ss["stage"] == "conclusion"
    # Mark every team finished -> idle.
    teams["a"]["conclusion_done"] = True
    _tick(ss, teams, GAME_CFG, _secs(1.0))
    assert ss["stage"] == "idle"


# --- skip authorization (via the UI request handler) ----------------------


def _control_state() -> dict:
    return {"soft_pause": False, "last_action": None, "last_action_ts_mono_ns": None}


def test_skip_rejected_in_non_skippable_stages() -> None:
    teams = _make_teams()
    for stage in ("daydream_interrupted", "reset", "conclusion"):
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
        assert reply["ok"] is False, stage
        assert reply["result"]["action"] == "skip"
        assert ss["skip_requested"] is False
        assert stage in str(reply["error"] or "")


def test_skip_accepted_in_skippable_stages() -> None:
    teams = _make_teams()
    for stage in ("daydreaming", "idle", "tutorial", "play"):
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
    assert _countdown(_enter("play", teams), GAME_CFG, _secs(1.0)) == 4
    assert _countdown(_enter("tutorial", teams), GAME_CFG, _secs(1.0)) == 3
    assert _countdown(_enter("reset", teams), GAME_CFG, _secs(0.5)) == 2
    assert _countdown(_enter("idle", teams), GAME_CFG, _secs(1.0)) == 0
    assert _countdown(_enter("daydreaming", teams), GAME_CFG, _secs(1.0)) == 0
