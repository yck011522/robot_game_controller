"""Unit tests for the play-stage practice sub-state.

The practice sub-state lets players 1..6 take turns jogging their own joint
(player N -> joint N-1) from ``robot_begin_pose`` to
``robot_practice_target_pose`` one at a time, while every other joint is frozen
at its exact begin / target angle. These tests exercise the pure helpers in
``apps.game_controller.stages`` (dial masking + arrival hand-off) and the
per-team seeding done on play entry, with no ZeroMQ bus, planner, or robot.

Run:
    python -m pytest tests/test_practice_substate.py -q
    # full env:
    C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe -m pytest tests/test_practice_substate.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import context as gc_context  # noqa: E402
from apps.game_controller import published_states as gc_published  # noqa: E402
from apps.game_controller import stages as gc_stages  # noqa: E402

# Gear ratio matching the production absolute-mode profile (dial -> joint via
# dial * gear). Negative to mirror the real hardware direction flip.
GEAR = [-0.1] * 6
# One second in nanoseconds, for building monotonic ``now_ns`` values.
_S = 1_000_000_000


def _feed(*, completed, active_idx, live_dial,
          begin_deg=None, target_deg=None):
    """Build a masked dial feed for a 6-joint team with the given state.

    ``begin_deg`` / ``target_deg`` default to distinct simple poses so the test
    can assert exact ``angle / gear`` freezing per joint.
    """
    begin_deg = begin_deg if begin_deg is not None else [10.0] * 6
    target_deg = target_deg if target_deg is not None else [90.0] * 6
    return gc_stages._practice_masked_dial_feed(
        begin_pose_rad=[math.radians(v) for v in begin_deg],
        target_pose_rad=[math.radians(v) for v in target_deg],
        gear=GEAR,
        completed=completed,
        active_idx=active_idx,
        live_dial=live_dial,
    )


# --- Masked dial feed -------------------------------------------------------


def test_masked_feed_active_joint_follows_live_dial():
    """The active joint's feed is passed through untouched from the live dial."""
    live = [1.11, 2.22, 3.33, 4.44, 5.55, 6.66]
    feed = _feed(completed=[False] * 6, active_idx=2, live_dial=live)
    assert feed[2] == 3.33


def test_masked_feed_not_started_joint_frozen_at_begin_over_gear():
    """A not-yet-started joint is commanded to exactly begin_pose / gear."""
    begin_deg = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    feed = _feed(
        completed=[False] * 6,
        active_idx=0,
        live_dial=[0.0] * 6,
        begin_deg=begin_deg,
    )
    # Joints 1..5 are not started -> begin_pose_rad / gear.
    for i in range(1, 6):
        assert math.isclose(feed[i], math.radians(begin_deg[i]) / GEAR[i])


def test_masked_feed_completed_joint_frozen_at_target_over_gear():
    """A completed joint is commanded to exactly target_pose / gear (exact latch)."""
    target_deg = [91.0, 92.0, 93.0, 94.0, 95.0, 96.0]
    completed = [True, True, False, False, False, False]
    feed = _feed(
        completed=completed,
        active_idx=2,
        live_dial=[7.0] * 6,
        target_deg=target_deg,
    )
    # Completed joints 0,1 -> target / gear; active joint 2 -> live dial.
    assert math.isclose(feed[0], math.radians(target_deg[0]) / GEAR[0])
    assert math.isclose(feed[1], math.radians(target_deg[1]) / GEAR[1])
    assert feed[2] == 7.0


def test_masked_feed_is_pure_and_returns_fresh_list():
    """The helper never mutates its inputs and returns a new six-element list."""
    completed = [False] * 6
    live = [0.0] * 6
    feed = _feed(completed=completed, active_idx=0, live_dial=live)
    assert len(feed) == 6
    assert feed is not live
    assert completed == [False] * 6  # unchanged


# --- Arrival hand-off -------------------------------------------------------


def _practice_team():
    """A minimal team scratch dict mid-practice: player 1 active, none done."""
    return {
        "in_practice": True,
        "practice_player": 1,
        "practice_completed": [False] * 6,
        "practice_dwell_start_ns": None,
    }


def _arrive(st, *, now_ns, on_target=True):
    """Drive one arrival tick with the active joint on/off the target."""
    target_rad = math.radians(90.0)
    gc_stages._tick_practice_arrival(
        st,
        active_q_target_rad=target_rad if on_target else 0.0,
        active_idx=int(st["practice_player"]) - 1,
        target_rad=target_rad,
        tolerance_rad=math.radians(0.5),
        dwell_s=0.4,
        now_ns=now_ns,
    )


def test_arrival_requires_dwell_before_advancing():
    """A single on-target tick starts the dwell timer but does not advance."""
    st = _practice_team()
    _arrive(st, now_ns=0, on_target=True)
    assert st["practice_player"] == 1
    assert st["practice_dwell_start_ns"] == 0
    # Still inside the 0.4 s dwell -> no advance.
    _arrive(st, now_ns=int(0.3 * _S), on_target=True)
    assert st["practice_player"] == 1
    assert st["practice_completed"] == [False] * 6


def test_arrival_advances_after_sustained_dwell():
    """Holding on target past the dwell latches joint 0 and advances to player 2."""
    st = _practice_team()
    _arrive(st, now_ns=0, on_target=True)
    _arrive(st, now_ns=int(0.5 * _S), on_target=True)
    assert st["practice_completed"][0] is True
    assert st["practice_player"] == 2
    assert st["practice_dwell_start_ns"] is None


def test_arrival_flythrough_resets_dwell():
    """Leaving the tolerance band before the dwell elapses restarts the timer."""
    st = _practice_team()
    _arrive(st, now_ns=0, on_target=True)
    assert st["practice_dwell_start_ns"] == 0
    _arrive(st, now_ns=int(0.2 * _S), on_target=False)
    assert st["practice_dwell_start_ns"] is None
    assert st["practice_player"] == 1


def test_all_six_players_complete_clears_in_practice():
    """Cycling every player to arrival ends practice with all joints latched."""
    st = _practice_team()
    now = 0
    for _ in range(6):
        _arrive(st, now_ns=now, on_target=True)          # start dwell
        now += int(0.5 * _S)
        _arrive(st, now_ns=now, on_target=True)          # exceed dwell -> advance
        now += int(0.1 * _S)
    assert st["practice_completed"] == [True] * 6
    assert st["in_practice"] is False


def test_arrival_noop_when_not_in_practice():
    """Once practice is over, further arrival ticks are inert."""
    st = _practice_team()
    st["in_practice"] = False
    _arrive(st, now_ns=0, on_target=True)
    assert st["practice_dwell_start_ns"] is None
    assert st["practice_player"] == 1


# --- Seeding on play entry --------------------------------------------------


def test_seed_play_teams_enables_practice_when_configured():
    """_seed_play_teams arms the practice sub-state when practice_enabled."""
    game_cfg = gc_context._game_config(
        {"sim_bucket_values": {"a": [1, 2, 3]}, "practice_enabled": True}
    )
    teams = {"a": {}}
    gc_stages._seed_play_teams(teams, game_cfg)
    st = teams["a"]
    assert st["in_practice"] is True
    assert st["practice_player"] == 1
    assert st["practice_completed"] == [False] * 6
    assert st["practice_dwell_start_ns"] is None


def test_seed_play_teams_leaves_practice_off_by_default():
    """With practice disabled, teams start play in normal six-player gameplay."""
    game_cfg = gc_context._game_config({"sim_bucket_values": {"a": [1, 2, 3]}})
    teams = {"a": {}}
    gc_stages._seed_play_teams(teams, game_cfg)
    assert teams["a"]["in_practice"] is False


def test_practice_target_pose_loaded_from_show_poses():
    """The loader now surfaces robot_practice_target_pose per team."""
    poses = gc_context._load_robot_show_poses_deg()
    for team_poses in poses.values():
        assert "robot_practice_target_pose" in team_poses
        assert len(team_poses["robot_practice_target_pose"]) == 6


# --- Published payload ------------------------------------------------------


def test_published_practice_block_reports_turn_and_target():
    """state.full carries the per-team practice block for the player displays."""
    from apps.game_controller.haptics import _haptic_config

    haptic_cfg = _haptic_config({"gear_ratio": GEAR})
    team_state = _full_team_state()
    payload = gc_published._team_state_full_payload(team_state, haptic_cfg)
    practice = payload["practice"]
    assert practice["in_practice"] is True
    assert practice["active_player"] == 3
    assert practice["active_joint_index"] == 2
    assert practice["completed"] == [True, True, False, False, False, False]
    assert practice["target_pose_deg"] == [-50.0, -60.0, 80.0, -10.0, 90.0, 0.0]


def test_published_practice_block_active_player_null_when_done():
    """Once in_practice clears, active_player / active_joint_index are null."""
    from apps.game_controller.haptics import _haptic_config

    haptic_cfg = _haptic_config({"gear_ratio": GEAR})
    team_state = _full_team_state()
    team_state["in_practice"] = False
    payload = gc_published._team_state_full_payload(team_state, haptic_cfg)
    practice = payload["practice"]
    assert practice["in_practice"] is False
    assert practice["active_player"] is None
    assert practice["active_joint_index"] is None


def test_published_state_full_winner_team_is_conclusion_latch():
    """state.full publishes winner_team only when conclusion reveal is ready."""
    from apps.game_controller.haptics import _haptic_config

    haptic_cfg = _haptic_config({"gear_ratio": GEAR})
    stage_state = {
        "stage": "conclusion",
        "stage_entered_mono_ns": 123,
        "winner_team": "a",
    }
    game_cfg = {"duration_s": 10, "sum_score_rate_unit_per_s": 5}
    payload = gc_published._build_state_full_payload(
        stage_state,
        safety_state={},
        weight_state={},
        teams={"a": _full_team_state()},
        game_cfg=game_cfg,
        haptic_cfg=haptic_cfg,
        paused=False,
        pause_reason=None,
        soft_paused=False,
        countdown_s=0,
    )
    assert payload["winner_team"] == "a"

    # The same internal value is hidden outside conclusion so receivers can
    # treat non-null as a conclusion-stage reveal latch.
    stage_state["stage"] = "idle"
    payload = gc_published._build_state_full_payload(
        stage_state,
        safety_state={},
        weight_state={},
        teams={"a": _full_team_state()},
        game_cfg=game_cfg,
        haptic_cfg=haptic_cfg,
        paused=False,
        pause_reason=None,
        soft_paused=False,
        countdown_s=0,
    )
    assert payload["winner_team"] is None


def _full_team_state():
    """A team scratch dict populated with every key _team_state_full_payload reads."""
    return {
        "rewind": None,
        "last_dial": [0.0] * 6,
        "last_dial_vel": [0.0] * 6,
        "last_target": [0.0] * 6,
        "last_q": [0.0] * 6,
        "robot_status": {},
        "last_haptic_connected": [False] * 6,
        "last_haptic_loop_hz": [0.0] * 6,
        "tutorial_progress": [0.0] * 6,
        "current_haptic_bounds_min_rad": [-1.0] * 6,
        "current_haptic_bounds_max_rad": [1.0] * 6,
        "play_sync": {},
        "last_collision": False,
        "last_first_hit": None,
        "last_path_scalar": 1.0,
        "last_prox_scalar": 1.0,
        "last_final_scalar": 1.0,
        "last_prox_probe_offsets_deg": [],
        "last_prox_hits": [[False] * 20 for _ in range(6)],
        "last_prox_age_ticks": [9999] * 6,
        "last_planner_info": {},
        "score": 0,
        "summed_score": 0,
        "bucket_labels": [],
        "bucket_values": [0, 0, 0],
        "conclusion_phase": None,
        "conclusion_active_bucket_index": None,
        "conclusion_target_pose_name": None,
        "conclusion_target_pose_deg": None,
        "conclusion_bucket_open_triggered": False,
        "conclusion_done": False,
        # Practice: player 3 active, players 1-2 already latched.
        "in_practice": True,
        "practice_player": 3,
        "practice_completed": [True, True, False, False, False, False],
        "practice_dwell_start_ns": None,
        "practice_target_pose_deg": [-50.0, -60.0, 80.0, -10.0, 90.0, 0.0],
        "practice_arrival_tolerance_deg": 0.5,
    }
