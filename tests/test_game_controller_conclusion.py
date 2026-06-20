"""Focused tests for GameController conclusion sequencing."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import __main__ as game_controller  # noqa: E402
from apps.game_controller import stages as game_controller_stages  # noqa: E402

# The conclusion scoring sequence (and its CONCLUSION_* timing constants) moved
# from __main__ into the stages module during the P7 refactor. Re-bind the
# moved names onto the ``game_controller`` alias so the call sites below keep
# exercising the real (now relocated) functions / constants.
game_controller._tick_conclusion_team = game_controller_stages._tick_conclusion_team
game_controller.CONCLUSION_INITIAL_PAUSE_S = (
    game_controller_stages.CONCLUSION_INITIAL_PAUSE_S
)
game_controller.CONCLUSION_BUCKET_EMPTY_PAUSE_S = (
    game_controller_stages.CONCLUSION_BUCKET_EMPTY_PAUSE_S
)
game_controller.CONCLUSION_ANNOUNCEMENT_PAUSE_S = (
    game_controller_stages.CONCLUSION_ANNOUNCEMENT_PAUSE_S
)
game_controller.CONCLUSION_WINNER_POSE_HOLD_S = (
    game_controller_stages.CONCLUSION_WINNER_POSE_HOLD_S
)


def _tick(state, **overrides):
    """Helper: advance one conclusion tick with sensible test defaults."""

    kwargs = {
        "dt": overrides.pop("dt", 1.0),
        "game_cfg": overrides.pop("game_cfg", {"sum_score_rate_unit_per_s": 100.0}),
        "pose_cfg": overrides.pop("pose_cfg", {}),
        "stage_state": overrides.pop("stage_state", {"winner_team": None}),
    }
    kwargs.update(overrides)
    game_controller._tick_conclusion_team(state, **kwargs)


def test_conclusion_uses_one_based_bucket_command() -> None:
    """Conclusion should publish B2, not a zero-based bucket index."""

    commands: list[dict] = []
    state = {
        "team": "b",
        "bucket_values": [0, 1, 0],
        "summed_score": 0,
        "score": 1,
        "conclusion_phase": "sum_bucket",
        "conclusion_active_bucket_index": 1,
        "conclusion_phase_elapsed_s": 0.0,
        "conclusion_sum_remainder_units": 0.0,
    }

    _tick(
        state,
        bucket_command_fn=lambda *args, **kwargs: commands.append(
            {"args": args, **kwargs}
        ),
    )

    assert commands == [
        {
            "args": ("open",),
            "team": "b",
            "bucket_number": 2,
            "reason": "conclusion_bucket_counted",
        }
    ]
    assert state["conclusion_phase"] == "empty_bucket"


def test_pause_then_move_request_then_arrival_starts_sum() -> None:
    """pause_before_sum -> move_to_bucket_pose (pending) -> arrival -> sum_bucket."""

    state = {
        "team": "b",
        "bucket_values": [10, 0, 0],
        "summed_score": 0,
        "score": 10,
        "conclusion_phase": "pause_before_sum",
        "conclusion_active_bucket_index": 0,
        "conclusion_phase_elapsed_s": game_controller.CONCLUSION_INITIAL_PAUSE_S,
        "conclusion_sum_remainder_units": 0.0,
    }
    pose_cfg = {"robot_lookb1_pose": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}

    _tick(state, dt=0.0, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] == "move_to_bucket_pose"
    assert state["conclusion_target_pose_name"] == "robot_lookb1_pose"
    assert state["conclusion_move_pending"] is True
    assert state["conclusion_move_arrived"] is False

    _tick(state, dt=0.5, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] == "move_to_bucket_pose"

    state["conclusion_move_arrived"] = True
    _tick(state, dt=0.5, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] == "sum_bucket"
    assert state["bucket_values"] == [10, 0, 0]


def test_final_bucket_empty_requests_announcement_move() -> None:
    """After the last bucket empties, the team moves to the announcement pose."""

    state = {
        "team": "b",
        "bucket_values": [0, 0, 0],
        "summed_score": 30,
        "score": 0,
        "conclusion_phase": "empty_bucket",
        "conclusion_active_bucket_index": 2,
        "conclusion_phase_elapsed_s": game_controller.CONCLUSION_BUCKET_EMPTY_PAUSE_S,
        "conclusion_sum_remainder_units": 0.0,
        "conclusion_bucket_open_triggered": True,
    }
    pose_cfg = {"robot_announcement_pose": [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]}

    _tick(state, dt=0.0, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] == "move_to_announcement"
    assert state["conclusion_target_pose_name"] == "robot_announcement_pose"
    assert state["conclusion_move_pending"] is True


def test_announcement_pause_waits_for_winner_then_moves_to_pose() -> None:
    """announcement_pause holds, resolves win/lose pose, then moves to it."""

    state = {
        "team": "b",
        "bucket_values": [0, 0, 0],
        "summed_score": 30,
        "score": 0,
        "conclusion_phase": "announcement_pause",
        "conclusion_active_bucket_index": 3,
        "conclusion_phase_elapsed_s": game_controller.CONCLUSION_ANNOUNCEMENT_PAUSE_S,
        "conclusion_sum_remainder_units": 0.0,
    }
    pose_cfg = {
        "robot_win_pose": [1, 1, 1, 1, 1, 1],
        "robot_lose_pose": [2, 2, 2, 2, 2, 2],
    }

    _tick(state, dt=0.0, pose_cfg=pose_cfg, stage_state={"winner_team": None})
    assert state["conclusion_phase"] == "announcement_pause"

    _tick(state, dt=0.0, pose_cfg=pose_cfg, stage_state={"winner_team": "b"})
    assert state["conclusion_phase"] == "move_to_winner_pose"
    assert state["conclusion_target_pose_name"] == "robot_win_pose"
    assert state["conclusion_move_pending"] is True


def test_winner_hold_then_move_to_begin_then_done() -> None:
    """winner_pose_hold -> move_to_begin -> arrival completes the show."""

    state = {
        "team": "a",
        "bucket_values": [0, 0, 0],
        "summed_score": 30,
        "score": 0,
        "conclusion_phase": "winner_pose_hold",
        "conclusion_phase_elapsed_s": game_controller.CONCLUSION_WINNER_POSE_HOLD_S,
        "conclusion_sum_remainder_units": 0.0,
    }
    pose_cfg = {"robot_begin_pose": [0, -116, 116, -35, 95, 180]}

    _tick(state, dt=0.0, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] == "move_to_begin"
    assert state["conclusion_target_pose_name"] == "robot_begin_pose"

    state["conclusion_move_arrived"] = True
    _tick(state, dt=0.0, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] is None
    assert state["conclusion_done"] is True


def test_phase_clock_is_pause_aware_accumulated_dt() -> None:
    """The phase clock only advances by dt, so skipped (paused) ticks freeze it."""

    state = {
        "team": "b",
        "bucket_values": [5, 0, 0],
        "summed_score": 0,
        "score": 5,
        "conclusion_phase": "empty_bucket",
        "conclusion_active_bucket_index": 0,
        "conclusion_phase_elapsed_s": 0.0,
        "conclusion_sum_remainder_units": 0.0,
    }
    pose_cfg = {"robot_lookb2_pose": [1, 2, 3, 4, 5, 6]}

    _tick(state, dt=0.2, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] == "empty_bucket"
    assert abs(state["conclusion_phase_elapsed_s"] - 0.2) < 1e-9

    _tick(state, dt=game_controller.CONCLUSION_BUCKET_EMPTY_PAUSE_S, pose_cfg=pose_cfg)
    assert state["conclusion_phase"] == "move_to_bucket_pose"
    assert state["conclusion_active_bucket_index"] == 1
    assert state["conclusion_target_pose_name"] == "robot_lookb2_pose"


def main() -> int:
    test_conclusion_uses_one_based_bucket_command()
    test_pause_then_move_request_then_arrival_starts_sum()
    test_final_bucket_empty_requests_announcement_move()
    test_announcement_pause_waits_for_winner_then_moves_to_pose()
    test_winner_hold_then_move_to_begin_then_done()
    test_phase_clock_is_pause_aware_accumulated_dt()
    print("[test] game controller conclusion tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
