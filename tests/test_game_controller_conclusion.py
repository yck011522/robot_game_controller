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
game_controller.CONCLUSION_BUCKET_LOOK_MOTION_WAIT_S = (
    game_controller_stages.CONCLUSION_BUCKET_LOOK_MOTION_WAIT_S
)
game_controller.CONCLUSION_BUCKET_EMPTY_PAUSE_S = (
    game_controller_stages.CONCLUSION_BUCKET_EMPTY_PAUSE_S
)
game_controller.CONCLUSION_CELEBRATION_MOTION_WAIT_S = (
    game_controller_stages.CONCLUSION_CELEBRATION_MOTION_WAIT_S
)



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
        "conclusion_phase_started_mono_ns": 0,
        "conclusion_sum_remainder_units": 0.0,
    }

    game_controller._tick_conclusion_team(
        state,
        dt=1.0,
        game_cfg={"sum_score_rate_unit_per_s": 100.0},
        pose_cfg={},
        stage_state={"winner_team": None},
        bucket_command_fn=lambda *args, **kwargs: commands.append({"args": args, **kwargs}),
    )

    assert commands == [
        {
            "args": ("open",),
            "team": "b",
            "bucket_number": 2,
            "reason": "conclusion_bucket_counted",
        }
    ]


def test_conclusion_waits_for_temporary_bucket_look_motion() -> None:
    """Conclusion should wait before summing to stand in for future robot motion."""

    state = {
        "team": "b",
        "bucket_values": [10, 0, 0],
        "summed_score": 0,
        "score": 10,
        "conclusion_phase": "pause_before_sum",
        "conclusion_active_bucket_index": 0,
        "conclusion_phase_started_mono_ns": -int(game_controller.CONCLUSION_INITIAL_PAUSE_S * 1e9),
        "conclusion_sum_remainder_units": 0.0,
    }
    pose_cfg = {"robot_lookb1_pose": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}

    game_controller._tick_conclusion_team(
        state,
        dt=1.0,
        game_cfg={"sum_score_rate_unit_per_s": 100.0},
        pose_cfg=pose_cfg,
        stage_state={"winner_team": None},
    )
    assert state["conclusion_phase"] == "move_to_bucket_pose"
    assert state["conclusion_target_pose_name"] == "robot_lookb1_pose"
    assert state["bucket_values"] == [10, 0, 0]

    state["conclusion_phase_started_mono_ns"] = -int(
        game_controller.CONCLUSION_BUCKET_LOOK_MOTION_WAIT_S * 1e9
    )
    game_controller._tick_conclusion_team(
        state,
        dt=1.0,
        game_cfg={"sum_score_rate_unit_per_s": 100.0},
        pose_cfg=pose_cfg,
        stage_state={"winner_team": None},
    )
    assert state["conclusion_phase"] == "sum_bucket"
    assert state["bucket_values"] == [10, 0, 0]


def test_conclusion_waits_for_temporary_celebration_motion() -> None:
    """Conclusion should wait after the final bucket opens for future celebration motion."""

    state = {
        "team": "b",
        "bucket_values": [0, 0, 0],
        "summed_score": 30,
        "score": 0,
        "conclusion_phase": "empty_bucket",
        "conclusion_active_bucket_index": 2,
        "conclusion_phase_started_mono_ns": -int(game_controller.CONCLUSION_BUCKET_EMPTY_PAUSE_S * 1e9),
        "conclusion_sum_remainder_units": 0.0,
        "conclusion_bucket_open_triggered": True,
    }
    pose_cfg = {"robot_celebration_pose": [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]}

    game_controller._tick_conclusion_team(
        state,
        dt=1.0,
        game_cfg={"sum_score_rate_unit_per_s": 100.0},
        pose_cfg=pose_cfg,
        stage_state={"winner_team": None},
    )
    assert state["conclusion_phase"] == "celebration_motion"
    assert state["conclusion_target_pose_name"] == "robot_celebration_pose"

    state["conclusion_phase_started_mono_ns"] = -int(
        game_controller.CONCLUSION_CELEBRATION_MOTION_WAIT_S * 1e9
    )
    game_controller._tick_conclusion_team(
        state,
        dt=1.0,
        game_cfg={"sum_score_rate_unit_per_s": 100.0},
        pose_cfg=pose_cfg,
        stage_state={"winner_team": None},
    )
    assert state["conclusion_phase"] == "announcement_pose"


def main() -> int:
    test_conclusion_uses_one_based_bucket_command()
    test_conclusion_waits_for_temporary_bucket_look_motion()
    test_conclusion_waits_for_temporary_celebration_motion()
    print("[test] game controller conclusion tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
