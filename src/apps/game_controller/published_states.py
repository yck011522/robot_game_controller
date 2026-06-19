"""Helpers that build published game-controller state payloads.

This module is intentionally side-effect free: it assembles plain dictionaries
for state publication, while the caller owns the actual bus envelope/publish.
"""

from __future__ import annotations

from typing import Any

from apps.game_controller.haptics import _coerce_float_list
from apps.game_controller.safety import _safety_pause_reason, _state_full_safety_barrier
from apps.game_controller.weight import _state_full_weight_sensor


def _state_full_planner(info: Any) -> dict[str, Any]:
    """Build the compact planner diagnostics block for state.full traces."""

    data = info if isinstance(info, dict) else {}
    return {
        "input_mode": data.get("input_mode"),
        "forward_certified": data.get("forward_certified"),
        "v_cmd_rad_s": _coerce_float_list(data.get("v_cmd_rad_s"), [0.0] * 6),
        "v_out_rad_s": _coerce_float_list(data.get("v_out_rad_s"), [0.0] * 6),
    }


def _pause_state_summary(
    control_state: dict[str, Any],
    safety_state: dict[str, Any],
    teams: dict[str, dict[str, Any]],
    *,
    soft_paused: bool,
) -> tuple[bool, str | None]:
    """Return the published paused flag and pause reason for the controller."""

    paused_teams = [
        team
        for team, st in teams.items()
        if bool(st.get("robot_status", {}).get("fault_active", False))
    ]
    pause_reason = None
    for team in paused_teams:
        reason = teams[team].get("robot_status", {}).get("fault_reason")
        if isinstance(reason, str) and reason:
            pause_reason = f"{team}:{reason}"
            break
    if bool(control_state.get("recovery_active", False)):
        pause_reason = "recovery"
    elif bool(control_state.get("safety_blocked", False)):
        pause_reason = _safety_pause_reason(safety_state)
    elif bool(control_state.get("safety_pause_latched", False)):
        pause_reason = "barrier_ack_required"
    elif soft_paused:
        pause_reason = "soft_estop"

    paused = bool(paused_teams) or soft_paused
    return paused, pause_reason


def _build_state_full_payload(
    stage_state: dict[str, Any],
    safety_state: dict[str, Any],
    weight_state: dict[str, Any],
    teams: dict[str, dict[str, Any]],
    game_cfg: dict[str, Any],
    haptic_cfg: dict[str, Any],
    *,
    paused: bool,
    pause_reason: str | None,
    soft_paused: bool,
    countdown_s: int | float,
) -> dict[str, Any]:
    """Build the full `state.full` payload body for one controller tick."""

    return {
        "stage": "paused" if paused else stage_state["stage"],
        "active_stage": stage_state["stage"],
        "paused": paused,
        "pause_reason": pause_reason,
        "soft_estop": soft_paused,
        "safety": {
            "barrier": _state_full_safety_barrier(safety_state),
        },
        "weight_sensor": _state_full_weight_sensor(weight_state),
        "countdown_s": countdown_s,
        "game_duration_s": game_cfg["duration_s"],
        "sum_score_rate_unit_per_s": game_cfg["sum_score_rate_unit_per_s"],
        "stage_entered_mono_ns": stage_state["stage_entered_mono_ns"],
        "tutorial_entered_wall_ns": None,
        "teams": {
            team: _team_state_full_payload(st, haptic_cfg)
            for team, st in teams.items()
        },
    }


def _team_state_full_payload(
    team_state: dict[str, Any], haptic_cfg: dict[str, Any]
) -> dict[str, Any]:
    """Build one team's nested block inside the published state.full payload."""

    rewind = team_state.get("rewind")
    rewind_state = rewind.snapshot() if rewind is not None else {
        "enabled": False,
        "status": "disabled",
        "recorded_point_count": 0,
        "point_count": 0,
        "current_index": 0,
        "progress": 0.0,
        "initial_q_rad": None,
        "max_error_deg": None,
    }

    return {
        "robot": {
            "q_target_rad": team_state["last_target"],
            "q_rad": (
                team_state["last_q"] if team_state["last_q"] is not None else [0.0] * 6
            ),
            "status": team_state.get("robot_status", {}),
        },
        "haptic": {
            "dial_pos_rad": team_state["last_dial"],
            "dial_vel_rad_s": team_state["last_dial_vel"],
            "connected": team_state["last_haptic_connected"],
            "board_loop_hz": team_state["last_haptic_loop_hz"],
            "bounds_min_rad": list(
                team_state.get("current_haptic_bounds_min_rad")
                or haptic_cfg["bounds_min_rad"]
            ),
            "bounds_max_rad": list(
                team_state.get("current_haptic_bounds_max_rad")
                or haptic_cfg["bounds_max_rad"]
            ),
            "play_sync": {
                "enabled": bool(
                    team_state.get("play_sync", {}).get("enabled", False)
                ),
                "requested": bool(
                    team_state.get("play_sync", {}).get("requested", False)
                ),
                "pending": bool(
                    team_state.get("play_sync", {}).get("pending", False)
                ),
                "settled_streak": int(
                    team_state.get("play_sync", {}).get("settled_streak", 0)
                ),
                "attempts": int(
                    team_state.get("play_sync", {}).get("attempts", 0)
                ),
            },
        },
        "collision": {
            "in_collision": team_state["last_collision"],
            "first_hit": team_state["last_first_hit"],
            "path_scalar": team_state["last_path_scalar"],
            "prox_scalar": team_state["last_prox_scalar"],
            "final_scalar": team_state["last_final_scalar"],
            "prox_probe_offsets_deg": team_state["last_prox_probe_offsets_deg"],
            "prox_hits": team_state["last_prox_hits"],
            "prox_age_ticks": team_state["last_prox_age_ticks"],
        },
        "planner": _state_full_planner(team_state.get("last_planner_info")),
        "rewind": rewind_state,
        "score": team_state["score"],
        "summed_score": team_state["summed_score"],
        "bucket_labels": list(team_state.get("bucket_labels", [])),
        "buckets": list(team_state["bucket_values"]),
        "conclusion": {
            "phase": team_state["conclusion_phase"],
            "active_bucket_index": team_state["conclusion_active_bucket_index"],
            "target_pose_name": team_state["conclusion_target_pose_name"],
            "target_pose_deg": team_state["conclusion_target_pose_deg"],
            "bucket_open_triggered": team_state["conclusion_bucket_open_triggered"],
            "done": team_state["conclusion_done"],
        },
    }
