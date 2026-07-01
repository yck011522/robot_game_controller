"""Helpers that build published game-controller state payloads.

This module is intentionally side-effect free: it assembles plain dictionaries
for state publication, while the caller owns the actual bus envelope/publish.
"""

from __future__ import annotations

import math

from typing import Any

from apps.game_controller.haptics import _coerce_float_list
from apps.game_controller.safety import _safety_pause_reason, _state_full_safety_barrier
from apps.game_controller.weight import _state_full_weight_sensor


# Proximity probe results older than this many planner ticks are treated as
# stale: the corresponding lane has no fresh collision test, so the published
# zone is marked invalid and a receiver should render the whole lane as the
# default "untested" grey. Kept in lock-step with the gamemaster dashboard so
# both consumers gate identically on the same ground truth.
_PROX_ZONE_STALE_TICKS = 12


def _prox_zone_for_axis(
    anchor_deg: float,
    offsets_deg: list[float],
    axis_hits: list[bool],
    age_ticks: int,
    *,
    q_min_deg: float = -180.0,
    q_max_deg: float = 180.0,
    stale_ticks: int = _PROX_ZONE_STALE_TICKS,
) -> dict[str, Any]:
    """Collapse one axis of proximity probes into an absolute-degree zone.

    The jogging planner round-robins a fan of probes around the *current* robot
    joint angle (``anchor_deg``), spaced ``offsets_deg`` degrees away (e.g.
    ``[-10..-1, 1..10]``; never including 0). ``axis_hits`` is the parallel
    per-probe collision flag for this joint. This helper reduces that raw fan
    into the three display bands a receiver actually needs, expressed in
    **absolute joint degrees** so the receiver never has to add the joint angle
    itself:

    * a green "free" band straddling the current angle, and
    * an optional red "blocked" band above and/or below it, each ending at the
      outer edge of the tested window (the furthest probe + half a probe step).

    Anything outside the tested window is intentionally not described here; the
    receiver leaves it as the default grey to make clear no collision test was
    performed there.

    All returned edges are additionally clamped to this joint's configured hard
    limits (``q_min_deg`` / ``q_max_deg``) so a band never advertises motion the
    robot is not allowed to reach.

    Parameters
    ----------
    anchor_deg:
        Current robot joint angle in degrees (``state.full`` ``robot.q_rad``
        mapped to degrees). All returned edges are ``anchor_deg`` + a probe
        offset, so they are absolute joint degrees.
    offsets_deg:
        Shared proximity probe offsets in degrees, relative to ``anchor_deg``.
    axis_hits:
        Per-probe collision booleans for this joint, parallel to ``offsets_deg``.
    age_ticks:
        Planner ticks since this axis was last probed. Greater than
        ``stale_ticks`` marks the zone invalid (lane stays grey).
    q_min_deg, q_max_deg:
        This joint's configured hard limits in degrees (from
        ``tuning.robot.q_limits_min_deg`` / ``q_limits_max_deg``; default
        ``-180`` / ``180``). Every published edge is clamped to this range.
    stale_ticks:
        Staleness threshold; see ``_PROX_ZONE_STALE_TICKS``.

    Returns
    -------
    dict with keys:
        ``valid`` (bool)
            ``False`` -> ignore the degree fields and draw the lane grey.
        ``free_min_deg`` / ``free_max_deg`` (float | None)
            Absolute-degree edges of the green free band around the current
            angle. When a side has no collision, the band extends to the outer
            edge of the tested window on that side (clamped to the joint limit).
        ``blocked_above_till_deg`` (float | None)
            Absolute-degree top edge of the red band above the green band (the
            highest tested angle + half a probe step, clamped to ``q_max_deg``).
            ``None`` when no collision was found above (no red band above; grey
            beyond green).
        ``blocked_below_till_deg`` (float | None)
            Absolute-degree bottom edge of the red band below the green band
            (clamped to ``q_min_deg``). ``None`` when no collision was found
            below.
    """

    n = min(len(offsets_deg), len(axis_hits))
    if n == 0 or age_ticks > stale_ticks:
        return {
            "valid": False,
            "free_min_deg": None,
            "free_max_deg": None,
            "blocked_above_till_deg": None,
            "blocked_below_till_deg": None,
        }

    offsets = [float(offsets_deg[i]) for i in range(n)]
    hits = [bool(axis_hits[i]) for i in range(n)]

    # Probe spacing (degrees). Offsets are evenly spaced integers apart from the
    # skipped 0 sample, so the smallest non-zero gap is the true step. Half a
    # step is how far each probe's tested coverage reaches past its centre.
    diffs = [
        abs(offsets[i + 1] - offsets[i])
        for i in range(n - 1)
        if abs(offsets[i + 1] - offsets[i]) > 1e-9
    ]
    half_step = (min(diffs) if diffs else 1.0) * 0.5

    # Outer edges of the tested window in absolute degrees. Grey lives beyond.
    tested_lo_deg = anchor_deg + min(offsets) - half_step
    tested_hi_deg = anchor_deg + max(offsets) + half_step

    # Nearest collision on each side of the current angle (offset 0). Everything
    # from there to the tested edge is collapsed into a single red band, which
    # matches both the operator's mental "red-green-red" model and the haptic
    # dynamic-bounds logic that clamps at the nearest hit.
    neg_hit = max((o for o, h in zip(offsets, hits) if h and o < 0.0), default=None)
    pos_hit = min((o for o, h in zip(offsets, hits) if h and o > 0.0), default=None)

    if pos_hit is not None:
        free_max_deg = anchor_deg + pos_hit - half_step
        blocked_above_till_deg = tested_hi_deg
    else:
        free_max_deg = tested_hi_deg
        blocked_above_till_deg = None

    if neg_hit is not None:
        free_min_deg = anchor_deg + neg_hit + half_step
        blocked_below_till_deg = tested_lo_deg
    else:
        free_min_deg = tested_lo_deg
        blocked_below_till_deg = None

    # Clamp every edge to the joint's configured hard limits: the probe window
    # can reach past q_min/q_max, but the robot cannot, so a band must never
    # claim space outside the allowed travel.
    lo_lim = min(q_min_deg, q_max_deg)
    hi_lim = max(q_min_deg, q_max_deg)

    def _clamp(value: float | None) -> float | None:
        if value is None:
            return None
        return max(lo_lim, min(hi_lim, value))

    return {
        "valid": True,
        "free_min_deg": _clamp(free_min_deg),
        "free_max_deg": _clamp(free_max_deg),
        "blocked_above_till_deg": _clamp(blocked_above_till_deg),
        "blocked_below_till_deg": _clamp(blocked_below_till_deg),
    }


def _prox_zones_payload(
    team_state: dict[str, Any],
    joint_limits_deg: tuple[list[float], list[float]] | None = None,
) -> list[dict[str, Any]]:
    """Build the per-axis absolute-degree proximity zones for one team.

    Reads the planner's raw proximity fan from ``team_state`` and anchors each
    axis on the published robot joint angle (``last_q``), returning the six
    display-ready zone dicts described by :func:`_prox_zone_for_axis`.

    Parameters
    ----------
    team_state:
        Per-team controller state holding ``last_q`` plus the raw proximity fan
        (``last_prox_probe_offsets_deg`` / ``last_prox_hits`` /
        ``last_prox_age_ticks``).
    joint_limits_deg:
        Optional ``(q_min_deg, q_max_deg)`` per-axis hard limits in degrees used
        to clamp the published band edges. Defaults to +/-180 per axis when not
        supplied (e.g. in unit tests).
    """

    q_min_deg, q_max_deg = (
        joint_limits_deg if joint_limits_deg is not None else ([-180.0] * 6, [180.0] * 6)
    )

    q_rad = team_state.get("last_q")
    offsets_deg = list(team_state.get("last_prox_probe_offsets_deg") or [])
    prox_hits = team_state.get("last_prox_hits") or []
    age_ticks = team_state.get("last_prox_age_ticks") or []

    zones: list[dict[str, Any]] = []
    for axis in range(6):
        anchor_deg = (
            math.degrees(float(q_rad[axis]))
            if isinstance(q_rad, list) and axis < len(q_rad)
            else 0.0
        )
        axis_hits = prox_hits[axis] if axis < len(prox_hits) else []
        if not isinstance(axis_hits, list):
            axis_hits = []
        age = int(age_ticks[axis]) if axis < len(age_ticks) else 9999
        zones.append(
            _prox_zone_for_axis(
                anchor_deg,
                offsets_deg,
                axis_hits,
                age,
                q_min_deg=q_min_deg[axis] if axis < len(q_min_deg) else -180.0,
                q_max_deg=q_max_deg[axis] if axis < len(q_max_deg) else 180.0,
            )
        )
    return zones


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
    joint_limits_deg: tuple[list[float], list[float]] | None = None,
) -> dict[str, Any]:
    """Build the full `state.full` payload body for one controller tick.

    ``joint_limits_deg`` is the optional ``(q_min_deg, q_max_deg)`` per-axis hard
    limit pair used to clamp the published proximity zones to the robot's
    allowed travel; it defaults to +/-180 per axis when omitted.
    """

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
            team: _team_state_full_payload(st, haptic_cfg, joint_limits_deg)
            for team, st in teams.items()
        },
    }


def _team_state_full_payload(
    team_state: dict[str, Any],
    haptic_cfg: dict[str, Any],
    joint_limits_deg: tuple[list[float], list[float]] | None = None,
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
        "shortcut": {
            "enabled": False,
            "status": "idle",
            "seed": None,
            "original_point_count": 0,
            "shortened_point_count": 0,
            "attempts": 0,
            "collision_free_candidates": 0,
            "accepted_shortcuts": 0,
            "collision_rejections": 0,
            "configurations_sent": 0,
            "completed_configurations": 0,
            "elapsed_s": 0.0,
            "original_duration_s": None,
            "shortened_duration_s": None,
            "error": None,
        },
    }

    dial_pos_rad = list(team_state.get("last_dial") or [])
    gear_ratio = list(haptic_cfg.get("gear_ratio") or [])

    return {
        "robot": {
            "q_target_rad": team_state["last_target"],
            "q_rad": (
                team_state["last_q"] if team_state["last_q"] is not None else [0.0] * 6
            ),
            "status": team_state.get("robot_status", {}),
        },
        "haptic": {
            "dial_pos_rad": dial_pos_rad,
            # Per-controller dial angle in degrees (A1-A6). Single source of
            # truth for both the dashboard and the light columns so they never
            # disagree on a unit conversion.
            "dial_deg": [
                math.degrees(float(v)) for v in dial_pos_rad
            ],
            # Dial angle mapped through the per-axis gear ratio into the
            # equivalent robot-joint angle in degrees. This keeps the raw
            # dial-space telemetry above while also publishing a display-ready
            # robot-space interpretation that respects direction flips.
            "dial_robot_deg": [
                math.degrees(
                    float(dial_pos_rad[i])
                    * (
                        float(gear_ratio[i])
                        if i < len(gear_ratio) and abs(float(gear_ratio[i])) > 1e-9
                        else 1.0
                    )
                )
                for i in range(len(dial_pos_rad))
            ],
            "dial_vel_rad_s": team_state["last_dial_vel"],
            "connected": team_state["last_haptic_connected"],
            "board_loop_hz": team_state["last_haptic_loop_hz"],
            # Per-player tutorial scroll progress (0..100%). Only meaningful
            # during the tutorial stage; held at its last value otherwise.
            "tutorial_progress_pct": [
                float(v) for v in (team_state.get("tutorial_progress") or [0.0] * 6)
            ],
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
            # Display-ready, absolute-degree red/green/grey bands per joint.
            # This is the recommended field for any proximity visualization:
            # the receiver draws green (free) and red (blocked) bands directly
            # and leaves everything else grey (untested). The raw probe fields
            # above remain for diagnostics and back-compat.
            "prox_zones": _prox_zones_payload(team_state, joint_limits_deg),
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
        # Practice sub-state of the play stage. ``in_practice`` is True while this
        # team is still running the one-player-at-a-time warm-up; the display in
        # front of each player uses ``active_player`` (1..6, null once done) to
        # show whose turn it is, ``completed`` for per-joint progress, and the
        # static ``target_pose_deg`` to show the goal each joint jogs toward.
        "practice": {
            "in_practice": bool(team_state.get("in_practice", False)),
            "active_player": (
                int(team_state.get("practice_player", 1))
                if bool(team_state.get("in_practice", False))
                else None
            ),
            "active_joint_index": (
                int(team_state.get("practice_player", 1)) - 1
                if bool(team_state.get("in_practice", False))
                else None
            ),
            "completed": [
                bool(v)
                for v in (team_state.get("practice_completed") or [False] * 6)
            ],
            "target_pose_deg": [
                float(v)
                for v in (team_state.get("practice_target_pose_deg") or [0.0] * 6)
            ],
            "arrival_tolerance_deg": float(
                team_state.get("practice_arrival_tolerance_deg", 0.5)
            ),
        },
    }
