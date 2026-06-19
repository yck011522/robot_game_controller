"""Haptic runtime helpers for the game controller."""

from __future__ import annotations

import math
from typing import Any

import zmq

from core import bus

from apps.game_controller.context import (
    DEFAULT_HAPTIC_BOUNDS_DEG_MAX,
    DEFAULT_HAPTIC_BOUNDS_DEG_MIN,
    _coerce_positive_float,
)


def _update_haptic_state(state: dict[str, Any], body: dict[str, Any]) -> None:
    """Cache the latest haptic dial telemetry and connection health."""

    dial_pos = body.get("dial_pos_rad")
    state["last_dial"] = (
        dial_pos if isinstance(dial_pos, list) else state["last_dial"]
    )
    if isinstance(dial_pos, list) and len(dial_pos) >= 6:
        state["haptic_seeded"] = True
    dial_vel = body.get("dial_vel_rad_s")
    if isinstance(dial_vel, list):
        state["last_dial_vel"] = [float(v) for v in dial_vel[:6]] + [0.0] * max(
            0, 6 - len(dial_vel[:6])
        )
        state["last_dial_vel"] = state["last_dial_vel"][:6]
    connected = body.get("board_connected")
    if isinstance(connected, list):
        state["last_haptic_connected"] = [bool(v) for v in connected[:6]] + [
            False
        ] * max(0, 6 - len(connected[:6]))
        state["last_haptic_connected"] = state["last_haptic_connected"][:6]
    loop_hz = body.get("board_loop_hz")
    if isinstance(loop_hz, list):
        state["last_haptic_loop_hz"] = [float(v) for v in loop_hz[:6]] + [0.0] * max(
            0, 6 - len(loop_hz[:6])
        )
        state["last_haptic_loop_hz"] = state["last_haptic_loop_hz"][:6]


def _publish_hold_current_pose(
    pub: zmq.Socket, producer: str, team: str, state: dict[str, Any]
) -> None:
    """Publish a hold-at-actual robot command while required input is absent."""

    q_actual = state.get("last_q")
    if not isinstance(q_actual, list) or len(q_actual) < 6:
        return
    _reset_team_motion_outputs(state, q_target_rad=list(q_actual[:6]))

    env = bus.make_envelope(producer)
    env.update(
        {
            "team": team,
            "q_target_rad": list(q_actual[:6]),
            "clamps": {
                "path": 1.0,
                "prox": 1.0,
                "final": 1.0,
            },
        }
    )
    bus.publish(pub, f"cmd.robot.target.{team}", env)


def _reset_team_motion_outputs(
    state: dict[str, Any], *, q_target_rad: list[float] | None
) -> None:
    """Clear transient planner/collision outputs and optionally hold a target.

    Called by the non-planning branches in the tick loop so startup alignment,
    pause/fault handling, and non-play stages all reset the same runtime fields.
    """

    state["last_target"] = (
        list(q_target_rad[:6]) if isinstance(q_target_rad, list) else None
    )
    state["last_collision"] = False
    state["last_first_hit"] = None
    state["last_path_scalar"] = 1.0
    state["last_prox_scalar"] = 1.0
    state["last_final_scalar"] = 1.0
    state["last_planner_info"] = {}
    state["last_prox_probe_offsets_deg"] = []
    state["last_prox_hits"] = [[False] * 20 for _ in range(6)]
    state["last_prox_age_ticks"] = [9999] * 6


def _haptic_config(node: Any) -> dict[str, Any]:
    """Load haptic tuning and convert profile bounds into dial-space values."""

    data = node if isinstance(node, dict) else {}
    gear_ratio = _coerce_float_list(data.get("gear_ratio"), [1.0] * 6)
    gear = [(v if abs(v) > 1e-9 else 1.0) for v in gear_ratio]
    bounds_min_robot_rad = [
        math.radians(v)
        for v in _coerce_float_list(
            data.get("bounds_deg_min"), DEFAULT_HAPTIC_BOUNDS_DEG_MIN
        )
    ]
    bounds_max_robot_rad = [
        math.radians(v)
        for v in _coerce_float_list(
            data.get("bounds_deg_max"), DEFAULT_HAPTIC_BOUNDS_DEG_MAX
        )
    ]
    bounds_min_dial_rad, bounds_max_dial_rad = _robot_bounds_to_dial_bounds_rad(
        bounds_min_robot_rad, bounds_max_robot_rad, gear
    )
    return {
        "gear_ratio": gear,
        "bounds_min_rad": bounds_min_dial_rad,
        "bounds_max_rad": bounds_max_dial_rad,
        "prox_bounds_stale_ticks": max(
            1, int(_coerce_positive_float(data.get("prox_bounds_stale_ticks"), 12.0))
        ),
        "startup_settle_tol_rad": math.radians(
            _coerce_positive_float(data.get("startup_settle_tolerance_deg"), 10.0)
        ),
        "startup_reseat_timeout_s": _coerce_positive_float(
            data.get("startup_reseat_timeout_s"), 1.0
        ),
        "startup_settle_streak_ticks": max(
            1,
            int(_coerce_positive_float(data.get("startup_settle_streak_ticks"), 3.0)),
        ),
    }


def _publish_haptic_command(
    pub: zmq.Socket,
    producer: str,
    team: str,
    state: dict[str, Any],
    haptic_cfg: dict[str, Any],
) -> None:
    """Publish one assistive haptic command using the current target and bounds."""

    gear = list(haptic_cfg.get("gear_ratio", [1.0] * 6))
    while len(gear) < 6:
        gear.append(1.0)
    tracking_target_dial_rad = [
        float(state["last_q"][i]) / float(gear[i]) for i in range(6)
    ]
    bounds_min_rad = state.get("current_haptic_bounds_min_rad")
    bounds_max_rad = state.get("current_haptic_bounds_max_rad")
    if not isinstance(bounds_min_rad, list) or len(bounds_min_rad) < 6:
        bounds_min_rad = list(haptic_cfg["bounds_min_rad"])
    if not isinstance(bounds_max_rad, list) or len(bounds_max_rad) < 6:
        bounds_max_rad = list(haptic_cfg["bounds_max_rad"])
    env = bus.make_envelope(producer)
    env.update(
        {
            "team": team,
            "tracking_target_rad": tracking_target_dial_rad,
            "bounds_min_rad": [float(v) for v in bounds_min_rad[:6]],
            "bounds_max_rad": [float(v) for v in bounds_max_rad[:6]],
        }
    )
    bus.publish(pub, f"cmd.haptic.{team}", env)


def _reset_haptic_bounds_to_static(
    state: dict[str, Any], haptic_cfg: dict[str, Any]
) -> None:
    """Set current assistive haptic bounds to the static profile defaults."""

    state["current_haptic_bounds_min_rad"] = [
        float(v) for v in haptic_cfg.get("bounds_min_rad", [-math.pi] * 6)[:6]
    ]
    state["current_haptic_bounds_max_rad"] = [
        float(v) for v in haptic_cfg.get("bounds_max_rad", [math.pi] * 6)[:6]
    ]


def _update_dynamic_haptic_bounds_from_prox(
    state: dict[str, Any], haptic_cfg: dict[str, Any]
) -> None:
    """Update current haptic bounds from proximity-hit masks (assistive only).

    Proximity checks are sampled in robot-joint space around the current pose.
    This function converts nearest-hit offsets into dial-space bounds and
    falls back to static bounds when axis data is stale or malformed.
    """

    static_min = [
        float(v) for v in haptic_cfg.get("bounds_min_rad", [-math.pi] * 6)[:6]
    ]
    static_max = [
        float(v) for v in haptic_cfg.get("bounds_max_rad", [math.pi] * 6)[:6]
    ]
    while len(static_min) < 6:
        static_min.append(-math.pi)
    while len(static_max) < 6:
        static_max.append(math.pi)

    q_robot = state.get("last_q")
    offsets_deg = state.get("last_prox_probe_offsets_deg")
    prox_hits = state.get("last_prox_hits")
    prox_age_ticks = state.get("last_prox_age_ticks")
    if not isinstance(q_robot, list) or len(q_robot) < 6:
        _reset_haptic_bounds_to_static(state, haptic_cfg)
        return
    if not isinstance(offsets_deg, list) or not offsets_deg:
        _reset_haptic_bounds_to_static(state, haptic_cfg)
        return
    if not isinstance(prox_hits, list) or len(prox_hits) < 6:
        _reset_haptic_bounds_to_static(state, haptic_cfg)
        return
    if not isinstance(prox_age_ticks, list) or len(prox_age_ticks) < 6:
        _reset_haptic_bounds_to_static(state, haptic_cfg)
        return

    gear = [float(v) for v in haptic_cfg.get("gear_ratio", [1.0] * 6)[:6]]
    while len(gear) < 6:
        gear.append(1.0)

    offsets_rad: list[float] = []
    for value in offsets_deg:
        try:
            offsets_rad.append(math.radians(float(value)))
        except (TypeError, ValueError):
            _reset_haptic_bounds_to_static(state, haptic_cfg)
            return

    stale_ticks = int(haptic_cfg.get("prox_bounds_stale_ticks", 12) or 12)
    stale_ticks = max(1, stale_ticks)

    out_min: list[float] = []
    out_max: list[float] = []
    for axis in range(6):
        lo_static = float(static_min[axis])
        hi_static = float(static_max[axis])
        if lo_static > hi_static:
            lo_static, hi_static = hi_static, lo_static

        age = prox_age_ticks[axis]
        axis_hits = prox_hits[axis]
        if (not isinstance(age, int) and not isinstance(age, float)) or float(
            age
        ) > float(stale_ticks):
            out_min.append(lo_static)
            out_max.append(hi_static)
            continue
        if not isinstance(axis_hits, list) or len(axis_hits) != len(offsets_rad):
            out_min.append(lo_static)
            out_max.append(hi_static)
            continue

        q_dial = _robot_to_dial_rad(float(q_robot[axis]), float(gear[axis]))
        neg_hit_dial: float | None = None
        pos_hit_dial: float | None = None
        for off_rad, hit in zip(offsets_rad, axis_hits):
            if not bool(hit):
                continue
            off_dial = _robot_to_dial_rad(float(off_rad), float(gear[axis]))
            if off_dial < 0.0 and (neg_hit_dial is None or off_dial > neg_hit_dial):
                neg_hit_dial = off_dial
            if off_dial > 0.0 and (pos_hit_dial is None or off_dial < pos_hit_dial):
                pos_hit_dial = off_dial

        min_dial = (
            lo_static
            if neg_hit_dial is None
            else _clamp(q_dial + neg_hit_dial, lo_static, hi_static)
        )
        max_dial = (
            hi_static
            if pos_hit_dial is None
            else _clamp(q_dial + pos_hit_dial, lo_static, hi_static)
        )
        if min_dial > max_dial:
            min_dial, max_dial = lo_static, hi_static
        out_min.append(float(min_dial))
        out_max.append(float(max_dial))

    state["current_haptic_bounds_min_rad"] = out_min
    state["current_haptic_bounds_max_rad"] = out_max


def _robot_to_dial_rad(robot_rad: float, gear_ratio: float) -> float:
    """Convert one robot-joint angle into dial-space using the axis gear ratio."""

    ratio = float(gear_ratio)
    if abs(ratio) < 1e-9:
        ratio = 1.0
    return float(robot_rad) / ratio


def _robot_bounds_to_dial_bounds_rad(
    bounds_min_robot_rad: list[float],
    bounds_max_robot_rad: list[float],
    gear_ratio: list[float],
) -> tuple[list[float], list[float]]:
    """Convert profile robot-joint bounds into dial-space firmware bounds.

    Called by `_haptic_config` while loading profile tuning. The haptic
    firmware receives dial-space `C,<target>,<min>,<max>` values, while the
    profile stores bounds in robot-joint degrees beside the robot limits.
    """

    out_min: list[float] = []
    out_max: list[float] = []
    for axis in range(6):
        gear = float(gear_ratio[axis]) if axis < len(gear_ratio) else 1.0
        lo_robot = float(bounds_min_robot_rad[axis])
        hi_robot = float(bounds_max_robot_rad[axis])
        lo_dial = _robot_to_dial_rad(lo_robot, gear)
        hi_dial = _robot_to_dial_rad(hi_robot, gear)
        out_min.append(min(lo_dial, hi_dial))
        out_max.append(max(lo_dial, hi_dial))
    return out_min, out_max


def _publish_haptic_reseat(
    pub: zmq.Socket,
    producer: str,
    team: str,
    *,
    current_pos_robot_rad: list[float],
    current_pos_dial_rad: list[float],
) -> None:
    """Request a one-shot haptic reseat using the latest robot/dial positions."""

    env = bus.make_envelope(producer)
    env.update(
        {
            "team": team,
            "current_pos_rad": list(current_pos_robot_rad),
            "current_pos_dial_rad": list(current_pos_dial_rad),
        }
    )
    bus.publish(pub, f"cmd.haptic.reseat.{team}", env)


def _tick_startup_alignment(
    pub: zmq.Socket,
    producer: str,
    team: str,
    state: dict[str, Any],
    haptic_cfg: dict[str, Any],
    *,
    now: float,
) -> None:
    """Drive the startup reseat loop until the haptic boards track the robot."""

    align = (
        state.get("startup_align")
        if isinstance(state.get("startup_align"), dict)
        else {}
    )
    q_robot = list(state.get("last_q") or [0.0] * 6)[:6]
    gear = list(haptic_cfg.get("gear_ratio", [1.0] * 6))[:6]
    while len(gear) < 6:
        gear.append(1.0)
    q_dial = [
        float(q_robot[i]) / (float(gear[i]) if abs(float(gear[i])) > 1e-9 else 1.0)
        for i in range(6)
    ]

    settled, max_err = _startup_alignment_is_settled(state, q_dial, haptic_cfg)
    if settled:
        align["settled_streak"] = int(align.get("settled_streak", 0)) + 1
    else:
        align["settled_streak"] = 0

    if int(align.get("settled_streak", 0)) >= int(
        haptic_cfg.get("startup_settle_streak_ticks", 3)
    ):
        align["done"] = True
        print(
            f"[game_controller] startup-align done team={team} attempts={int(align.get('attempts', 0))} max_err_rad={max_err:.4f}",
            flush=True,
        )
        return

    attempts = int(align.get("attempts", 0))
    last_send = float(align.get("last_reseat_mono_s", 0.0) or 0.0)
    timeout_s = float(haptic_cfg.get("startup_reseat_timeout_s", 1.0))
    should_send = attempts == 0 or ((now - last_send) >= timeout_s)
    if not should_send:
        return

    _publish_haptic_reseat(
        pub,
        producer,
        team,
        current_pos_robot_rad=q_robot,
        current_pos_dial_rad=q_dial,
    )
    align["attempts"] = attempts + 1
    align["last_reseat_mono_s"] = now
    align["settled_streak"] = 0
    print(
        f"[game_controller] startup-align reseat team={team} attempt={align['attempts']} "
        f"j6_robot={q_robot[5]:.4f} j6_dial={q_dial[5]:.4f} max_err_rad={max_err:.4f}",
        flush=True,
    )


def _startup_alignment_is_settled(
    state: dict[str, Any], target_dial_rad: list[float], haptic_cfg: dict[str, Any]
) -> tuple[bool, float]:
    """Return whether startup-alignment has all six dials on target.

    This is stricter than normal tracking: every dial must report connected and
    the maximum dial-to-target error must be within the configured settle limit.
    """

    dial = list(state.get("last_dial") or [0.0] * 6)[:6]
    conn = list(state.get("last_haptic_connected") or [False] * 6)[:6]
    while len(dial) < 6:
        dial.append(0.0)
    while len(conn) < 6:
        conn.append(False)

    if not all(bool(v) for v in conn[:6]):
        return False, float("inf")

    tol = float(haptic_cfg.get("startup_settle_tol_rad", math.radians(10.0)))
    max_err = 0.0
    for i in range(6):
        err = abs(float(dial[i]) - float(target_dial_rad[i]))
        if err > max_err:
            max_err = err
    return max_err <= tol, max_err


def _coerce_float_list(value: Any, fallback: list[float]) -> list[float]:
    """Coerce up to six numeric items, falling back element-wise on errors."""

    if not isinstance(value, list):
        return list(fallback)
    out: list[float] = []
    for idx, item in enumerate(value[:6]):
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            out.append(float(fallback[idx]))
    if len(out) < 6:
        out.extend(float(v) for v in fallback[len(out) : 6])
    return out[:6]


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp one scalar to the inclusive [lo, hi] range."""

    return lo if value < lo else hi if value > hi else value