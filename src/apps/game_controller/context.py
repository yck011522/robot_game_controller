"""game_controller shared context: constants + profile/config construction.

This is the base layer of the game_controller package. It holds the values
and pure builders that the rest of the package is constructed from, with no
dependency on the runtime loop, sockets, or hardware:

* the shared module-level constants (team bucket wiring, default poses /
  bounds, stage order), and
* the config coercion helpers that turn a profile's ``tuning.*`` blocks and
  the ``robot_show_poses.yaml`` file into validated plain-dict configs.

Layering: ``context`` (here) <- ``stages`` <- ``__main__``. ``context`` must
never import from ``stages`` or ``__main__`` so the package stays
cycle-free. ``__main__`` and ``stages`` import the names they need from here.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

# Repo ``src`` directory (this file is src/apps/game_controller/context.py).
# Used to locate the sibling ``config/`` directory for show poses.
_SRC = Path(__file__).resolve().parents[2]

# --- Shared constants -------------------------------------------------------

# Physical bucket IDs per team. Indexed by team key ("a"/"b"); used to label
# state.full buckets and address the bucket_controller.
TEAM_BUCKET_IDS = {
    "a": [11, 12, 13],
    "b": [21, 22, 23],
}
# Fallback per-team bucket fill values (3 buckets) when a profile / weight
# source provides none. Tune via tuning.game.sim_bucket_values.
DEFAULT_BUCKET_VALUES = [0.0, 0.0, 0.0]
# Fallback robot "look" pose (deg, 6 joints) for conclusion motions when a
# named pose is missing from robot_show_poses.yaml.
DEFAULT_LOOK_POSE_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
# Default static assistive-haptic bounds (deg, 6 joints) when a profile's
# haptic block omits bounds_deg_min / bounds_deg_max.
DEFAULT_HAPTIC_BOUNDS_DEG_MIN = [-180.0] * 6
DEFAULT_HAPTIC_BOUNDS_DEG_MAX = [180.0] * 6
# Max age (ms) of a telem.safety sample before the local barrier cache is
# treated as stale (and the game pauses). Overridable per runtime config.
DEFAULT_SAFETY_TELEM_AGE_MAX_MS = 1100.0

# High-level game lifecycle, in order. `daydreaming <-> idle` is the only
# two-way edge; the rest advance one way and loop back to idle after
# conclusion. See docs/GAME_MECHANICS.md section 4 for the full description.
STAGE_ORDER = ("daydreaming", "idle", "tutorial", "play", "reset", "conclusion")


# --- Config coercion / construction ----------------------------------------


def _coerce_positive_float(value: Any, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if number > 0.0 else default


def _coerce_float_seq(value: Any) -> list[float]:
    """Coerce a JSON list into a plain float list of arbitrary length."""
    if not isinstance(value, list):
        return []
    out: list[float] = []
    for item in value:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            continue
    return out


def _coerce_start_stage(value: Any, fallback: Any) -> str:
    """Pick a valid boot stage from start_stage, then force_stage, else play."""
    for candidate in (value, fallback):
        if isinstance(candidate, str) and candidate in STAGE_ORDER:
            return candidate
    return "play"


def _coerce_bucket_value_list(value: Any) -> list[int]:
    if not isinstance(value, list):
        return [0, 0, 0]
    out: list[int] = []
    for item in value[:3]:
        try:
            out.append(max(0, int(round(float(item)))))
        except (TypeError, ValueError):
            out.append(0)
    if len(out) < 3:
        out.extend([0] * (3 - len(out)))
    return out[:3]


def _coerce_team_bucket_values(value: Any) -> dict[str, list[int]]:
    if not isinstance(value, dict):
        return {}
    out: dict[str, list[int]] = {}
    for team, buckets in value.items():
        if not isinstance(team, str):
            continue
        out[team] = _coerce_bucket_value_list(buckets)
    return out


def _game_config(node: Any) -> dict[str, Any]:
    data = node if isinstance(node, dict) else {}
    return {
        "duration_s": _coerce_positive_float(data.get("duration_s"), 240.0),
        "sum_score_rate_unit_per_s": _coerce_positive_float(
            data.get("sum_score_rate_unit_per_s"), 100.0
        ),
        "sim_bucket_values": _coerce_team_bucket_values(data.get("sim_bucket_values")),
        # --- State machine timing / thresholds (P4 bring-up) ---
        # tutorial_duration_s: tutorial countdown length (seconds).
        "tutorial_duration_s": _coerce_positive_float(
            data.get("tutorial_duration_s"), 30.0
        ),
        # reset_duration_s: placeholder hold while the robots "return to
        # start", until the real return-to-start motion + arrived signal
        # exist. Tune higher than the slowest expected homing move.
        "reset_duration_s": _coerce_positive_float(data.get("reset_duration_s"), 3.0),
        # idle_timeout_s: idle -> daydreaming if no significant dial movement
        # for this long (seconds).
        "idle_timeout_s": _coerce_positive_float(data.get("idle_timeout_s"), 60.0),
        # daydream_to_idle_dial_deg: dial-space degrees of movement (on any
        # one dial) that wakes daydreaming -> idle. Lower = more sensitive.
        "daydream_to_idle_dial_deg": _coerce_positive_float(
            data.get("daydream_to_idle_dial_deg"), 30.0
        ),
        # idle_to_tutorial_dial_deg: dial-space degrees (on any one dial)
        # that start the tutorial from idle (the "scroll up" gesture).
        "idle_to_tutorial_dial_deg": _coerce_positive_float(
            data.get("idle_to_tutorial_dial_deg"), 360.0
        ),
        # tutorial_scroll_max_deg / tutorial_detents_deg: reserved for the
        # tutorial scroll-with-detents haptic feel; parsed now, wired later.
        "tutorial_scroll_max_deg": _coerce_positive_float(
            data.get("tutorial_scroll_max_deg"), 3600.0
        ),
        "tutorial_detents_deg": _coerce_float_seq(data.get("tutorial_detents_deg")),
        # movement_arm_quiet_deg / movement_arm_quiet_ticks: gate before
        # movement detection is "armed" in daydreaming / idle. After startup
        # alignment finishes, the dials must stay still (max per-tick change
        # <= quiet_deg, in dial-space degrees) for quiet_ticks consecutive
        # ticks before the baseline is captured. This prevents the startup
        # digital-reseat settle from being mistaken for a player turning the
        # dial. quiet_ticks is in game-loop ticks (~60/s); raise either value
        # if a clean boot still false-triggers a wake.
        "movement_arm_quiet_deg": _coerce_positive_float(
            data.get("movement_arm_quiet_deg"), 2.0
        ),
        "movement_arm_quiet_ticks": int(
            _coerce_positive_float(data.get("movement_arm_quiet_ticks"), 30.0)
        ),
        # start_stage: boot stage. Falls back to the legacy force_stage key,
        # then to "play" so existing profiles keep their current behavior.
        "start_stage": _coerce_start_stage(
            data.get("start_stage"), data.get("force_stage")
        ),
    }


# --- Robot show-pose loading -----------------------------------------------


def _coerce_deg_pose(value: Any, fallback: list[float]) -> list[float]:
    if not isinstance(value, list):
        return list(fallback)
    out: list[float] = []
    for item in value[:6]:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            out.append(0.0)
    if len(out) < 6:
        out.extend(fallback[len(out) : 6])
    return out[:6]


def _default_pose_map() -> dict[str, list[float]]:
    return {
        "robot_lookb1_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lookb2_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lookb3_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_celebration_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_announcement_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_win_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lose_pose": list(DEFAULT_LOOK_POSE_DEG),
    }


def _load_robot_show_poses_deg() -> dict[str, dict[str, list[float]]]:
    default = {team: _default_pose_map() for team in TEAM_BUCKET_IDS}
    path = _SRC.parent / "config" / "robot_show_poses.yaml"
    if not path.exists():
        return default
    try:
        body = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return default
    teams = body.get("teams") if isinstance(body, dict) else None
    if not isinstance(teams, dict):
        return default
    out: dict[str, dict[str, list[float]]] = {}
    for team, fallback in default.items():
        node = teams.get(team)
        if not isinstance(node, dict):
            out[team] = fallback
            continue
        out[team] = {
            name: _coerce_deg_pose(node.get(name), fallback[name]) for name in fallback
        }
    return out


# --- Runtime state predicates ----------------------------------------------


def _startup_alignment_active(state: dict[str, Any]) -> bool:
    align = state.get("startup_align") if isinstance(state, dict) else None
    if not isinstance(align, dict):
        return False
    return bool(align.get("enabled", False)) and not bool(align.get("done", False))
