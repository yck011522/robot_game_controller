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

# High-level game lifecycle, in order. `daydreaming -> daydream_interrupted ->
# idle` is the attract-mode path; the rest advance one way and loop back to
# idle after conclusion. See docs/GAME_MECHANICS.md section 4 for the full
# description.
STAGE_ORDER = (
    "daydreaming",
    "daydream_interrupted",
    "idle",
    "tutorial",
    "play",
    "reset",
    "conclusion",
)


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


def _coerce_tutorial_scroll(start_end: Any, bound: Any) -> dict[str, float]:
    """Normalize the tutorial scroll span + soft bounds (deci-degrees).

    ``start_end`` is a ``[start, end]`` pair the player scrolls through; ``end``
    is normally negative so scrolling toward it advances progress. ``bound`` is
    a ``[min, max]`` soft-limit pair (typically a touch wider than start/end so
    both endpoints are reachable). Missing / malformed values fall back to the
    reference defaults: span ``0 -> -10000`` and bounds ``-10010 .. 10``.
    """

    span = _coerce_float_seq(start_end)
    start = span[0] if len(span) >= 1 else 0.0
    end = span[1] if len(span) >= 2 else -10000.0
    limits = _coerce_float_seq(bound)
    bmin = limits[0] if len(limits) >= 1 else -10010.0
    bmax = limits[1] if len(limits) >= 2 else 10.0
    if bmin > bmax:  # tolerate a swapped [max, min] pair
        bmin, bmax = bmax, bmin
    return {
        "tutorial_scroll_dial_start_decideg": start,
        "tutorial_scroll_dial_end_decideg": end,
        "tutorial_scroll_dial_bound_min_decideg": bmin,
        "tutorial_scroll_dial_bound_max_decideg": bmax,
    }


def _coerce_tutorial_bound_zones(value: Any) -> list[dict[str, float]]:
    """Normalize tutorial position-triggered soft-bound zones.

    Called by :func:`_tutorial_config` for optional
    ``tutorial_scroll_dial_bound_zones`` entries. Each zone is a mapping with
    ``active_range`` (dial position interval that selects the zone) and
    ``bound`` (soft haptic bounds to publish while inside that interval), both
    expressed in dial-space deci-degrees. Invalid entries are ignored.
    """

    if not isinstance(value, list):
        return []
    zones: list[dict[str, float]] = []
    for item in value:
        if not isinstance(item, dict):
            continue
        active_range = _coerce_float_seq(item.get("active_range"))
        bound = _coerce_float_seq(item.get("bound"))
        if len(active_range) < 2 or len(bound) < 2:
            continue
        active_min = min(active_range[0], active_range[1])
        active_max = max(active_range[0], active_range[1])
        bound_min = min(bound[0], bound[1])
        bound_max = max(bound[0], bound[1])
        zones.append(
            {
                "active_min_decideg": active_min,
                "active_max_decideg": active_max,
                "bound_min_decideg": bound_min,
                "bound_max_decideg": bound_max,
            }
        )
    return zones


def _coerce_tutorial_detents_pct(value: Any) -> list[float]:
    """Coerce the tutorial detent percentages, clamped to 0..100 and sorted.

    Each entry marks a progress percentage where the dial snaps to a detent.
    Out-of-range / non-numeric entries are dropped; the reference defaults
    ``[10, 30, 40, 70, 100]`` are used when nothing valid is supplied.
    """

    out: list[float] = []
    if isinstance(value, list):
        for item in value:
            try:
                pct = float(item)
            except (TypeError, ValueError):
                continue
            if 0.0 <= pct <= 100.0:
                out.append(pct)
    if not out:
        return [10.0, 30.0, 40.0, 70.0, 100.0]
    return sorted(out)


def _tutorial_config(node: Any, haptic_node: Any = None) -> dict[str, Any]:
    """Normalize the profile's ``tuning.tutorial`` block.

    Called once by ``game_controller`` startup. The returned dictionary owns all
    tutorial-stage knobs: duration, temporary tracking gain, scroll bounds, and
    detent percentages. ``haptic_node`` supplies the normal tracking gain as the
    default so profiles can omit ``tuning.tutorial.tracking_kp`` when they want
    tutorial and play to feel identical.
    """

    data = node if isinstance(node, dict) else {}
    haptic_data = haptic_node if isinstance(haptic_node, dict) else {}
    return {
        # duration_s: tutorial countdown length (seconds).
        "duration_s": _coerce_positive_float(data.get("duration_s"), 30.0),
        # tracking_kp: temporary haptic tracking proportional gain used only
        # while the tutorial is active. Falls back to the normal/play gain.
        "tracking_kp": _coerce_positive_float(
            data.get("tracking_kp"),
            _coerce_positive_float(haptic_data.get("tracking_kp"), 10.0),
        ),
        # tutorial_scroll_dial_start_end: [start, end] dial position the player
        # scrolls through during the tutorial, in deci-degrees (decideg =
        # deg * 10). Defaults span 0 -> -10000 decideg (0 -> -1000 deg). The
        # end is negative so scrolling "down" advances; progress 0..100% maps
        # linearly start -> end. Split into two scalar keys below.
        # tutorial_scroll_dial_bound: [min, max] soft haptic bounds (decideg)
        # held constant for the whole tutorial; set slightly wider than
        # start/end so both endpoints are reachable.
        **_coerce_tutorial_scroll(
            data.get("tutorial_scroll_dial_start_end"),
            data.get("tutorial_scroll_dial_bound"),
        ),
        # tutorial_scroll_dial_bound_zones: optional per-position soft-bound
        # overrides. The first active_range containing a dial's measured
        # decidegree position wins for that dial; otherwise the default
        # tutorial_scroll_dial_bound is used.
        "tutorial_scroll_dial_bound_zones": _coerce_tutorial_bound_zones(
            data.get("tutorial_scroll_dial_bound_zones")
        ),
        # tutorial_detents_pct: progress percentages (0..100) at which the dial
        # snaps to a detent (nearest-detent tracking each tick). No 0% detent
        # means the dial pulls to the first detent the moment the tutorial
        # begins. Defaults match the reference profile.
        "tutorial_detents_pct": _coerce_tutorial_detents_pct(
            data.get("tutorial_detents_pct")
        ),
    }


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
        # reset_duration_s: placeholder hold while the robots "return to
        # start", until the real return-to-start motion + arrived signal
        # exist. Tune higher than the slowest expected homing move.
        "reset_duration_s": _coerce_positive_float(data.get("reset_duration_s"), 3.0),
        # rewind_enabled: record certified play targets and retrace them during
        # reset. False preserves the placeholder reset timer for other profiles.
        "rewind_enabled": bool(data.get("rewind_enabled", False)),
        # --- Practice sub-state (per team, inside the play stage) ---
        # practice_enabled: when true, every team starts the play stage in a
        # per-team "practice" sub-state (published as teams.<t>.practice) before
        # normal gameplay. Players 1..6 take turns jogging their own joint
        # (player N -> joint N-1) from robot_begin_pose to
        # robot_practice_target_pose while all other joints are frozen at their
        # exact begin / target angle. The shared play timer keeps running, so
        # practice consumes part of duration_s. Defaults False so existing
        # profiles keep jumping straight into gameplay; enable per profile.
        "practice_enabled": bool(data.get("practice_enabled", False)),
        # practice_arrival_tolerance_deg: the active joint counts as "arrived"
        # once its commanded target is within this absolute error (degrees) of
        # the practice target angle. Kept small so the hand-off latches the joint
        # essentially on target. Tune up if players struggle to trigger arrival.
        "practice_arrival_tolerance_deg": _coerce_positive_float(
            data.get("practice_arrival_tolerance_deg"), 0.5
        ),
        # practice_arrival_dwell_s: the active joint must stay within tolerance
        # continuously for this many seconds before the turn advances to the
        # next player. Rejects a fly-through where the joint only clips the
        # target for a single tick. Tune up for a firmer settle requirement.
        "practice_arrival_dwell_s": _coerce_positive_float(
            data.get("practice_arrival_dwell_s"), 0.4
        ),
        # rewind_speed_fraction: fraction of configured per-joint maximum
        # velocity used for geometry-only rewind retiming. Acceleration
        # retiming is intentionally deferred until the hardware workflow works.
        "rewind_speed_fraction": min(
            1.0,
            _coerce_positive_float(data.get("rewind_speed_fraction"), 0.3),
        ),
        # rewind_arrival_tolerance_deg: every measured joint must be within
        # this absolute error of the play-entry pose before reset completes.
        "rewind_arrival_tolerance_deg": _coerce_positive_float(
            data.get("rewind_arrival_tolerance_deg"), 0.5
        ),
        # conclusion_speed_fraction: fraction (0, 1] of each configured per-joint
        # maximum velocity used to retime every conclusion show move (look-at-
        # bucket, win/lose, return-to-begin). This is the single speed knob for
        # the end-of-game choreography. 0.60 = move at 60% of the per-axis max.
        # Raise for a snappier show, lower for a calmer one.
        "conclusion_speed_fraction": min(
            1.0,
            _coerce_positive_float(data.get("conclusion_speed_fraction"), 0.60),
        ),
        # conclusion_cert_budget_s: wall-clock budget for the one-shot background
        # collision certification of the fixed conclusion pose-to-pose path,
        # started on entering conclusion. Certification must finish within this
        # many seconds (it overlaps the initial pause); a team whose path is not
        # certified collision-free in time is hard-stopped (holds its pose and is
        # marked done). Tune up if the collision pool is slow / heavily loaded.
        "conclusion_cert_budget_s": _coerce_positive_float(
            data.get("conclusion_cert_budget_s"), 2.0
        ),
        # conclusion_collision_step_deg: maximum joint-space increment (degrees)
        # between collision samples when densifying each conclusion path edge for
        # certification. Smaller = finer (safer but slower) checking.
        "conclusion_collision_step_deg": _coerce_positive_float(
            data.get("conclusion_collision_step_deg"), 1.0
        ),
        # idle_timeout_s: idle -> daydreaming if no significant dial movement
        # for this long (seconds).
        "idle_timeout_s": _coerce_positive_float(data.get("idle_timeout_s"), 60.0),
        # daydream_to_idle_error_deg: wake daydreaming -> idle when any dial is
        # pushed this far OFF its commanded tracking target (dial-space deg). The
        # dials are spring-tracked to the robot (held still, or following the
        # attract-mode playback), so only a human pushing against the spring
        # deviates. With gear_ratio 0.1, 900 dial deg ~= 90 robot deg.
        "daydream_to_idle_error_deg": _coerce_positive_float(
            data.get("daydream_to_idle_error_deg"), 900.0
        ),
        # idle_to_tutorial_dial_deg: dial-space degrees (on any one dial)
        # that start the tutorial from idle (the "scroll up" gesture).
        "idle_to_tutorial_dial_deg": _coerce_positive_float(
            data.get("idle_to_tutorial_dial_deg"), 360.0
        ),
        # movement_window_s: length (seconds) of the rolling dial-history
        # window used for movement detection in daydreaming / idle. Detection
        # looks at the peak-to-peak range of each dial *within this window*
        # instead of the displacement from a single fixed baseline, so slow
        # drift around a set point rolls off the back of the window and never
        # accumulates into a false wake. Longer = more drift rejection, but a
        # genuine deliberate turn must complete within this many seconds to be
        # seen. Detection only arms once a full clean window has been collected
        # after startup alignment finishes (so a baseline can never be sampled
        # mid-reseat). Tune up if slow drift still wakes the game; tune down if
        # deliberate turns feel sluggish to register.
        "movement_window_s": _coerce_positive_float(
            data.get("movement_window_s"), 2.0
        ),
        # movement_glitch_trim: number of most-extreme samples discarded at
        # EACH end (high and low) of every dial's window before computing the
        # peak-to-peak range. This rejects brief single-/few-frame encoder
        # glitches (e.g. a one-tick 140 deg J6 spike) that would otherwise trip
        # detection, while a sustained real turn still produces a large range.
        # At ~50 Hz a 2 s window holds ~100 samples, so the default 3 tolerates
        # up to 3 glitch frames per joint. Raise if dials glitch in longer
        # bursts; set to 0 for plain peak-to-peak (no glitch rejection).
        "movement_glitch_trim": int(
            _coerce_positive_float(data.get("movement_glitch_trim"), 3.0)
        ),
        # start_stage: boot stage. Falls back to the legacy force_stage key,
        # then to "play" so existing profiles keep their current behavior.
        "start_stage": _coerce_start_stage(
            data.get("start_stage"), data.get("force_stage")
        ),
        # score_min_increment_g: weight deadband (grams). A bucket's summed
        # live load-cell reading must reach at least this many grams before it
        # counts toward the score (the on-board "counter" increments); readings
        # below it publish as 0, rejecting empty-bucket drift / noise. 0 disables
        # the deadband (every gram counts). Raise if light debris registers a
        # score; lower for finer sensitivity. Only affects real weight-sensor
        # play, not sim_bucket_values seeding.
        "score_min_increment_g": max(
            0.0, _coerce_positive_float(data.get("score_min_increment_g"), 0.0)
        ),
    }


def _daydream_config(node: Any) -> dict[str, Any]:
    """Normalize the optional ``tuning.daydream`` attract-mode playback block.

    Controls whether daydreaming replays the most recent recorded game's robot
    trajectory (forward = verbatim recording, no smoothing) and rewinds it
    (smoothed) before looping. ``per_team_own_trajectory`` true means each team
    replays its own recorded path; false means every active robot replays team
    A's path. Independent rewind speed / shortcut from the gameplay reset so the
    attract loop can be tuned calmer without changing reset behavior.
    """

    data = node if isinstance(node, dict) else {}
    return {
        "enabled": bool(data.get("enabled", False)),
        "per_team_own_trajectory": bool(data.get("per_team_own_trajectory", True)),
        "recording_dir": str(
            data.get("recording_dir") or "logs/display_broadcast_recording"
        ),
        # rewind speed/tolerance for the smoothed daydream return (separate from
        # the gameplay reset rewind knobs).
        "rewind_speed_fraction": min(
            1.0, _coerce_positive_float(data.get("rewind_speed_fraction"), 0.3)
        ),
        "rewind_arrival_tolerance_deg": _coerce_positive_float(
            data.get("rewind_arrival_tolerance_deg"), 0.5
        ),
        # Reuse the rewind-shortcut normalizer for the nested smoothing block.
        "rewind_shortcut": _rewind_shortcut_config(data.get("rewind_shortcut")),
    }


def _rewind_shortcut_config(node: Any) -> dict[str, Any]:
    """Normalize the optional geometric rewind-shortcut tuning block.

    ``random_seed: null`` selects a fresh production seed for every rewind.
    Validation profiles and tests may set an integer seed for reproducibility.
    """

    data = node if isinstance(node, dict) else {}
    raw_seed = data.get("random_seed")
    try:
        random_seed = int(raw_seed) if raw_seed is not None else None
    except (TypeError, ValueError):
        random_seed = None
    return {
        "enabled": bool(data.get("enabled", False)),
        # Wall-clock search budget. There is intentionally no iteration cap.
        "optimization_budget_s": _coerce_positive_float(
            data.get("optimization_budget_s"), 3.0
        ),
        # Maximum joint-space increment between collision samples.
        "collision_step_deg": _coerce_positive_float(
            data.get("collision_step_deg"), 1.0
        ),
        # Number of configurations placed in each collision worker request.
        "collision_batch_size": max(
            1, int(_coerce_positive_float(data.get("collision_batch_size"), 8.0))
        ),
        "random_seed": random_seed,
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
        "robot_begin_pose": list(DEFAULT_LOOK_POSE_DEG),
        # Static target the practice sub-state walks each joint to, one player at
        # a time. Loaded from robot_show_poses.yaml; falls back to the neutral
        # look pose when absent so the loader always yields six values.
        "robot_practice_target_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lookb1_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lookb2_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lookb3_pose": list(DEFAULT_LOOK_POSE_DEG),
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
