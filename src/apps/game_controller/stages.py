"""game_controller stage machine: lifecycle transitions + conclusion scoring.

This module owns the pure, hardware-free game state machine that advances the
high-level lifecycle ``daydreaming -> idle -> tutorial -> play -> reset ->
conclusion -> idle`` (``daydreaming <-> idle`` is the only two-way edge), plus
the scripted per-team conclusion scoring sequence.

These functions operate only on plain dicts (``stage_state``, ``teams``,
``game_cfg``, ``pose_cfg``) and a monotonic ``now_ns`` clock, so they are unit
tested directly without any ZeroMQ bus or robots. ``__main__`` drives them once
per tick (only while the game is not paused) and supplies the publish callback
used by the conclusion sequence.

Layering: ``context`` <- ``stages`` (here) <- ``__main__``. This module imports
shared constants / predicates from ``context`` and is imported by ``__main__``.
"""

from __future__ import annotations

import math
from typing import Any

from apps.game_controller.context import (
    DEFAULT_BUCKET_VALUES,
    DEFAULT_LOOK_POSE_DEG,
    _startup_alignment_active,
)

# --- Conclusion scoring sequence timing ------------------------------------
# Hold durations (seconds) for the non-motion phases of the conclusion show.
# Motion phases are NOT timed here: they advance when the per-team SegmentMover
# (driven in ``__main__``) reports arrival, so their duration follows the
# retimed trajectory and ``conclusion_speed_fraction`` rather than a constant.
#
# Every duration below is measured against a pause-aware clock
# (``conclusion_phase_elapsed_s``) that only accumulates ``dt`` on ticks where
# the game is actually running, so an e-stop / soft-pause freezes the show.
#
# Pause after entering conclusion, before the first bucket-look move. This also
# overlaps the background collision-certification window owned by ``__main__``.
CONCLUSION_INITIAL_PAUSE_S = 1.0
# Short pause after a bucket empties (door open) before moving to the next one.
CONCLUSION_BUCKET_EMPTY_PAUSE_S = 0.5
# Pause on the announcement pose before the winner / loser pose is resolved.
CONCLUSION_ANNOUNCEMENT_PAUSE_S = 0.5
# Hold on the winner / loser pose before returning to the begin pose.
CONCLUSION_WINNER_POSE_HOLD_S = 3.0

# Conclusion phases whose ``conclusion_phase`` value means "the winner pose has
# been (or is about to be) resolved". The stage machine computes the winner once
# every team has reached one of these, so the per-team winner/loser move can read
# ``stage_state["winner_team"]``.
_CONCLUSION_ANNOUNCEMENT_READY = frozenset(
    {
        "announcement_pause",
        "move_to_winner_pose",
        "winner_pose_hold",
        "move_to_begin",
    }
)


def _tick_stage_state(
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    game_cfg: dict[str, Any],
    now_ns: int,
) -> None:
    """Advance the high-level game state machine by one tick.

    Called only while the game is NOT paused, so every timer and every
    movement-driven transition freezes during an e-stop / pause overlay.
    Lifecycle: daydreaming -> idle -> tutorial -> play -> reset ->
    conclusion -> idle. `daydreaming <-> idle` is the only two-way edge.
    """
    stage = stage_state["stage"]

    if stage == "daydreaming":
        # Attract mode. Wake to idle as soon as any dial is turned past the
        # configured threshold. No timer here.
        _update_dial_window(stage_state, teams, game_cfg, now_ns)
        wake_rad = math.radians(float(game_cfg["daydream_to_idle_dial_deg"]))
        _maybe_log_movement_progress(
            stage_state, teams, game_cfg, wake_rad, "daydreaming", now_ns
        )
        delta, detail = _max_dial_delta_detail(stage_state, teams, game_cfg)
        if detail is not None and delta >= wake_rad:
            _enter_stage(
                stage_state, teams, "idle", game_cfg, now_ns,
                reason=_movement_reason("dial moved -> wake", detail, delta, wake_rad),
            )
        return

    if stage == "idle":
        # Ready state. A big "scroll up" starts the tutorial; otherwise a long
        # quiet period drops back to daydreaming. No countdown shown.
        _update_dial_window(stage_state, teams, game_cfg, now_ns)
        start_rad = math.radians(float(game_cfg["idle_to_tutorial_dial_deg"]))
        _maybe_log_movement_progress(
            stage_state, teams, game_cfg, start_rad, "idle", now_ns
        )
        delta, detail = _max_dial_delta_detail(stage_state, teams, game_cfg)
        if detail is not None and delta >= start_rad:
            _enter_stage(
                stage_state, teams, "tutorial", game_cfg, now_ns,
                reason=_movement_reason(
                    "dial scrolled up -> start tutorial", detail, delta, start_rad
                ),
            )
            return
        if _stage_elapsed_s(stage_state, now_ns) >= float(game_cfg["idle_timeout_s"]):
            _enter_stage(
                stage_state, teams, "daydreaming", game_cfg, now_ns,
                reason="idle timeout",
            )
        return

    if stage == "tutorial":
        # Timed; skippable; also exits early once every active player has
        # scrolled to 100% tutorial progress. Progress is computed in the
        # per-team motion loop (which runs earlier this tick) and cached on
        # each team's ``tutorial_progress``.
        skip = bool(stage_state.get("skip_requested"))
        timed_out = _stage_elapsed_s(stage_state, now_ns) >= float(
            game_cfg["tutorial_duration_s"]
        )
        all_done = _all_tutorial_progress_complete(teams)
        if skip or timed_out or all_done:
            if skip:
                reason = "skip"
            elif all_done:
                reason = "all players reached 100%"
            else:
                reason = "tutorial timer expired"
            _enter_stage(stage_state, teams, "play", game_cfg, now_ns, reason=reason)
        return

    if stage == "play":
        # The actual game. Timed; skippable (skip == set the timer to zero).
        if stage_state.get("skip_requested") or _stage_elapsed_s(
            stage_state, now_ns
        ) >= float(game_cfg["duration_s"]):
            reason = (
                "skip" if stage_state.get("skip_requested") else "game timer expired"
            )
            _enter_stage(stage_state, teams, "reset", game_cfg, now_ns, reason=reason)
        return

    if stage == "reset":
        # Enabled rewind profiles wait for measured arrival from every robot.
        # Other profiles retain the placeholder fixed timer for compatibility.
        rewind_enabled = bool(game_cfg.get("rewind_enabled", False))
        rewind_complete = bool(teams) and all(
            bool(getattr(st.get("rewind"), "complete", False))
            for st in teams.values()
        )
        timer_complete = _stage_elapsed_s(stage_state, now_ns) >= float(
            game_cfg["reset_duration_s"]
        )
        if (rewind_enabled and rewind_complete) or (
            not rewind_enabled and timer_complete
        ):
            _enter_stage(
                stage_state, teams, "conclusion", game_cfg, now_ns,
                reason=(
                    "rewind arrived at play-entry pose"
                    if rewind_enabled
                    else "robots returned to start (placeholder timer)"
                ),
            )
        return

    if stage == "conclusion":
        # Scripted scoring sequence advanced per-team in the main loop.
        # Not skippable; returns to idle once every team has finished.
        if stage_state["winner_team"] is None and all(
            bool(st.get("conclusion_done", False))
            or str(st.get("conclusion_phase")) in _CONCLUSION_ANNOUNCEMENT_READY
            for st in teams.values()
        ):
            stage_state["winner_team"] = _winner_team(teams)

        if not teams or all(
            bool(st.get("conclusion_done", False)) for st in teams.values()
        ):
            _enter_stage(
                stage_state, teams, "idle", game_cfg, now_ns,
                reason="scoring complete",
            )
        return


def _all_tutorial_progress_complete(teams: dict[str, dict]) -> bool:
    """Return True once every active team's six dials are at 100% progress.

    Reads the per-team ``tutorial_progress`` list (0..100 per dial) that the
    runtime loop refreshes each tutorial tick. Returns False when there are no
    teams or any team has not yet populated progress, so a fresh tutorial never
    exits before the players have actually scrolled. A small epsilon absorbs
    float rounding around the 100% endpoint.
    """

    if not teams:
        return False
    for st in teams.values():
        progress = st.get("tutorial_progress")
        if not isinstance(progress, list) or len(progress) < 6:
            return False
        if any(float(p) < 99.999 for p in progress[:6]):
            return False
    return True


def _enter_stage(
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    new_stage: str,
    game_cfg: dict[str, Any],
    now_ns: int,
    *,
    reason: str = "",
) -> None:
    """Transition the stage machine and run per-stage entry side effects.

    Resets the stage timer + pause accounting, clears the movement-detection
    rolling window, and any pending skip, seeds a fresh game on `play`, and
    seeds the scoring sequence on `conclusion`. Prints a one-line transition
    banner so the operator can trace the flow on the console.
    """
    old_stage = stage_state.get("stage")
    stage_state["stage"] = new_stage
    stage_state["stage_entered_mono_ns"] = now_ns
    stage_state["pause_started_mono_ns"] = None
    stage_state["paused_total_ns"] = 0
    # Rolling per-team dial-history window used by movement detection, plus the
    # per-team arming trackers. Cleared on every stage entry so the new stage
    # collects a fresh clean window before detection can fire.
    stage_state["dial_window"] = {}
    stage_state["dial_arm"] = {}
    # Throttle timestamp for the movement-progress debug readout.
    stage_state["_move_dbg_ns"] = None
    stage_state["skip_requested"] = False

    if new_stage == "play":
        stage_state["winner_team"] = None
        _seed_play_teams(teams, game_cfg)
        for st in teams.values():
            play_sync = st.get("play_sync")
            if isinstance(play_sync, dict) and bool(
                play_sync.get("enabled", False)
            ):
                play_sync["requested"] = True
                play_sync["pending"] = False
                play_sync["settled_streak"] = 0
                play_sync["attempts"] = 0
            rewind = st.get("rewind")
            if rewind is not None:
                rewind.start_recording(
                    st.get("last_q"), now_s=float(now_ns) / 1e9
                )
    elif new_stage == "tutorial":
        # Tutorial entry side effects (dict-only; the runtime loop owns the
        # actual haptic reseat + bounds publishing). Every active team starts
        # the scroll at zero progress and flags a one-shot dial reset so the
        # per-team loop reseats each dial to 0 and installs the wide tutorial
        # bounds on the first tutorial tick.
        for st in teams.values():
            st["tutorial_progress"] = [0.0] * 6
            st["tutorial_reset_pending"] = True
    elif new_stage == "reset" and bool(game_cfg.get("rewind_enabled", False)):
        for st in teams.values():
            rewind = st.get("rewind")
            if rewind is not None:
                rewind.start_rewind()
    elif new_stage == "conclusion":
        stage_state["winner_team"] = None
        for st in teams.values():
            _enter_conclusion(st, now_ns)

    print(
        f"[game_controller] STAGE {old_stage} -> {new_stage}"
        + (f"  ({reason})" if reason else ""),
        flush=True,
    )


def _seed_play_teams(teams: dict[str, dict], game_cfg: dict[str, Any]) -> None:
    """Reset each team's score / bucket / conclusion scratch for a new game."""
    sim_bucket_values = (
        game_cfg.get("sim_bucket_values") if isinstance(game_cfg, dict) else {}
    )
    if not isinstance(sim_bucket_values, dict):
        sim_bucket_values = {}

    for team, st in teams.items():
        seed_buckets = sim_bucket_values.get(team, DEFAULT_BUCKET_VALUES)
        st["bucket_values"] = list(seed_buckets)
        st["score"] = int(sum(st["bucket_values"]))
        st["summed_score"] = 0
        st["conclusion_phase"] = None
        st["conclusion_active_bucket_index"] = None
        st["conclusion_target_pose_name"] = None
        st["conclusion_target_pose_deg"] = None
        st["conclusion_bucket_open_triggered"] = False
        st["conclusion_phase_elapsed_s"] = 0.0
        st["conclusion_move_pending"] = False
        st["conclusion_move_arrived"] = False
        st["conclusion_hardstopped"] = False
        st["conclusion_done"] = False
        st["conclusion_sum_remainder_units"] = 0.0


def _update_dial_window(
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    game_cfg: dict[str, Any],
    now_ns: int,
) -> None:
    """Append the current dial positions to each team's rolling history window.

    Movement detection no longer uses a single fixed baseline. Instead, every
    tick we record ``(now_ns, dial[6])`` into a per-team ring kept in
    ``stage_state['dial_window'][team]`` and trim anything older than
    ``movement_window_s``. Detection then looks at the peak-to-peak range
    *inside* this window (see ``_max_dial_delta_detail``), so slow drift around
    a set point rolls off the back of the window and never accumulates into a
    false wake.

    Arming (per team, tracked in ``stage_state['dial_arm'][team]``):
      * The window is dropped and arming restarts while haptic telemetry has
        not arrived (``haptic_seeded`` False) or a startup digital-reseat is
        still in progress (``_startup_alignment_active``). This guarantees the
        window only ever contains post-reseat samples.
      * A team arms only once a FULL window has elapsed since sampling (re)began
        (``now_ns - start_ns >= movement_window_s``) and at least two samples
        are buffered. Until armed, ``_max_dial_delta_detail`` ignores the team,
        so a half-collected window can never trigger a transition.

    Units: dial positions are dial-space radians (``last_dial``); ``now_ns`` is
    the monotonic game clock in nanoseconds.
    """
    window_s = float(game_cfg.get("movement_window_s", 2.0))
    window_ns = int(window_s * 1e9)
    trim = max(0, int(game_cfg.get("movement_glitch_trim", 3)))
    windows = stage_state.setdefault("dial_window", {})
    arm = stage_state.setdefault("dial_arm", {})

    for team, st in teams.items():
        # Not ready to sample: telemetry missing or still reseating. Drop the
        # window + tracker so arming restarts cleanly once alignment finishes.
        if not bool(st.get("haptic_seeded", False)) or _startup_alignment_active(st):
            windows.pop(team, None)
            arm.pop(team, None)
            continue

        cur = list(st.get("last_dial") or [0.0] * 6)[:6]
        buf = windows.get(team)
        tracker = arm.get(team)
        if buf is None or tracker is None:
            # First eligible tick: open a fresh window + arming clock.
            windows[team] = [(now_ns, cur)]
            arm[team] = {"start_ns": now_ns, "armed": False}
            continue

        buf.append((now_ns, cur))
        # Trim samples that have aged out of the window (always keep >= 1).
        cutoff = now_ns - window_ns
        while len(buf) > 1 and buf[0][0] < cutoff:
            buf.pop(0)

        # Arm once a full clean window has been collected.
        if not bool(tracker.get("armed", False)):
            elapsed = now_ns - int(tracker.get("start_ns", now_ns))
            if elapsed >= window_ns and len(buf) >= 2:
                tracker["armed"] = True
                ranges_deg = []
                for j in range(6):
                    low, high = _robust_range(_joint_series(buf, j), trim)
                    ranges_deg.append(round(math.degrees(high - low), 1))
                print(
                    f"[game_controller] movement-detect armed team={team} "
                    f"window_s={window_s:.2f} trim={trim} "
                    f"range_deg={ranges_deg}",
                    flush=True,
                )


def _joint_series(buf: list[tuple[int, list[float]]], joint: int) -> list[float]:
    """Extract one joint's value across every sample in a dial window."""
    return [s[1][joint] for s in buf if len(s[1]) > joint]


def _robust_range(values: list[float], trim: int) -> tuple[float, float]:
    """Return a glitch-tolerant (low, high) bound for one joint's samples.

    Sorts the samples and discards up to ``trim`` of the most-extreme values at
    each end before reading the bounds, which rejects brief encoder glitches
    (a few outlier frames) while preserving a sustained real turn. ``trim`` is
    clamped so at least one sample always survives on each side, so tiny
    windows degrade gracefully to plain min/max instead of collapsing to zero.
    """
    n = len(values)
    if n == 0:
        return 0.0, 0.0
    ordered = sorted(values)
    k = min(max(0, trim), (n - 1) // 2)
    return ordered[k], ordered[n - 1 - k]


def _max_dial_delta_detail(
    stage_state: dict[str, Any], teams: dict[str, dict], game_cfg: dict[str, Any]
) -> tuple[float, tuple[str, int, float, float] | None]:
    """Largest glitch-tolerant dial range (rad) over the window + detail.

    For every armed team, computes each joint's robust peak-to-peak range
    (``_robust_range``) across its rolling window and returns
    ``(best_range_rad, detail)`` where ``detail`` is
    ``(team, joint_index, low_rad, high_rad)`` for the joint with the widest
    range, or ``None`` when no team is armed yet (in which case movement
    detection must not fire). Scans every team that has data, so the same code
    path works whether one or both teams are active.
    """
    windows = stage_state.get("dial_window", {})
    arm = stage_state.get("dial_arm", {})
    trim = max(0, int(game_cfg.get("movement_glitch_trim", 3)))
    best = 0.0
    detail: tuple[str, int, float, float] | None = None
    for team in teams:
        tracker = arm.get(team)
        buf = windows.get(team)
        if not isinstance(tracker, dict) or not bool(tracker.get("armed", False)):
            continue
        if not isinstance(buf, list) or len(buf) < 2:
            continue
        for j in range(6):
            series = _joint_series(buf, j)
            if len(series) < 2:
                continue
            low, high = _robust_range(series, trim)
            span = high - low
            if span > best or detail is None:
                best = span
                detail = (team, j, float(low), float(high))
    return best, detail


def _movement_reason(
    label: str,
    detail: tuple[str, int, float, float],
    delta_rad: float,
    threshold_rad: float,
) -> str:
    """Format a human-readable transition reason describing the dial movement."""
    team, joint, low, high = detail
    return (
        f"{label}: team {team} J{joint + 1} moved {math.degrees(delta_rad):.1f}deg "
        f"(window range {math.degrees(low):.1f} -> {math.degrees(high):.1f}, "
        f"thr {math.degrees(threshold_rad):.0f}deg)"
    )


def _maybe_log_movement_progress(
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    game_cfg: dict[str, Any],
    threshold_rad: float,
    label: str,
    now_ns: int,
) -> None:
    """Throttled (~1 Hz) console readout of how close any dial is to triggering.

    Prints either the current widest window range vs the threshold (once
    armed), or a note that movement detection is still collecting its first
    window. Helps diagnose unexpected (or missing) stage transitions.
    """
    last = stage_state.get("_move_dbg_ns")
    if last is not None and (now_ns - last) < 1_000_000_000:
        return
    stage_state["_move_dbg_ns"] = now_ns

    delta, detail = _max_dial_delta_detail(stage_state, teams, game_cfg)
    if detail is None:
        print(
            f"[game_controller] {label}: movement detection not armed yet "
            f"(collecting window)",
            flush=True,
        )
        return
    team, joint, _low, _high = detail
    print(
        f"[game_controller] {label}: max dial range team {team} J{joint + 1} "
        f"{math.degrees(delta):.1f}deg / thr {math.degrees(threshold_rad):.0f}deg",
        flush=True,
    )


def _stage_countdown_s(
    stage_state: dict[str, Any], game_cfg: dict[str, Any], now_ns: int
) -> int:
    """Whole seconds remaining in the active timed stage (0 if untimed)."""
    stage = stage_state["stage"]
    if stage == "play":
        duration_s = float(game_cfg["duration_s"])
    elif stage == "tutorial":
        duration_s = float(game_cfg["tutorial_duration_s"])
    elif stage == "reset":
        if bool(game_cfg.get("rewind_enabled", False)):
            return 0
        duration_s = float(game_cfg["reset_duration_s"])
    else:
        return 0
    remaining_s = duration_s - _stage_elapsed_s(stage_state, now_ns)
    return max(0, int(math.ceil(remaining_s)))


def _stage_elapsed_s(stage_state: dict[str, Any], now_ns: int) -> float:
    pause_started_ns = stage_state.get("pause_started_mono_ns")
    paused_total_ns = int(stage_state.get("paused_total_ns") or 0)
    active_pause_ns = 0
    if pause_started_ns is not None:
        active_pause_ns = max(0, now_ns - int(pause_started_ns))
    return max(
        0.0,
        (
            now_ns
            - int(stage_state["stage_entered_mono_ns"])
            - paused_total_ns
            - active_pause_ns
        )
        / 1e9,
    )


def _update_stage_pause_tracking(
    stage_state: dict[str, Any], paused: bool, now_ns: int
) -> None:
    pause_started_ns = stage_state.get("pause_started_mono_ns")
    if paused:
        if pause_started_ns is None:
            stage_state["pause_started_mono_ns"] = now_ns
        return
    if pause_started_ns is None:
        return
    stage_state["paused_total_ns"] = int(stage_state.get("paused_total_ns") or 0) + (
        now_ns - int(pause_started_ns)
    )
    stage_state["pause_started_mono_ns"] = None


def _enter_conclusion(state: dict[str, Any], now_ns: int) -> None:
    """Initialise one team's conclusion show state on entering the stage.

    Args:
        state: The per-team state dict (mutated in place).
        now_ns: Monotonic clock; accepted for signature symmetry with the other
            stage-entry helpers (the conclusion show is driven by an accumulated
            ``dt`` clock, not ``now_ns``, so it is not stored here).
    """

    state["bucket_values"] = [max(0, int(round(v))) for v in state["bucket_values"]]
    # Current show phase (see ``_tick_conclusion_team``); ``None`` once finished.
    state["conclusion_phase"] = "pause_before_sum"
    # Which bucket (0-based) is currently being counted / looked at.
    state["conclusion_active_bucket_index"] = 0
    # Name + degrees of the pose the active move targets (None when holding).
    state["conclusion_target_pose_name"] = None
    state["conclusion_target_pose_deg"] = None
    # True for exactly the tick a bucket's door-open command is emitted.
    state["conclusion_bucket_open_triggered"] = False
    # Pause-aware clock: seconds spent in the current phase. Only advanced by
    # ``dt`` on running ticks, so a pause freezes every conclusion timer.
    state["conclusion_phase_elapsed_s"] = 0.0
    # Handshake with ``__main__``'s SegmentMover: set True to ask it to (re)seed
    # a move toward ``conclusion_target_pose_*``; ``__main__`` clears it.
    state["conclusion_move_pending"] = False
    # Set True by ``__main__`` once the active move's mover reports arrival.
    state["conclusion_move_arrived"] = False
    # True once the whole show has finished (lets the stage machine return idle).
    state["conclusion_done"] = False
    state["summed_score"] = 0
    state["score"] = int(sum(state["bucket_values"]))
    # Fractional score carried between sum ticks (units < 1 not yet credited).
    state["conclusion_sum_remainder_units"] = 0.0


def _request_conclusion_move(
    state: dict[str, Any],
    phase: str,
    pose_name: str,
    pose_cfg: dict[str, list[float]],
) -> None:
    """Switch to a motion phase and ask ``__main__`` to drive the SegmentMover.

    Sets the target pose, raises the ``conclusion_move_pending`` handshake, and
    resets the pause-aware phase clock + arrival flag.

    Args:
        state: Per-team state dict (mutated in place).
        phase: The motion phase name to enter (e.g. ``"move_to_bucket_pose"``).
        pose_name: Key into ``pose_cfg`` for the goal pose (degrees).
        pose_cfg: This team's ``robot_show_poses`` mapping (name -> degrees).
    """

    state["conclusion_phase"] = phase
    state["conclusion_target_pose_name"] = pose_name
    state["conclusion_target_pose_deg"] = pose_cfg.get(
        pose_name, list(DEFAULT_LOOK_POSE_DEG)
    )
    state["conclusion_move_pending"] = True
    state["conclusion_move_arrived"] = False
    state["conclusion_phase_elapsed_s"] = 0.0


def _enter_hold_phase(state: dict[str, Any], phase: str) -> None:
    """Switch to a non-motion (hold/count) phase and reset its phase clock."""

    state["conclusion_phase"] = phase
    state["conclusion_phase_elapsed_s"] = 0.0


# Bucket index -> "look at bucket" pose name in ``robot_show_poses``.
_BUCKET_LOOK_POSE_NAMES = (
    "robot_lookb1_pose",
    "robot_lookb2_pose",
    "robot_lookb3_pose",
)


def _tick_conclusion_team(
    state: dict[str, Any],
    dt: float,
    game_cfg: dict[str, float],
    pose_cfg: dict[str, list[float]],
    stage_state: dict[str, Any],
    *,
    bucket_command_fn=None,
) -> None:
    """Advance one team's conclusion scoring + choreography by one tick.

    Show sequence (per team):
      pause_before_sum -> for each bucket: move_to_bucket_pose -> sum_bucket ->
      empty_bucket -> ... -> move_to_announcement -> announcement_pause ->
      move_to_winner_pose -> winner_pose_hold -> move_to_begin -> done.

    Motion phases (``move_*``) set ``conclusion_move_pending`` and wait for
    ``__main__`` to report ``conclusion_move_arrived``; the SegmentMover owns
    their duration. Hold phases use the pause-aware ``conclusion_phase_elapsed_s``
    clock advanced below.

    Args:
        state: Per-team state dict (mutated in place).
        dt: Seconds elapsed since the previous running tick. Only added to the
            phase clock, so paused ticks (which never call this) never advance it.
        game_cfg: Game config; uses ``sum_score_rate_unit_per_s`` (points/sec
            the buckets count down at).
        pose_cfg: This team's ``robot_show_poses`` mapping (name -> degrees).
        stage_state: Shared stage state; read ``winner_team`` to pick win/lose.
        bucket_command_fn: Optional callback ``(action, *, team, bucket_number,
            reason)`` used to open a bucket door when its count reaches zero.
    """

    phase = state.get("conclusion_phase")
    if phase is None:
        return

    # Pause-aware clock: only running ticks reach this function, so summing dt
    # here naturally freezes every timer during an e-stop / soft-pause and keeps
    # both teams (advanced with the same dt each loop) phase-aligned.
    elapsed_s = float(state.get("conclusion_phase_elapsed_s", 0.0)) + max(0.0, float(dt))
    state["conclusion_phase_elapsed_s"] = elapsed_s

    if phase == "pause_before_sum":
        if elapsed_s >= CONCLUSION_INITIAL_PAUSE_S:
            bucket_index = int(state.get("conclusion_active_bucket_index") or 0)
            pose_name = _BUCKET_LOOK_POSE_NAMES[
                min(bucket_index, len(_BUCKET_LOOK_POSE_NAMES) - 1)
            ]
            _request_conclusion_move(state, "move_to_bucket_pose", pose_name, pose_cfg)
        return

    if phase == "move_to_bucket_pose":
        # Advance only once the real robot move has finished (arrival reported by
        # __main__). Duration follows the retimed trajectory, not a timer.
        if bool(state.get("conclusion_move_arrived")):
            _enter_hold_phase(state, "sum_bucket")
        return

    if phase == "sum_bucket":
        bucket_index = int(state.get("conclusion_active_bucket_index") or 0)
        if bucket_index >= len(state["bucket_values"]):
            _request_conclusion_move(
                state, "move_to_announcement", "robot_announcement_pose", pose_cfg
            )
            return

        remaining = int(state["bucket_values"][bucket_index])
        accumulated_units = float(state.get("conclusion_sum_remainder_units", 0.0)) + (
            game_cfg["sum_score_rate_unit_per_s"] * dt
        )
        delta = min(remaining, int(accumulated_units))
        state["conclusion_sum_remainder_units"] = accumulated_units - delta
        state["bucket_values"][bucket_index] = max(0, remaining - delta)
        state["summed_score"] = int(state.get("summed_score", 0)) + delta
        state["score"] = int(sum(state["bucket_values"]))
        if state["bucket_values"][bucket_index] <= 0:
            state["bucket_values"][bucket_index] = 0
            _enter_hold_phase(state, "empty_bucket")
            state["conclusion_bucket_open_triggered"] = True
            state["conclusion_sum_remainder_units"] = 0.0
            if bucket_command_fn is not None:
                bucket_command_fn(
                    "open",
                    team=state["team"],
                    bucket_number=bucket_index + 1,
                    reason="conclusion_bucket_counted",
                )
        return

    if phase == "empty_bucket":
        if elapsed_s >= CONCLUSION_BUCKET_EMPTY_PAUSE_S:
            next_bucket_index = (
                int(state.get("conclusion_active_bucket_index") or 0) + 1
            )
            state["conclusion_active_bucket_index"] = next_bucket_index
            state["conclusion_bucket_open_triggered"] = False
            if next_bucket_index >= len(state["bucket_values"]):
                _request_conclusion_move(
                    state, "move_to_announcement", "robot_announcement_pose", pose_cfg
                )
            else:
                pose_name = _BUCKET_LOOK_POSE_NAMES[
                    min(next_bucket_index, len(_BUCKET_LOOK_POSE_NAMES) - 1)
                ]
                _request_conclusion_move(
                    state, "move_to_bucket_pose", pose_name, pose_cfg
                )
        return

    if phase == "move_to_announcement":
        if bool(state.get("conclusion_move_arrived")):
            _enter_hold_phase(state, "announcement_pause")
        return

    if phase == "announcement_pause":
        if elapsed_s < CONCLUSION_ANNOUNCEMENT_PAUSE_S:
            return
        winner_team = stage_state.get("winner_team")
        if winner_team is None:
            # Wait for the stage machine to resolve the winner (it does so once
            # every team has reached the announcement-ready phase set).
            return
        if winner_team == "tie":
            pose_name = "robot_win_pose"
        else:
            pose_name = (
                "robot_win_pose"
                if state["team"] == winner_team
                else "robot_lose_pose"
            )
        _request_conclusion_move(state, "move_to_winner_pose", pose_name, pose_cfg)
        return

    if phase == "move_to_winner_pose":
        if bool(state.get("conclusion_move_arrived")):
            _enter_hold_phase(state, "winner_pose_hold")
        return

    if phase == "winner_pose_hold":
        if elapsed_s >= CONCLUSION_WINNER_POSE_HOLD_S:
            _request_conclusion_move(
                state, "move_to_begin", "robot_begin_pose", pose_cfg
            )
        return

    if phase == "move_to_begin":
        if bool(state.get("conclusion_move_arrived")):
            state["conclusion_phase"] = None
            state["conclusion_target_pose_name"] = None
            state["conclusion_target_pose_deg"] = None
            state["conclusion_done"] = True
        return


def _winner_team(teams: dict[str, dict]) -> str | None:
    if not teams:
        return None
    ordered = sorted(
        ((team, int(st.get("summed_score", 0) or 0)) for team, st in teams.items()),
        key=lambda item: item[1],
        reverse=True,
    )
    if len(ordered) >= 2 and ordered[0][1] == ordered[1][1]:
        return "tie"
    return ordered[0][0]
