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
import time
from typing import Any

from apps.game_controller.context import (
    DEFAULT_BUCKET_VALUES,
    DEFAULT_LOOK_POSE_DEG,
    _startup_alignment_active,
)

# --- Conclusion scoring sequence timing ------------------------------------
# All are placeholder waits standing in for future robot motions; tune as the
# real conclusion trajectories land. Units are seconds.
# Pause after entering conclusion, before the first bucket is summed.
CONCLUSION_INITIAL_PAUSE_S = 1.0
# Stand-in wait for the "move to bucket look pose" motion.
CONCLUSION_BUCKET_LOOK_MOTION_WAIT_S = 5.0
# Short pause after a bucket empties (door open) before the next bucket.
CONCLUSION_BUCKET_EMPTY_PAUSE_S = 0.5
# Stand-in wait for the celebration motion after the final bucket opens.
CONCLUSION_CELEBRATION_MOTION_WAIT_S = 5.0
# Pause on the announcement pose before resolving winner / loser poses.
CONCLUSION_ANNOUNCEMENT_PAUSE_S = 1.0


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
        _refresh_dial_baselines(stage_state, teams, game_cfg)
        wake_rad = math.radians(float(game_cfg["daydream_to_idle_dial_deg"]))
        _maybe_log_movement_progress(stage_state, teams, wake_rad, "daydreaming", now_ns)
        delta, detail = _max_dial_delta_detail(stage_state, teams)
        if detail is not None and delta >= wake_rad:
            _enter_stage(
                stage_state, teams, "idle", game_cfg, now_ns,
                reason=_movement_reason("dial moved -> wake", detail, delta, wake_rad),
            )
        return

    if stage == "idle":
        # Ready state. A big "scroll up" starts the tutorial; otherwise a long
        # quiet period drops back to daydreaming. No countdown shown.
        _refresh_dial_baselines(stage_state, teams, game_cfg)
        start_rad = math.radians(float(game_cfg["idle_to_tutorial_dial_deg"]))
        _maybe_log_movement_progress(stage_state, teams, start_rad, "idle", now_ns)
        delta, detail = _max_dial_delta_detail(stage_state, teams)
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
        announcement_ready = {"announcement_pose", "winner_pose"}
        if stage_state["winner_team"] is None and all(
            bool(st.get("conclusion_done", False))
            or str(st.get("conclusion_phase")) in announcement_ready
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
    baselines and any pending skip, seeds a fresh game on `play`, and seeds
    the scoring sequence on `conclusion`. Prints a one-line transition banner
    so the operator can trace the flow on the console.
    """
    old_stage = stage_state.get("stage")
    stage_state["stage"] = new_stage
    stage_state["stage_entered_mono_ns"] = now_ns
    stage_state["pause_started_mono_ns"] = None
    stage_state["paused_total_ns"] = 0
    stage_state["dial_baseline"] = {}
    # Per-team "is the dial still yet?" trackers used to settle-gate baseline
    # capture; reset so the new stage re-arms movement detection cleanly.
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
        st["conclusion_phase_started_mono_ns"] = None
        st["conclusion_done"] = False
        st["conclusion_sum_remainder_units"] = 0.0


def _refresh_dial_baselines(
    stage_state: dict[str, Any], teams: dict[str, dict], game_cfg: dict[str, Any]
) -> None:
    """Capture each team's dial position as the movement baseline, once settled.

    Movement detection is "armed" per team only after ALL of:
      * haptic telemetry has arrived (`haptic_seeded`), and
      * any startup digital-reseat alignment has finished, and
      * the dial has then stayed still (max per-tick change <=
        ``movement_arm_quiet_deg``) for ``movement_arm_quiet_ticks`` ticks.

    The stillness streak is the key safeguard: right after the startup reseat
    (which can be a multi-radian digital jump) the dial telemetry keeps moving
    while it settles, so we wait for it to go quiet before snapshotting the
    baseline. Without this, the settle motion was being mistaken for a player
    turning the dial and instantly woke daydreaming -> idle on boot.

    Per-team trackers live in ``stage_state['dial_arm']`` and are cleared on
    every stage entry (alongside ``dial_baseline``).
    """
    baselines = stage_state.setdefault("dial_baseline", {})
    arm = stage_state.setdefault("dial_arm", {})
    quiet_rad = math.radians(float(game_cfg.get("movement_arm_quiet_deg", 2.0)))
    quiet_ticks = max(1, int(game_cfg.get("movement_arm_quiet_ticks", 30)))

    for team, st in teams.items():
        if team in baselines:
            continue
        # Not ready to even start settling: haptic not seeded, or still
        # aligning. Drop any partial tracker so the streak restarts cleanly.
        if not bool(st.get("haptic_seeded", False)) or _startup_alignment_active(st):
            arm.pop(team, None)
            continue

        cur = list(st.get("last_dial") or [0.0] * 6)[:6]
        tracker = arm.get(team)
        if tracker is None or not isinstance(tracker.get("prev"), list):
            # First eligible tick: start the stillness window.
            arm[team] = {"streak": 0, "prev": cur}
            continue

        prev = tracker["prev"]
        moved = 0.0
        for i in range(min(6, len(prev), len(cur))):
            change = abs(float(cur[i]) - float(prev[i]))
            if change > moved:
                moved = change
        tracker["streak"] = (
            int(tracker.get("streak", 0)) + 1 if moved <= quiet_rad else 0
        )
        tracker["prev"] = cur

        if tracker["streak"] >= quiet_ticks:
            baselines[team] = list(cur)
            arm.pop(team, None)
            print(
                f"[game_controller] movement-detect armed team={team} "
                f"baseline_deg={[round(math.degrees(v), 1) for v in cur]}",
                flush=True,
            )


def _max_dial_delta_detail(
    stage_state: dict[str, Any], teams: dict[str, dict]
) -> tuple[float, tuple[str, int, float, float] | None]:
    """Largest absolute dial change (rad) from the captured baseline + detail.

    Returns ``(best_delta_rad, detail)`` where ``detail`` is
    ``(team, joint_index, baseline_rad, current_rad)`` for the joint that moved
    the most, or ``None`` if no team has an armed baseline yet (in which case
    movement detection must not fire). Scans every team that has a baseline, so
    the same code path works whether only team A is active now or both teams
    are active later.
    """
    baselines = stage_state.get("dial_baseline", {})
    best = 0.0
    detail: tuple[str, int, float, float] | None = None
    for team, st in teams.items():
        base = baselines.get(team)
        if not isinstance(base, list):
            continue
        cur = st.get("last_dial") or []
        for i in range(min(6, len(base), len(cur))):
            delta = abs(float(cur[i]) - float(base[i]))
            if delta > best or detail is None:
                best = delta
                detail = (team, i, float(base[i]), float(cur[i]))
    return best, detail


def _movement_reason(
    label: str,
    detail: tuple[str, int, float, float],
    delta_rad: float,
    threshold_rad: float,
) -> str:
    """Format a human-readable transition reason describing the dial movement."""
    team, joint, base, cur = detail
    return (
        f"{label}: team {team} J{joint + 1} moved {math.degrees(delta_rad):.1f}deg "
        f"(base {math.degrees(base):.1f} -> now {math.degrees(cur):.1f}, "
        f"thr {math.degrees(threshold_rad):.0f}deg)"
    )


def _maybe_log_movement_progress(
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    threshold_rad: float,
    label: str,
    now_ns: int,
) -> None:
    """Throttled (~1 Hz) console readout of how close any dial is to triggering.

    Prints either the current max dial delta vs the threshold (once armed), or
    a note that movement detection is still waiting for the dials to settle.
    Helps diagnose unexpected (or missing) stage transitions.
    """
    last = stage_state.get("_move_dbg_ns")
    if last is not None and (now_ns - last) < 1_000_000_000:
        return
    stage_state["_move_dbg_ns"] = now_ns

    delta, detail = _max_dial_delta_detail(stage_state, teams)
    if detail is None:
        print(
            f"[game_controller] {label}: movement detection not armed yet "
            f"(waiting for dials to settle)",
            flush=True,
        )
        return
    team, joint, _base, _cur = detail
    print(
        f"[game_controller] {label}: max dial delta team {team} J{joint + 1} "
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
    state["bucket_values"] = [max(0, int(round(v))) for v in state["bucket_values"]]
    state["conclusion_phase"] = "pause_before_sum"
    state["conclusion_active_bucket_index"] = 0
    state["conclusion_target_pose_name"] = None
    state["conclusion_target_pose_deg"] = None
    state["conclusion_bucket_open_triggered"] = False
    state["conclusion_phase_started_mono_ns"] = now_ns
    state["conclusion_done"] = False
    state["summed_score"] = 0
    state["score"] = int(sum(state["bucket_values"]))
    state["conclusion_sum_remainder_units"] = 0.0


def _tick_conclusion_team(
    state: dict[str, Any],
    dt: float,
    game_cfg: dict[str, float],
    pose_cfg: dict[str, list[float]],
    stage_state: dict[str, Any],
    *,
    bucket_command_fn=None,
) -> None:
    """Advance one team's conclusion scoring sequence and bucket commands."""

    phase = state.get("conclusion_phase")
    if phase is None:
        return

    now_ns = time.perf_counter_ns()
    phase_started_ns = int(state.get("conclusion_phase_started_mono_ns") or now_ns)
    phase_elapsed_s = (now_ns - phase_started_ns) / 1e9

    if phase == "pause_before_sum":
        if phase_elapsed_s >= CONCLUSION_INITIAL_PAUSE_S:
            _set_bucket_pose_phase(state, now_ns, pose_cfg)
        return

    if phase == "move_to_bucket_pose":
        # Temporary stand-in for the future conclusion robot motion that
        # will move to the active bucket look pose. Do not block the GC
        # loop here; this phase simply lets state.full keep publishing
        # while we simulate waiting for that motion to finish.
        # TODO(conclusion-motion): replace this timer with completion from
        # the free-motion planner / robot trajectory executor.
        if phase_elapsed_s >= CONCLUSION_BUCKET_LOOK_MOTION_WAIT_S:
            state["conclusion_phase"] = "sum_bucket"
            state["conclusion_phase_started_mono_ns"] = now_ns
        return

    if phase == "sum_bucket":
        bucket_index = int(state.get("conclusion_active_bucket_index") or 0)
        if bucket_index >= len(state["bucket_values"]):
            _set_announcement_phase(state, now_ns, pose_cfg)
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
            state["conclusion_phase"] = "empty_bucket"
            state["conclusion_bucket_open_triggered"] = True
            state["conclusion_phase_started_mono_ns"] = now_ns
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
        if phase_elapsed_s >= CONCLUSION_BUCKET_EMPTY_PAUSE_S:
            next_bucket_index = (
                int(state.get("conclusion_active_bucket_index") or 0) + 1
            )
            state["conclusion_active_bucket_index"] = next_bucket_index
            state["conclusion_bucket_open_triggered"] = False
            if next_bucket_index >= len(state["bucket_values"]):
                _set_celebration_phase(state, now_ns, pose_cfg)
            else:
                _set_bucket_pose_phase(state, now_ns, pose_cfg)
        return

    if phase == "celebration_motion":
        # Temporary stand-in for the future celebration robot motion that
        # happens after the final bucket opens. Keep this non-blocking so
        # GameController continues publishing state while the fake motion
        # duration elapses.
        # TODO(conclusion-motion): replace this timer with completion from
        # the celebration trajectory executor.
        if phase_elapsed_s >= CONCLUSION_CELEBRATION_MOTION_WAIT_S:
            _set_announcement_phase(state, now_ns, pose_cfg)
        return

    if phase == "announcement_pose":
        if phase_elapsed_s >= CONCLUSION_ANNOUNCEMENT_PAUSE_S:
            winner_team = stage_state.get("winner_team")
            if winner_team is None:
                return
            state["conclusion_phase"] = "winner_pose"
            if winner_team == "tie":
                state["conclusion_target_pose_name"] = "robot_win_pose"
            else:
                state["conclusion_target_pose_name"] = (
                    "robot_win_pose"
                    if state["team"] == winner_team
                    else "robot_lose_pose"
                )
            state["conclusion_target_pose_deg"] = pose_cfg.get(
                state["conclusion_target_pose_name"], list(DEFAULT_LOOK_POSE_DEG)
            )
            state["conclusion_phase_started_mono_ns"] = now_ns
            # TODO(conclusion-motion): replace this pose bookkeeping with a
            # collision-free motion plan once the dedicated planner lands.
        return

    if phase == "winner_pose":
        state["conclusion_done"] = True


def _set_bucket_pose_phase(
    state: dict[str, Any], now_ns: int, pose_cfg: dict[str, list[float]]
) -> None:
    bucket_index = int(state.get("conclusion_active_bucket_index") or 0)
    pose_names = ["robot_lookb1_pose", "robot_lookb2_pose", "robot_lookb3_pose"]
    pose_name = pose_names[min(bucket_index, len(pose_names) - 1)]
    state["conclusion_phase"] = "move_to_bucket_pose"
    state["conclusion_target_pose_name"] = pose_name
    state["conclusion_target_pose_deg"] = pose_cfg.get(
        pose_name, list(DEFAULT_LOOK_POSE_DEG)
    )
    state["conclusion_phase_started_mono_ns"] = now_ns
    # TODO(conclusion-motion): send the collision-free move to this
    # bucket-look pose here. Until that planner exists,
    # move_to_bucket_pose waits for CONCLUSION_BUCKET_LOOK_MOTION_WAIT_S.


def _set_celebration_phase(
    state: dict[str, Any], now_ns: int, pose_cfg: dict[str, list[float]]
) -> None:
    """Enter the temporary post-bucket celebration phase."""

    state["conclusion_phase"] = "celebration_motion"
    state["conclusion_target_pose_name"] = "robot_celebration_pose"
    state["conclusion_target_pose_deg"] = pose_cfg.get(
        "robot_celebration_pose", list(DEFAULT_LOOK_POSE_DEG)
    )
    state["conclusion_phase_started_mono_ns"] = now_ns
    # TODO(conclusion-motion): send the celebration trajectory here. Until
    # that exists, celebration_motion waits for
    # CONCLUSION_CELEBRATION_MOTION_WAIT_S.


def _set_announcement_phase(
    state: dict[str, Any], now_ns: int, pose_cfg: dict[str, list[float]]
) -> None:
    state["conclusion_phase"] = "announcement_pose"
    state["conclusion_target_pose_name"] = "robot_announcement_pose"
    state["conclusion_target_pose_deg"] = pose_cfg.get(
        "robot_announcement_pose", list(DEFAULT_LOOK_POSE_DEG)
    )
    state["conclusion_phase_started_mono_ns"] = now_ns


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
