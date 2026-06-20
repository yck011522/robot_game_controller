"""game_controller entry point ??see __init__.py."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting  # noqa: E402
from core.proc import Proc, banner  # noqa: E402
from subsystems.jogging.in_process import InProcessPlanner  # noqa: E402
from subsystems.rewind.in_process import RewindController  # noqa: E402
from subsystems.rewind.shortcut import ShortcutSettings  # noqa: E402
from apps.game_controller.batch_validation import (  # noqa: E402
    BatchValidationSession,
    batch_validation_settings,
)

# Shared constants + profile/config construction live in the context module;
# the stage machine + conclusion scoring live in the stages module; runtime
# helpers are grouped by domain in the sibling safety/weight/haptics modules.
# Everything is imported here by name so main()/tick() remain orchestration-only.
from apps.game_controller.context import (  # noqa: E402
    DEFAULT_SAFETY_TELEM_AGE_MAX_MS,
    TEAM_BUCKET_IDS,
    _game_config,
    _load_robot_show_poses_deg,
    _rewind_shortcut_config,
    _startup_alignment_active,
)
from apps.game_controller.context import (  # noqa: E402
    DEFAULT_BUCKET_VALUES,
)
from apps.game_controller.haptics import (  # noqa: E402
    _begin_play_sync,
    _haptic_config,
    _publish_haptic_command,
    _publish_hold_current_pose,
    _reset_haptic_bounds_to_static,
    _reset_team_motion_outputs,
    _tick_startup_alignment,
    _tick_play_sync,
    _tick_tutorial_team,
    _update_dynamic_haptic_bounds_from_prox,
    _update_haptic_state,
)
from apps.game_controller.operator_inputs import (  # noqa: E402
    _drain_operator_input_requests,
    _handle_operator_input_request,
    _is_robot_status_recovered,
    _publish_pending_recovery_requests,
)
from apps.game_controller.published_states import (  # noqa: E402
    _build_state_full_payload,
    _pause_state_summary,
)
from apps.game_controller.safety import (  # noqa: E402
    _initial_safety_state,
    _refresh_safety_block,
    _update_safety_state,
)
from apps.game_controller.stages import (  # noqa: E402
    _enter_stage,
    _stage_countdown_s,
    _tick_conclusion_team,
    _tick_stage_state,
    _update_stage_pause_tracking,
)
from apps.game_controller.weight import (  # noqa: E402
    _apply_weight_bucket_values,
    _initial_weight_state,
    _team_bucket_labels,
    _update_weight_state,
)

# game_controller ticks at a fixed rate. Each tick blocks on the
# forward-collision certify (~worker compute + ZMQ round-trip), then
# publishes one cmd.robot.target.<team> per team and one state.full.
# 60 Hz gives ~16 ms per tick, which is comfortably above the
# forward_timeout_ms budget and keeps state.full at a recordable rate.
TICK_HZ = 60.0
BUCKET_COMMAND_TOPIC = "cmd.bucket"
WEIGHT_TARE_TOPIC = "cmd.weight.tare"
RECOVERY_TIMEOUT_S = 4.0


def main(argv: list[str] | None = None) -> int:
    """Run the central game loop, state machine, and robot command publisher."""
    proc, _ = Proc.from_argv(target_hz=TICK_HZ, default_proc="game_controller")

    active_teams = list(proc.profile.active_teams)
    game_cfg = _game_config(proc.profile.tuning.get("game"))
    shortcut_cfg = _rewind_shortcut_config(
        proc.profile.tuning.get("rewind_shortcut")
    )
    batch_cfg = batch_validation_settings(
        proc.profile.tuning.get("batch_validation")
    )
    batch_session = BatchValidationSession(batch_cfg) if batch_cfg.enabled else None
    haptic_cfg = _haptic_config(proc.profile.tuning.get("haptic"))
    safety_enabled = (
        proc.profile.subsystem_impl("safety_barrier_controller") is not None
    )
    bucket_controller_enabled = (
        proc.profile.subsystem_impl("bucket_controller") is not None
    )
    weight_sensor_enabled = proc.profile.subsystem_impl("weight_sensor_io") is not None
    safety_telem_age_max_s = (
        default_runtime_setting(
            "safety_barrier_controller",
            "telem_age_max",
            DEFAULT_SAFETY_TELEM_AGE_MAX_MS,
        )
        or DEFAULT_SAFETY_TELEM_AGE_MAX_MS
    ) / 1000.0
    robot_show_poses = _load_robot_show_poses_deg()
    robot_tuning = proc.profile.tuning.get("robot", {})
    max_velocity_deg_s = robot_tuning.get(
        "max_velocity_deg_s", [20.0, 20.0, 20.0, 30.0, 30.0, 30.0]
    )
    max_velocity_rad_s = [
        math.radians(float(value)) for value in list(max_velocity_deg_s)[:6]
    ]
    while len(max_velocity_rad_s) < 6:
        max_velocity_rad_s.append(math.radians(20.0))
    pub = bus.make_pub(proc.ctx)
    operator_input_rep = bus.make_rep(proc.ctx)
    safety_sub = (
        bus.make_sub(proc.ctx, topics=["telem.safety"]) if safety_enabled else None
    )
    weight_sub = (
        bus.make_sub(proc.ctx, topics=["telem.weight"]) if weight_sensor_enabled else None
    )
    proc.use_heartbeat_pub(pub)

    # P2 ships team-A only; team-B wiring is symmetric and lands when
    # the second arm joins.
    if "a" not in active_teams:
        banner(
            proc.proc, "no active teams; will only emit heartbeat + skeleton state.full"
        )

    collision_worker_count = (
        int(proc.profile.subsystems["collision_workers"].get("count", 0))
        if isinstance(proc.profile.subsystems.get("collision_workers"), dict)
        else 0
    )
    collision_enabled = (
        isinstance(proc.profile.subsystems.get("collision_workers"), dict)
        and collision_worker_count > 0
    )

    # Per-team state. P2 builds only `a`; the structure generalizes.
    teams: dict[str, dict] = {}
    team_count = max(1, len(active_teams))
    workers_per_team = collision_worker_count // team_count
    extra_workers = collision_worker_count % team_count
    for team_index, team in enumerate(active_teams):
        # One team receives the entire collision pool. With multiple teams,
        # bounded in-flight request counts divide broker capacity evenly.
        shortcut_worker_limit = workers_per_team + (
            1 if team_index < extra_workers else 0
        )
        configured_seed = shortcut_cfg["random_seed"]
        team_seed = (
            batch_session.shortcut_seed(team_index)
            if batch_session is not None
            else configured_seed + team_index
            if configured_seed is not None
            else None
        )
        planner = InProcessPlanner(
            ctx=proc.ctx,
            profile=proc.profile,
            team=team,
            collision_enabled=collision_enabled,
        )
        sub = bus.make_sub(proc.ctx, topics=[f"telem.haptic.{team}"])
        actual_sub = bus.make_sub(proc.ctx, topics=[f"telem.robot.actual.{team}"])
        teams[team] = {
            "planner": planner,
            # In-process geometric recorder/rewinder. It returns targets only;
            # this game-controller loop remains the owner of all bus routing.
            "rewind": RewindController(
                enabled=game_cfg["rewind_enabled"],
                max_velocity_rad_s=max_velocity_rad_s,
                speed_fraction=game_cfg["rewind_speed_fraction"],
                arrival_tolerance_rad=math.radians(
                    game_cfg["rewind_arrival_tolerance_deg"]
                ),
                team=team,
                shortcut_settings=ShortcutSettings(
                    enabled=(
                        shortcut_cfg["enabled"]
                        and collision_enabled
                        and shortcut_worker_limit > 0
                    ),
                    optimization_budget_s=shortcut_cfg["optimization_budget_s"],
                    collision_step_rad=math.radians(
                        shortcut_cfg["collision_step_deg"]
                    ),
                    collision_batch_size=shortcut_cfg["collision_batch_size"],
                    worker_limit=max(1, shortcut_worker_limit),
                    random_seed=team_seed,
                ),
            ),
            "team": team,
            "sub_haptic": sub,
            "sub_actual": actual_sub,
            "last_dial": [0.0] * 6,
            "last_dial_vel": [0.0] * 6,
            "haptic_required": (
                proc.profile.subsystems.get("haptic_io", {}).get(team) is not None
                if isinstance(proc.profile.subsystems.get("haptic_io"), dict)
                else False
            ),
            "haptic_seeded": False,
            "last_haptic_connected": [False] * 6,
            "last_haptic_loop_hz": [0.0] * 6,
            # Current assistive haptic bounds (dial space, rad) sent on
            # cmd.haptic.<team>; initialized to static profile defaults.
            "current_haptic_bounds_min_rad": list(haptic_cfg["bounds_min_rad"]),
            "current_haptic_bounds_max_rad": list(haptic_cfg["bounds_max_rad"]),
            # last_q starts as None; planner only re-seeds once a real
            # telem.robot.actual.<team> has actually arrived. Without
            # this guard the very first tick would seed the planner's
            # integrator with all-zero (the default) and the robot
            # would snap to the in-pedestal pose.
            "last_q": None,
            "last_target": None,
            "last_collision": False,
            "last_first_hit": None,
            "last_path_scalar": 1.0,
            "last_prox_scalar": 1.0,
            "last_final_scalar": 1.0,
            "last_planner_info": {},
            "last_prox_probe_offsets_deg": [],
            "last_prox_hits": [[False] * 20 for _ in range(6)],
            "last_prox_age_ticks": [9999] * 6,
            "robot_status": {},
            "bucket_ids": list(TEAM_BUCKET_IDS.get(team, [])),
            "bucket_labels": _team_bucket_labels(team),
            "bucket_values": list(
                game_cfg["sim_bucket_values"].get(team, DEFAULT_BUCKET_VALUES)
            ),
            "score": int(
                sum(game_cfg["sim_bucket_values"].get(team, DEFAULT_BUCKET_VALUES))
            ),
            "summed_score": 0,
            "conclusion_phase": None,
            "conclusion_active_bucket_index": None,
            "conclusion_target_pose_name": None,
            "conclusion_target_pose_deg": None,
            "conclusion_bucket_open_triggered": False,
            "conclusion_phase_started_mono_ns": None,
            "conclusion_done": False,
            "conclusion_sum_remainder_units": 0.0,
            "last_tick_t": time.perf_counter(),
            "startup_align": {
                "enabled": (
                    proc.profile.subsystems.get("haptic_io", {}).get(team) == "real"
                    if isinstance(proc.profile.subsystems.get("haptic_io"), dict)
                    else False
                ),
                "done": False,
                "attempts": 0,
                "last_reseat_mono_s": 0.0,
                "settled_streak": 0,
            },
            # Per-game logical dial reseat. The stage machine requests it on
            # play entry; this runtime loop owns publishing and settle gating.
            "play_sync": {
                "enabled": (
                    proc.profile.subsystems.get("haptic_io", {}).get(team) == "real"
                    if isinstance(proc.profile.subsystems.get("haptic_io"), dict)
                    else False
                ),
                "requested": False,
                "pending": False,
                "target_dial_rad": None,
                "last_reseat_mono_s": 0.0,
                "settled_streak": 0,
                "attempts": 0,
            },
            # Per-player tutorial scroll progress (0..100%), refreshed every
            # tutorial tick from the measured dial position. Published in
            # state.full and consumed by the LEDs + dashboard.
            "tutorial_progress": [0.0] * 6,
            # One-shot flag set on tutorial entry; the runtime loop reseats the
            # dial to 0 and installs the tutorial bounds, then clears it.
            "tutorial_reset_pending": False,
        }
    banner(proc.proc, f"teams={active_teams} collision_check={collision_enabled}")

    state_seq = 0
    bucket_command_seq = 0
    weight_tare_seq = 0
    # Game stage machine. The boot stage comes from tuning.game.start_stage
    # (or the legacy force_stage), defaulting to "play" for back-compat.
    # `_enter_stage` below runs the boot-stage entry effects + banner.
    stage_state = {
        "stage": "(init)",
        "stage_entered_mono_ns": time.perf_counter_ns(),
        "winner_team": None,
        "pause_started_mono_ns": None,
        "paused_total_ns": 0,
        # Movement-detection baselines (team -> [6] dial rad); captured on
        # entering daydreaming / idle, cleared on every stage change.
        "dial_baseline": {},
        # Set by the UI / physical SKIP control; consumed in play & tutorial.
        "skip_requested": False,
        # Edge tracker so PAUSE banners only print on on/off transitions.
        "prev_paused": False,
    }
    control_state = {
        "soft_pause": False,
        "last_action": None,
        "last_action_ts_mono_ns": None,
        "fault_active_prev_by_team": {team: False for team in active_teams},
        "recovery_active": False,
        "recovery_deadline_mono_ns": None,
        "recovery_pending_dispatch": False,
        "recovery_request_id": 0,
        "recovery_teams": [],
        "safety_blocked": False,
        "safety_pause_latched": False,
        # Cache the last reply per source so a UI retry with the same
        # request_id can be acknowledged without reapplying the action.
        "last_request_id_by_source": {},
        "last_reply_by_source": {},
    }
    safety_state = _initial_safety_state(enabled=safety_enabled)
    weight_state = _initial_weight_state(enabled=weight_sensor_enabled)

    # Run the boot-stage entry effects (banner + seeding) once teams exist.
    _enter_stage(
        stage_state,
        teams,
        game_cfg["start_stage"],
        game_cfg,
        time.perf_counter_ns(),
        reason="boot",
    )
    if batch_session is not None and stage_state["stage"] == "play":
        batch_session.mark_play_started()
    last_batch_shutdown_s = 0.0

    def _prepare_next_batch_game() -> None:
        """Install the next game's seeds and notify synthetic haptic inputs."""

        assert batch_session is not None
        for team_index, (team, st) in enumerate(teams.items()):
            gameplay_seed = batch_session.gameplay_seed(team_index)
            shortcut_seed = batch_session.shortcut_seed(team_index)
            st["rewind"].set_shortcut_seed(shortcut_seed)
            env = bus.make_envelope(proc.proc)
            env.update(
                {
                    "team": team,
                    "game_index": batch_session.game_index,
                    "seed": gameplay_seed,
                }
            )
            bus.publish(pub, f"cmd.validation.seed.{team}", env)

    def _publish_batch_shutdown() -> None:
        """Repeat a sparse shutdown request until the launcher acknowledges by exit."""

        nonlocal last_batch_shutdown_s
        assert batch_session is not None
        now_s = time.perf_counter()
        if now_s - last_batch_shutdown_s < 0.2:
            return
        last_batch_shutdown_s = now_s
        env = bus.make_envelope(proc.proc, with_wall=True)
        env.update(
            {
                "reason": "batch_validation_complete",
                "completed_games": batch_session.completed_game_count,
            }
        )
        bus.publish(pub, "cmd.launcher.shutdown", env)

    def _publish_bucket_command(
        action: str,
        *,
        team: str | None = None,
        bucket_number: int | None = None,
        reason: str,
    ) -> None:
        """Publish one sparse command for the bucket_controller process."""

        nonlocal bucket_command_seq
        if not bucket_controller_enabled:
            return
        request_id = f"bucket-{bucket_command_seq}"
        env = bus.make_envelope(proc.proc)
        env.update(
            {
                "action": action,
                "request_id": request_id,
                "reason": reason,
            }
        )
        if team is not None:
            env["team"] = team
        if bucket_number is not None:
            env["bucket_number"] = bucket_number
            env["bucket_label"] = f"{team.upper()}{bucket_number}" if team else None
        bus.publish(pub, BUCKET_COMMAND_TOPIC, env)
        bucket_command_seq += 1

    def _publish_weight_tare(reason: str) -> None:
        """Publish one tare command for the weight_sensor_io process."""

        nonlocal weight_tare_seq
        if not weight_sensor_enabled:
            return
        request_id = f"weight-tare-{weight_tare_seq}"
        env = bus.make_envelope(proc.proc)
        env.update({"request_id": request_id, "reason": reason})
        bus.publish(pub, WEIGHT_TARE_TOPIC, env)
        weight_tare_seq += 1

    def tick(p: Proc) -> None:
        nonlocal state_seq
        # Tick flow summary:
        # 1) Ingest operator inputs + safety + latest telem from haptic/robot.
        # 2) Publish assistive haptic command (cmd.haptic.<team>) with the
        #    latest computed bounds (stale by <=1 tick in normal play).
        # 3) Plan robot target (collision-aware) and publish
        #    cmd.robot.target.<team>.
        # 4) Publish one authoritative state.full snapshot for UIs and
        #    downstream process consumers.
        now_ns = time.perf_counter_ns()
        _drain_operator_input_requests(
            operator_input_rep,
            on_msg=lambda body: _handle_operator_input_request(
                control_state,
                stage_state,
                teams,
                body,
                time.perf_counter_ns(),
                producer=p.proc,
                recovery_timeout_s=RECOVERY_TIMEOUT_S,
            ),
        )
        _publish_pending_recovery_requests(
            pub,
            p.proc,
            control_state,
            recovery_timeout_s=RECOVERY_TIMEOUT_S,
        )

        if safety_sub is not None:
            _drain_latest(
                safety_sub, on_msg=lambda body: _update_safety_state(safety_state, body)
            )
        if weight_sub is not None:
            _drain_latest(
                weight_sub, on_msg=lambda body: _update_weight_state(weight_state, body)
            )
        _refresh_safety_block(control_state, safety_state, safety_telem_age_max_s)

        if bool(control_state.get("recovery_active", False)):
            deadline_ns = control_state.get("recovery_deadline_mono_ns")
            if isinstance(deadline_ns, int) and now_ns > deadline_ns:
                control_state["recovery_active"] = False
                control_state["recovery_pending_dispatch"] = False
                control_state["last_action"] = "play_resume_timeout"
                control_state["last_action_ts_mono_ns"] = now_ns

        soft_paused = bool(control_state.get("soft_pause", False))
        for team, st in teams.items():
            _drain_latest(
                st["sub_haptic"], on_msg=lambda b, s=st: _update_haptic_state(s, b)
            )
            _drain_latest(
                st["sub_actual"], on_msg=lambda b, s=st: _update_actual_state(s, b)
            )

            planner: InProcessPlanner = st["planner"]
            # Only re-seed once we've actually received a measured
            # pose; otherwise the planner keeps its home pose.
            if st["last_q"] is not None:
                planner.seed(st["last_q"])

            now = time.perf_counter()
            dt = now - st["last_tick_t"]
            st["last_tick_t"] = now
            # Cap dt: a long stall (debugger, GC pause) shouldn't push
            # a huge accel-clamped velocity jump on the next tick.
            if dt > 0.1:
                dt = 0.1

            if stage_state["stage"] == "play":
                _apply_weight_bucket_values(st, weight_state)
                st["rewind"].ensure_recording_started(
                    st["last_q"], now_s=float(now_ns) / 1e9
                )

            if st["last_q"] is None:
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                _reset_team_motion_outputs(st, q_target_rad=None)
                st["score"] = int(sum(st["bucket_values"]))
                continue

            if bool(st.get("haptic_required", False)) and not bool(
                st.get("haptic_seeded", False)
            ):
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                _publish_hold_current_pose(pub, p.proc, team, st)
                continue

            if _startup_alignment_active(st):
                # Keep publishing tracking during alignment so boards have
                # a coherent target immediately after digital reseat.
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                _publish_haptic_command(pub, p.proc, team, st, haptic_cfg)
                _tick_startup_alignment(
                    pub, p.proc, team, st, haptic_cfg, now=time.perf_counter()
                )

                # Hold robot at measured pose until haptic settles to avoid startup jerk.
                _publish_hold_current_pose(pub, p.proc, team, st)
                continue

            play_sync = st.get("play_sync", {})
            if stage_state["stage"] == "play" and (
                bool(play_sync.get("requested", False))
                or bool(play_sync.get("pending", False))
            ):
                sync_now_s = time.perf_counter()
                if bool(play_sync.get("requested", False)):
                    _begin_play_sync(
                        pub,
                        p.proc,
                        team,
                        st,
                        haptic_cfg,
                        now=sync_now_s,
                    )
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                _publish_haptic_command(pub, p.proc, team, st, haptic_cfg)
                sync_ready = _tick_play_sync(
                    pub,
                    p.proc,
                    team,
                    st,
                    haptic_cfg,
                    now=sync_now_s,
                )
                if sync_ready:
                    planner.reseed(st["last_q"], dial_pos_rad=st["last_dial"])
                _publish_hold_current_pose(pub, p.proc, team, st)
                continue

            if stage_state["stage"] == "tutorial":
                # Tutorial: the robot holds its measured pose while each player
                # scrolls their dial. _tick_tutorial_team performs the one-shot
                # reseat-to-zero + bounds install on entry, refreshes per-player
                # progress, and publishes the snap-to-detent haptic command.
                _tick_tutorial_team(
                    pub, p.proc, team, st, haptic_cfg, game_cfg
                )
                _publish_hold_current_pose(pub, p.proc, team, st)
                continue

            if stage_state["stage"] == "reset":
                # Rewind keeps position tracking active but discards stale
                # proximity-derived bounds from the final gameplay tick.
                _reset_haptic_bounds_to_static(st, haptic_cfg)
            _publish_haptic_command(pub, p.proc, team, st, haptic_cfg)

            robot_status = st.get("robot_status", {})
            robot_fault_active = bool(robot_status.get("fault_active", False))
            fault_prev_by_team = control_state.setdefault(
                "fault_active_prev_by_team", {}
            )
            was_fault_active = bool(fault_prev_by_team.get(team, False))
            if robot_fault_active and not was_fault_active:
                # Latch into soft e-stop on new robot fault so the game
                # only resumes on an explicit PLAY/RESUME action.
                control_state["soft_pause"] = True
                control_state["last_action"] = "soft_estop"
                control_state["last_action_ts_mono_ns"] = now_ns
                soft_paused = True
            fault_prev_by_team[team] = robot_fault_active
            if robot_fault_active or soft_paused:
                # Keep the planner anchored to measured robot state while no
                # motion can execute. This prevents target position and
                # velocity from surviving a protective stop and jumping on
                # the first resumed tick.
                planner.reseed(st["last_q"], dial_pos_rad=st["last_dial"])
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                _publish_hold_current_pose(pub, p.proc, team, st)
                continue

            if stage_state["stage"] == "play":
                st["score"] = int(sum(st["bucket_values"]))
            elif stage_state["stage"] == "reset" and bool(
                game_cfg.get("rewind_enabled", False)
            ):
                # Reset ignores haptic input and collision/proximity checks.
                # The haptic command published above still tracks measured q.
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                q_target = st["rewind"].next_target(
                    dt_s=dt,
                    q_actual_rad=st["last_q"],
                )
                if q_target is None:
                    _publish_hold_current_pose(pub, p.proc, team, st)
                    continue
                _reset_team_motion_outputs(st, q_target_rad=q_target)
                env = bus.make_envelope(p.proc)
                env.update(
                    {
                        "team": team,
                        "q_target_rad": q_target,
                        "clamps": {"path": 1.0, "prox": 1.0, "final": 1.0},
                    }
                )
                bus.publish(pub, f"cmd.robot.target.{team}", env)
                continue
            else:
                # All non-play stages hold the robot at its measured pose.
                # Only the conclusion stage additionally advances the scripted
                # scoring sequence; daydreaming / idle / tutorial / reset just
                # hold until their transition fires.
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                if stage_state["stage"] == "conclusion":
                    _tick_conclusion_team(
                        st,
                        dt,
                        game_cfg,
                        robot_show_poses.get(team, {}),
                        stage_state,
                        bucket_command_fn=_publish_bucket_command,
                    )
                _publish_hold_current_pose(pub, p.proc, team, st)
                continue

            q_target, info = planner.plan(
                dial_pos_rad=st["last_dial"],
                dt=dt,
            )
            st["last_target"] = q_target
            st["last_collision"] = info.get("collision", False)
            st["last_first_hit"] = info.get("collision_first_hit")
            st["last_path_scalar"] = float(info.get("path_scalar", 1.0))
            st["last_prox_scalar"] = float(info.get("prox_scalar", 1.0))
            st["last_final_scalar"] = float(info.get("final_scalar", 1.0))
            st["last_planner_info"] = dict(info)
            st["last_prox_probe_offsets_deg"] = list(
                info.get("prox_probe_offsets_deg") or []
            )
            raw_hits = (
                info.get("prox_hits") if isinstance(info.get("prox_hits"), list) else []
            )
            st["last_prox_hits"] = [
                [bool(v) for v in axis_hits] if isinstance(axis_hits, list) else []
                for axis_hits in raw_hits[:6]
            ]
            while len(st["last_prox_hits"]) < 6:
                st["last_prox_hits"].append([])
            raw_ages = (
                info.get("prox_age_ticks")
                if isinstance(info.get("prox_age_ticks"), list)
                else []
            )
            st["last_prox_age_ticks"] = [int(v) for v in raw_ages[:6]] + [9999] * max(
                0, 6 - len(raw_ages[:6])
            )
            st["last_prox_age_ticks"] = st["last_prox_age_ticks"][:6]
            _update_dynamic_haptic_bounds_from_prox(st, haptic_cfg)

            env = bus.make_envelope(p.proc)
            env.update(
                {
                    "team": team,
                    "q_target_rad": q_target,
                    "clamps": {
                        "path": st["last_path_scalar"],
                        "prox": st["last_prox_scalar"],
                        "final": st["last_final_scalar"],
                    },
                }
            )
            bus.publish(pub, f"cmd.robot.target.{team}", env)
            st["rewind"].record_target(
                q_target,
                now_s=float(now_ns) / 1e9,
            )

        if bool(control_state.get("recovery_active", False)):
            recovery_teams = [
                team
                for team in list(control_state.get("recovery_teams", []))
                if team in teams
            ]
            recovered = bool(recovery_teams) and all(
                _is_robot_status_recovered(teams[team].get("robot_status", {}))
                for team in recovery_teams
            )
            if recovered:
                control_state["recovery_active"] = False
                control_state["recovery_pending_dispatch"] = False
                if not bool(control_state.get("safety_blocked", False)) and not bool(
                    control_state.get("safety_pause_latched", False)
                ):
                    control_state["soft_pause"] = False
                    control_state["last_action"] = "play_resume"
                    control_state["last_action_ts_mono_ns"] = now_ns
                    soft_paused = False

        paused, pause_reason = _pause_state_summary(
            control_state,
            safety_state,
            teams,
            soft_paused=soft_paused,
        )

        if paused != bool(stage_state.get("prev_paused", False)):
            print(
                f"[game_controller] PAUSE {'ON' if paused else 'OFF'}"
                + (f" reason={pause_reason}" if paused else ""),
                flush=True,
            )
            stage_state["prev_paused"] = paused

        _update_stage_pause_tracking(stage_state, paused, now_ns)
        stage_before_tick = stage_state["stage"]
        if not paused:
            batch_rewind_complete = (
                batch_session is not None
                and stage_state["stage"] == "reset"
                and bool(teams)
                and all(st["rewind"].complete for st in teams.values())
            )
            if batch_rewind_complete and not batch_session.shutdown_requested:
                completed_index = batch_session.game_index
                start_next = batch_session.record_completed_game(teams)
                print(
                    f"[batch-validation] completed game={completed_index}/"
                    f"{batch_cfg.game_count} report={batch_cfg.output_jsonl}",
                    flush=True,
                )
                if start_next:
                    _prepare_next_batch_game()
                    _enter_stage(
                        stage_state,
                        teams,
                        "tutorial",
                        game_cfg,
                        now_ns,
                        reason="batch auto-restart",
                    )
                elif batch_cfg.shutdown_when_complete:
                    batch_session.shutdown_requested = True
                else:
                    _enter_stage(
                        stage_state,
                        teams,
                        "conclusion",
                        game_cfg,
                        now_ns,
                        reason="batch complete",
                    )
            elif batch_session is None:
                _tick_stage_state(stage_state, teams, game_cfg, now_ns)
            elif not batch_session.shutdown_requested:
                _tick_stage_state(stage_state, teams, game_cfg, now_ns)
        if batch_session is not None and batch_session.shutdown_requested:
            _publish_batch_shutdown()
        if stage_before_tick != "play" and stage_state["stage"] == "play":
            # Stage transitions happen after the per-team motion loop. Issue
            # the coordinate-reset command now, on that same transition tick;
            # the following ticks hold until haptic telemetry confirms it.
            for team, st in teams.items():
                _begin_play_sync(
                    pub,
                    p.proc,
                    team,
                    st,
                    haptic_cfg,
                    now=time.perf_counter(),
                )
            if batch_session is not None:
                batch_session.mark_play_started()
        if stage_before_tick == "conclusion" and stage_state["stage"] != "conclusion":
            # Leaving conclusion (now conclusion -> idle): close any buckets
            # opened during scoring and tare the load cells for the next game.
            # TODO(conclusion-motion): coordinate close_all with the real
            # return-to-start trajectory once that motion exists.
            _publish_bucket_command("close_all", reason="conclusion_reset")
            # TODO(reset-flow): tare after close_all completes once bucket
            # close completion is tracked by GC.
            _publish_weight_tare("conclusion_reset")

        countdown_s = _stage_countdown_s(stage_state, game_cfg, now_ns)

        env = bus.make_envelope(p.proc, with_wall=True, seq=state_seq)
        env.update(
            _build_state_full_payload(
                stage_state,
                safety_state,
                weight_state,
                teams,
                game_cfg,
                haptic_cfg,
                paused=paused,
                pause_reason=pause_reason,
                soft_paused=soft_paused,
                countdown_s=countdown_s,
            )
        )
        if batch_session is not None:
            env["batch_validation"] = {
                "enabled": True,
                "game_index": batch_session.game_index,
                "game_count": batch_cfg.game_count,
                "gameplay_seed": batch_session.gameplay_seed(),
                "shortcut_seed": batch_session.shortcut_seed(),
                "shutdown_requested": batch_session.shutdown_requested,
            }
        bus.publish(pub, "state.full", env)
        state_seq += 1

    def teardown(_: Proc) -> None:
        if batch_session is not None:
            batch_session.close()
        for st in teams.values():
            st["rewind"].close()
            st["planner"].close()
            st["sub_haptic"].close(0)
            st["sub_actual"].close(0)
        if safety_sub is not None:
            safety_sub.close(0)
        if weight_sub is not None:
            weight_sub.close(0)
        operator_input_rep.close(0)

    return proc.run(tick, teardown=teardown)


def _drain_latest(sub: zmq.Socket, *, on_msg) -> None:
    """Drain every queued message on a SUB; call on_msg with the last body."""
    last = None
    while True:
        try:
            _, body = bus.recv(sub, flags=zmq.NOBLOCK)
            last = body
        except zmq.Again:
            break
    if last is not None:
        on_msg(last)


def _update_actual_state(state: dict[str, Any], body: dict[str, Any]) -> None:
    """Cache the latest measured robot joints and status for one team."""

    state["last_q"] = body.get("q_rad", state["last_q"])
    robot_status = body.get("robot_status")
    if isinstance(robot_status, dict):
        state["robot_status"] = robot_status


if __name__ == "__main__":
    sys.exit(main())
