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

import yaml  # noqa: E402
import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting  # noqa: E402
from core.proc import Proc, banner  # noqa: E402
from subsystems.jogging.in_process import InProcessPlanner  # noqa: E402

# game_controller ticks at a fixed rate. Each tick blocks on the
# forward-collision certify (~worker compute + ZMQ round-trip), then
# publishes one cmd.robot.target.<team> per team and one state.full.
# 60 Hz gives ~16 ms per tick, which is comfortably above the
# forward_timeout_ms budget and keeps state.full at a recordable rate.
TICK_HZ = 60.0
TEAM_BUCKET_IDS = {
    "a": [11, 12, 13],
    "b": [21, 22, 23],
}
DEFAULT_BUCKET_VALUES = [0.0, 0.0, 0.0]
DEFAULT_LOOK_POSE_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
DEFAULT_HAPTIC_BOUNDS_DEG_MIN = [-180.0] * 6
DEFAULT_HAPTIC_BOUNDS_DEG_MAX = [180.0] * 6
DEFAULT_SAFETY_TELEM_AGE_MAX_MS = 1100.0
CONCLUSION_INITIAL_PAUSE_S = 1.0
CONCLUSION_BUCKET_EMPTY_PAUSE_S = 0.5
CONCLUSION_ANNOUNCEMENT_PAUSE_S = 1.0
UI_GAME_CONTROL_TOPIC = "cmd.ui.game_control"
RECOVERY_TIMEOUT_S = 4.0


def main(argv: list[str] | None = None) -> int:
    """Run the central game loop, state machine, and robot command publisher."""
    proc, _ = Proc.from_argv(target_hz=TICK_HZ, default_proc="game_controller")

    active_teams = list(proc.profile.active_teams)
    game_cfg = _game_config(proc.profile.tuning.get("game"))
    haptic_cfg = _haptic_config(proc.profile.tuning.get("haptic"))
    safety_enabled = (
        proc.profile.subsystem_impl("safety_barrier_controller") is not None
    )
    safety_telem_age_max_s = (
        default_runtime_setting(
            "safety_barrier_controller",
            "telem_age_max",
            DEFAULT_SAFETY_TELEM_AGE_MAX_MS,
        )
        or DEFAULT_SAFETY_TELEM_AGE_MAX_MS
    ) / 1000.0
    robot_show_poses = _load_robot_show_poses_deg()
    pub = bus.make_pub(proc.ctx)
    control_rep = bus.make_rep(proc.ctx)
    safety_sub = (
        bus.make_sub(proc.ctx, topics=["telem.safety"]) if safety_enabled else None
    )
    proc.use_heartbeat_pub(pub)

    # P2 ships team-A only; team-B wiring is symmetric and lands when
    # the second arm joins.
    if "a" not in active_teams:
        banner(
            proc.proc, "no active teams; will only emit heartbeat + skeleton state.full"
        )

    collision_enabled = (
        isinstance(proc.profile.subsystems.get("collision_workers"), dict)
        and int(proc.profile.subsystems["collision_workers"].get("count", 0)) > 0
    )

    # Per-team state. P2 builds only `a`; the structure generalizes.
    teams: dict[str, dict] = {}
    for team in active_teams:
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
            "team": team,
            "sub_haptic": sub,
            "sub_actual": actual_sub,
            "last_dial": [0.0] * 6,
            "last_dial_vel": [0.0] * 6,
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
            "last_prox_probe_offsets_deg": [],
            "last_prox_hits": [[False] * 20 for _ in range(6)],
            "last_prox_age_ticks": [9999] * 6,
            "robot_status": {},
            "bucket_ids": list(TEAM_BUCKET_IDS.get(team, [])),
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
        }
    banner(proc.proc, f"teams={active_teams} collision_check={collision_enabled}")

    state_seq = 0
    # TODO(state-machine): once Idle/Tutorial are wired, start there instead
    # of jumping straight into Play on boot.
    stage_state = {
        "stage": "play",
        "stage_entered_mono_ns": time.perf_counter_ns(),
        "winner_team": None,
        "pause_started_mono_ns": None,
        "paused_total_ns": 0,
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

    def tick(p: Proc) -> None:
        nonlocal state_seq
        # Tick flow summary:
        # 1) Ingest UI control + safety + latest telem from haptic/robot.
        # 2) Publish assistive haptic command (cmd.haptic.<team>) with the
        #    latest computed bounds (stale by <=1 tick in normal play).
        # 3) Plan robot target (collision-aware) and publish
        #    cmd.robot.target.<team>.
        # 4) Publish one authoritative state.full snapshot for UIs and
        #    downstream process consumers.
        now_ns = time.perf_counter_ns()
        _drain_ui_game_control_requests(
            control_rep,
            on_msg=lambda body: _handle_ui_game_control_request(
                control_state,
                stage_state,
                teams,
                body,
                time.perf_counter_ns(),
            ),
        )
        _maybe_publish_recovery_request(pub, p.proc, control_state)

        if safety_sub is not None:
            _drain_latest(
                safety_sub, on_msg=lambda body: _update_safety_state(safety_state, body)
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

            if st["last_q"] is None:
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                st["last_target"] = None
                st["last_collision"] = False
                st["last_first_hit"] = None
                st["last_path_scalar"] = 1.0
                st["last_prox_scalar"] = 1.0
                st["last_final_scalar"] = 1.0
                st["last_prox_probe_offsets_deg"] = []
                st["last_prox_hits"] = [[False] * 20 for _ in range(6)]
                st["last_prox_age_ticks"] = [9999] * 6
                st["score"] = int(sum(st["bucket_values"]))
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
                st["last_target"] = list(st["last_q"])
                st["last_collision"] = False
                st["last_first_hit"] = None
                st["last_path_scalar"] = 1.0
                st["last_prox_scalar"] = 1.0
                st["last_final_scalar"] = 1.0
                st["last_prox_probe_offsets_deg"] = []
                st["last_prox_hits"] = [[False] * 20 for _ in range(6)]
                st["last_prox_age_ticks"] = [9999] * 6

                env = bus.make_envelope(p.proc)
                env.update(
                    {
                        "team": team,
                        "q_target_rad": list(st["last_q"]),
                        "clamps": {
                            "path": 1.0,
                            "prox": 1.0,
                            "final": 1.0,
                        },
                    }
                )
                bus.publish(pub, f"cmd.robot.target.{team}", env)
                continue

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
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                st["last_target"] = list(st["last_q"])
                st["last_collision"] = False
                st["last_first_hit"] = None
                st["last_path_scalar"] = 1.0
                st["last_prox_scalar"] = 1.0
                st["last_final_scalar"] = 1.0
                st["last_prox_probe_offsets_deg"] = []
                st["last_prox_hits"] = [[False] * 20 for _ in range(6)]
                st["last_prox_age_ticks"] = [9999] * 6

                env = bus.make_envelope(p.proc)
                env.update(
                    {
                        "team": team,
                        "q_target_rad": list(st["last_q"]),
                        "clamps": {
                            "path": 1.0,
                            "prox": 1.0,
                            "final": 1.0,
                        },
                    }
                )
                bus.publish(pub, f"cmd.robot.target.{team}", env)
                continue

            if stage_state["stage"] == "play":
                st["score"] = int(sum(st["bucket_values"]))
            else:
                _reset_haptic_bounds_to_static(st, haptic_cfg)
                _tick_conclusion_team(
                    st, dt, game_cfg, robot_show_poses.get(team, {}), stage_state
                )
                st["last_target"] = list(st["last_q"])
                st["last_collision"] = False
                st["last_first_hit"] = None
                st["last_path_scalar"] = 1.0
                st["last_prox_scalar"] = 1.0
                st["last_final_scalar"] = 1.0
                st["last_prox_probe_offsets_deg"] = []
                st["last_prox_hits"] = [[False] * 20 for _ in range(6)]
                st["last_prox_age_ticks"] = [9999] * 6

                env = bus.make_envelope(p.proc)
                env.update(
                    {
                        "team": team,
                        "q_target_rad": list(st["last_q"]),
                        "clamps": {
                            "path": 1.0,
                            "prox": 1.0,
                            "final": 1.0,
                        },
                    }
                )
                bus.publish(pub, f"cmd.robot.target.{team}", env)
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

        if bool(control_state.get("recovery_active", False)):
            recovery_teams = [
                team
                for team in list(control_state.get("recovery_teams", []))
                if team in teams
            ]
            recovered = bool(recovery_teams) and all(
                _robot_status_recovered(teams[team].get("robot_status", {}))
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

        # Emit a (very small) state.full snapshot per BUS.md 禮6.1.
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

        _update_stage_pause_tracking(stage_state, paused, now_ns)
        if not paused:
            _tick_stage_state(stage_state, teams, game_cfg, now_ns)

        countdown_s = _stage_countdown_s(stage_state, game_cfg, now_ns)

        env = bus.make_envelope(p.proc, with_wall=True, seq=state_seq)
        env.update(
            {
                "stage": "paused" if paused else stage_state["stage"],
                "active_stage": stage_state["stage"],
                "paused": paused,
                "pause_reason": pause_reason,
                "soft_estop": soft_paused,
                "safety": {
                    "barrier": _state_full_safety_barrier(safety_state),
                },
                "countdown_s": countdown_s,
                "game_duration_s": game_cfg["duration_s"],
                "sum_score_rate_unit_per_s": game_cfg["sum_score_rate_unit_per_s"],
                "stage_entered_mono_ns": stage_state["stage_entered_mono_ns"],
                "tutorial_entered_wall_ns": None,
                "teams": {
                    team: {
                        "robot": {
                            "q_target_rad": st["last_target"],
                            "q_rad": (
                                st["last_q"] if st["last_q"] is not None else [0.0] * 6
                            ),
                            "status": st.get("robot_status", {}),
                        },
                        "haptic": {
                            "dial_pos_rad": st["last_dial"],
                            "dial_vel_rad_s": st["last_dial_vel"],
                            "connected": st["last_haptic_connected"],
                            "board_loop_hz": st["last_haptic_loop_hz"],
                            "bounds_min_rad": list(
                                st.get("current_haptic_bounds_min_rad")
                                or haptic_cfg["bounds_min_rad"]
                            ),
                            "bounds_max_rad": list(
                                st.get("current_haptic_bounds_max_rad")
                                or haptic_cfg["bounds_max_rad"]
                            ),
                        },
                        "collision": {
                            "in_collision": st["last_collision"],
                            "first_hit": st["last_first_hit"],
                            "path_scalar": st["last_path_scalar"],
                            "prox_scalar": st["last_prox_scalar"],
                            "final_scalar": st["last_final_scalar"],
                            "prox_probe_offsets_deg": st["last_prox_probe_offsets_deg"],
                            "prox_hits": st["last_prox_hits"],
                            "prox_age_ticks": st["last_prox_age_ticks"],
                        },
                        "score": st["score"],
                        "summed_score": st["summed_score"],
                        "buckets": list(st["bucket_values"]),
                        "conclusion": {
                            "phase": st["conclusion_phase"],
                            "active_bucket_index": st["conclusion_active_bucket_index"],
                            "target_pose_name": st["conclusion_target_pose_name"],
                            "target_pose_deg": st["conclusion_target_pose_deg"],
                            "bucket_open_triggered": st[
                                "conclusion_bucket_open_triggered"
                            ],
                            "done": st["conclusion_done"],
                        },
                    }
                    for team, st in teams.items()
                },
            }
        )
        bus.publish(pub, "state.full", env)
        state_seq += 1

    def teardown(_: Proc) -> None:
        for st in teams.values():
            st["planner"].close()
            st["sub_haptic"].close(0)
            st["sub_actual"].close(0)
        if safety_sub is not None:
            safety_sub.close(0)
        control_rep.close(0)

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


def _initial_safety_state(*, enabled: bool) -> dict[str, Any]:
    """Return the GameController's local safety barrier cache."""

    return {
        "enabled": enabled,
        "ok": True if not enabled else False,
        "channels": [],
        "errors": [],
        "last_recv_mono_s": None,
        "stale": enabled,
    }


def _update_safety_state(state: dict[str, Any], body: dict[str, Any]) -> None:
    """Store the latest `telem.safety` payload and local receipt time."""

    state["ok"] = bool(body.get("ok", False))
    channels = body.get("channels")
    state["channels"] = (
        [bool(value) for value in channels] if isinstance(channels, list) else []
    )
    errors = body.get("errors")
    state["errors"] = (
        [str(value) for value in errors] if isinstance(errors, list) else []
    )
    state["last_recv_mono_s"] = time.perf_counter()
    state["stale"] = False


def _refresh_safety_block(
    control_state: dict[str, Any],
    safety_state: dict[str, Any],
    telem_age_max_s: float,
) -> None:
    """Update safety pause flags from the latest barrier sample and age budget."""

    if not bool(safety_state.get("enabled", False)):
        control_state["safety_blocked"] = False
        safety_state["stale"] = False
        return

    last_recv = safety_state.get("last_recv_mono_s")
    stale = (
        not isinstance(last_recv, float)
        or (time.perf_counter() - last_recv) > telem_age_max_s
    )
    safety_state["stale"] = stale
    blocked = stale or not bool(safety_state.get("ok", False))
    control_state["safety_blocked"] = blocked
    if blocked:
        control_state["soft_pause"] = True
        control_state["safety_pause_latched"] = True


def _safety_pause_reason(safety_state: dict[str, Any]) -> str:
    """Return the pause reason string for the current safety block."""

    if bool(safety_state.get("stale", False)):
        return "barrier_stale"
    return "barrier_open"


def _state_full_safety_barrier(safety_state: dict[str, Any]) -> dict[str, Any]:
    """Build the `state.full.safety.barrier` block for UI and RobotIO consumers."""

    return {
        "enabled": bool(safety_state.get("enabled", False)),
        "ok": bool(safety_state.get("ok", True))
        and not bool(safety_state.get("stale", False)),
        "channels": list(safety_state.get("channels", [])),
        "stale": bool(safety_state.get("stale", False)),
        "errors": list(safety_state.get("errors", [])),
    }


def _drain_ui_game_control_requests(rep: zmq.Socket, *, on_msg) -> None:
    """Drain pending UI admin requests and reply to each one.

    The GC owns the bind-side REP socket, so every received request must
    produce exactly one reply before the next request can be received.
    """
    while True:
        try:
            body = bus.recv_json(rep, flags=zmq.NOBLOCK)
        except zmq.Again:
            break
        bus.send_json(rep, on_msg(body))


def _update_actual_state(state: dict, body: dict) -> None:
    state["last_q"] = body.get("q_rad", state["last_q"])
    robot_status = body.get("robot_status")
    if isinstance(robot_status, dict):
        state["robot_status"] = robot_status


def _update_haptic_state(state: dict, body: dict) -> None:
    state["last_dial"] = body.get("dial_pos_rad", state["last_dial"])
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


def _game_config(node: Any) -> dict[str, float]:
    data = node if isinstance(node, dict) else {}
    return {
        "duration_s": _coerce_positive_float(data.get("duration_s"), 240.0),
        "sum_score_rate_unit_per_s": _coerce_positive_float(
            data.get("sum_score_rate_unit_per_s"), 100.0
        ),
        "sim_bucket_values": _coerce_team_bucket_values(data.get("sim_bucket_values")),
    }


def _haptic_config(node: Any) -> dict[str, Any]:
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
            1, int(_coerce_positive_float(data.get("startup_settle_streak_ticks"), 3.0))
        ),
    }


def _publish_haptic_command(
    pub: zmq.Socket,
    producer: str,
    team: str,
    state: dict[str, Any],
    haptic_cfg: dict[str, Any],
) -> None:
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
    static_max = [float(v) for v in haptic_cfg.get("bounds_max_rad", [math.pi] * 6)[:6]]
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
        # Each axis may use a different dial-to-joint gear ratio; convert
        # both endpoints and sort so negative gearing still yields lo <= hi.
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
    env = bus.make_envelope(producer)
    env.update(
        {
            "team": team,
            "current_pos_rad": list(current_pos_robot_rad),
            "current_pos_dial_rad": list(current_pos_dial_rad),
        }
    )
    bus.publish(pub, f"cmd.haptic.reseat.{team}", env)


def _startup_alignment_active(state: dict[str, Any]) -> bool:
    align = state.get("startup_align") if isinstance(state, dict) else None
    if not isinstance(align, dict):
        return False
    return bool(align.get("enabled", False)) and not bool(align.get("done", False))


def _tick_startup_alignment(
    pub: zmq.Socket,
    producer: str,
    team: str,
    state: dict[str, Any],
    haptic_cfg: dict[str, Any],
    *,
    now: float,
) -> None:
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

    settled, max_err = _haptic_settled_to_target(state, q_dial, haptic_cfg)
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


def _haptic_settled_to_target(
    state: dict[str, Any], target_dial_rad: list[float], haptic_cfg: dict[str, Any]
) -> tuple[bool, float]:
    dial = list(state.get("last_dial") or [0.0] * 6)[:6]
    conn = list(state.get("last_haptic_connected") or [False] * 6)[:6]
    while len(dial) < 6:
        dial.append(0.0)
    while len(conn) < 6:
        conn.append(False)

    # Require all six dials connected before declaring startup settled.
    if not all(bool(v) for v in conn[:6]):
        return False, float("inf")

    tol = float(haptic_cfg.get("startup_settle_tol_rad", math.radians(10.0)))
    max_err = 0.0
    for i in range(6):
        err = abs(float(dial[i]) - float(target_dial_rad[i]))
        if err > max_err:
            max_err = err
    return max_err <= tol, max_err


def _tick_stage_state(
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    game_cfg: dict[str, float],
    now_ns: int,
) -> None:
    if stage_state["stage"] == "play":
        elapsed_s = _stage_elapsed_s(stage_state, now_ns)
        if elapsed_s >= game_cfg["duration_s"]:
            _force_conclusion(stage_state, teams, now_ns)
        return

    if stage_state["stage"] != "conclusion":
        return

    announcement_ready = {"announcement_pose", "winner_pose"}
    if stage_state["winner_team"] is None and all(
        bool(st.get("conclusion_done", False))
        or str(st.get("conclusion_phase")) in announcement_ready
        for st in teams.values()
    ):
        stage_state["winner_team"] = _winner_team(teams)

    if all(bool(st.get("conclusion_done", False)) for st in teams.values()):
        # Temporary P4 short-circuit: once the conclusion sequence has
        # fully finished for every team, jump straight back into play so
        # repeated dev runs do not need an external reset path yet.
        # TODO(state-machine): implement a proper Idle/Tutorial stage and transition
        #                      to that instead of hard-resetting play on conclusion end.
        _reset_for_play(stage_state, teams, game_cfg, now_ns)


def _reset_for_play(
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    game_cfg: dict[str, float],
    now_ns: int,
) -> None:
    stage_state["stage"] = "play"
    stage_state["stage_entered_mono_ns"] = now_ns
    stage_state["winner_team"] = None
    stage_state["pause_started_mono_ns"] = None
    stage_state["paused_total_ns"] = 0

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


def _stage_countdown_s(
    stage_state: dict[str, Any], game_cfg: dict[str, float], now_ns: int
) -> int:
    if stage_state["stage"] != "play":
        return 0
    remaining_s = game_cfg["duration_s"] - _stage_elapsed_s(stage_state, now_ns)
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


def _force_conclusion(
    stage_state: dict[str, Any], teams: dict[str, dict], now_ns: int
) -> None:
    if stage_state["stage"] == "conclusion":
        return
    stage_state["stage"] = "conclusion"
    stage_state["stage_entered_mono_ns"] = now_ns
    stage_state["winner_team"] = None
    for st in teams.values():
        _enter_conclusion(st, now_ns)


def _handle_ui_game_control_request(
    control_state: dict[str, Any],
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    body: dict[str, Any],
    now_ns: int,
) -> dict[str, Any]:
    """Apply one UI request with request-id dedupe and envelope formatting."""
    request = body if isinstance(body, dict) else {}
    source = (
        request.get("source") if isinstance(request.get("source"), str) else "unknown"
    )
    request_id = (
        request.get("request_id")
        if isinstance(request.get("request_id"), (int, str))
        else None
    )

    last_request_id_by_source = control_state.setdefault(
        "last_request_id_by_source", {}
    )
    last_reply_by_source = control_state.setdefault("last_reply_by_source", {})
    if request_id is not None and last_request_id_by_source.get(source) == request_id:
        cached = last_reply_by_source.get(source)
        if isinstance(cached, dict):
            return dict(cached)

    ok, error, action = _apply_ui_game_control(
        control_state, stage_state, teams, request, now_ns
    )
    reply = bus.make_envelope("game_controller", with_wall=True)
    reply.update(
        {
            "ok": ok,
            "error": error,
            "request_id": request_id,
            "source": source,
            "result": {
                "action": action,
                "soft_estop": bool(control_state.get("soft_pause", False)),
                "active_stage": stage_state["stage"],
                "last_action": control_state.get("last_action"),
            },
        }
    )
    if request_id is not None:
        last_request_id_by_source[source] = request_id
        last_reply_by_source[source] = dict(reply)
    return reply


def _apply_ui_game_control(
    control_state: dict[str, Any],
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    body: dict[str, Any],
    now_ns: int,
) -> tuple[bool, str | None, str | None]:
    """Apply one UI control request and update pause/recovery state."""
    action = body.get("action") if isinstance(body, dict) else None
    if not isinstance(action, str):
        return False, "missing action", None
    if action == "play_resume":
        if bool(control_state.get("safety_blocked", False)):
            control_state["soft_pause"] = True
            return False, "safety barrier is not clear", action
        recovery_teams = [
            team
            for team, st in teams.items()
            if _robot_status_needs_recovery(st.get("robot_status", {}))
        ]
        if recovery_teams:
            control_state["soft_pause"] = True
            control_state["safety_pause_latched"] = False
            control_state["recovery_active"] = True
            control_state["recovery_deadline_mono_ns"] = now_ns + int(
                RECOVERY_TIMEOUT_S * 1e9
            )
            control_state["recovery_pending_dispatch"] = True
            control_state["recovery_request_id"] = (
                int(control_state.get("recovery_request_id", 0)) + 1
            )
            control_state["recovery_teams"] = list(recovery_teams)
        else:
            control_state["soft_pause"] = False
            control_state["safety_pause_latched"] = False
            control_state["recovery_active"] = False
            control_state["recovery_pending_dispatch"] = False
            control_state["recovery_teams"] = []
    elif action == "soft_estop":
        control_state["soft_pause"] = True
        control_state["recovery_active"] = False
        control_state["recovery_pending_dispatch"] = False
        control_state["recovery_teams"] = []
    elif action == "end_game":
        control_state["soft_pause"] = False
        control_state["recovery_active"] = False
        control_state["recovery_pending_dispatch"] = False
        control_state["recovery_teams"] = []
        _force_conclusion(stage_state, teams, now_ns)
    else:
        return False, f"unsupported action: {action}", action
    control_state["last_action"] = action
    control_state["last_action_ts_mono_ns"] = now_ns
    return True, None, action


def _robot_status_needs_recovery(status: dict[str, Any]) -> bool:
    """Return whether a robot status still needs an explicit recovery request."""
    if bool(status.get("fault_active", False)):
        return True
    if status.get("program_running") is False:
        return True
    return not bool(status.get("control_ok", True))


def _robot_status_recovered(status: dict[str, Any]) -> bool:
    """Return whether a robot status is good enough to clear recovery pause."""
    if bool(status.get("fault_active", False)):
        return False
    if not bool(status.get("control_ok", False)):
        return False
    return status.get("program_running") is not False


def _maybe_publish_recovery_request(
    pub: zmq.Socket, producer: str, control_state: dict[str, Any]
) -> None:
    """Publish one recovery command per requested team, once per resume attempt."""
    if not bool(control_state.get("recovery_active", False)):
        return
    if not bool(control_state.get("recovery_pending_dispatch", False)):
        return

    request_id = int(control_state.get("recovery_request_id", 0))
    for team in list(control_state.get("recovery_teams", [])):
        env = bus.make_envelope(producer)
        env.update(
            {
                "team": team,
                "request_id": request_id,
                "timeout_s": RECOVERY_TIMEOUT_S,
            }
        )
        bus.publish(pub, f"cmd.robot.recover.{team}", env)
    control_state["recovery_pending_dispatch"] = False


def _tick_conclusion_team(
    state: dict[str, Any],
    dt: float,
    game_cfg: dict[str, float],
    pose_cfg: dict[str, list[float]],
    stage_state: dict[str, Any],
) -> None:
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
            # TODO(bucket-controller): send the real bucket-open command
            # once BucketController exists on the new runtime path.
        return

    if phase == "empty_bucket":
        if phase_elapsed_s >= CONCLUSION_BUCKET_EMPTY_PAUSE_S:
            next_bucket_index = (
                int(state.get("conclusion_active_bucket_index") or 0) + 1
            )
            state["conclusion_active_bucket_index"] = next_bucket_index
            state["conclusion_bucket_open_triggered"] = False
            if next_bucket_index >= len(state["bucket_values"]):
                _set_announcement_phase(state, now_ns, pose_cfg)
            else:
                _set_bucket_pose_phase(state, now_ns, pose_cfg)
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
    state["conclusion_phase"] = "sum_bucket"
    state["conclusion_target_pose_name"] = pose_name
    state["conclusion_target_pose_deg"] = pose_cfg.get(
        pose_name, list(DEFAULT_LOOK_POSE_DEG)
    )
    state["conclusion_phase_started_mono_ns"] = now_ns
    # TODO(conclusion-motion): insert the collision-free move to the
    # bucket-look pose here. Until that planner exists, the robot stays
    # frozen and the controller advances directly into score summation.


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


def _coerce_team_bucket_values(value: Any) -> dict[str, list[int]]:
    if not isinstance(value, dict):
        return {}
    out: dict[str, list[int]] = {}
    for team, buckets in value.items():
        if not isinstance(team, str):
            continue
        out[team] = _coerce_bucket_value_list(buckets)
    return out


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


def _default_pose_map() -> dict[str, list[float]]:
    return {
        "robot_lookb1_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lookb2_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lookb3_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_announcement_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_win_pose": list(DEFAULT_LOOK_POSE_DEG),
        "robot_lose_pose": list(DEFAULT_LOOK_POSE_DEG),
    }


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


def _coerce_positive_float(value: Any, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if number > 0.0 else default


def _coerce_float_list(value: Any, fallback: list[float]) -> list[float]:
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
    return lo if value < lo else hi if value > hi else value


if __name__ == "__main__":
    sys.exit(main())
