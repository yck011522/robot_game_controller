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
CONCLUSION_INITIAL_PAUSE_S = 1.0
CONCLUSION_BUCKET_EMPTY_PAUSE_S = 0.5
CONCLUSION_ANNOUNCEMENT_PAUSE_S = 1.0
UI_GAME_CONTROL_TOPIC = "cmd.ui.game_control"


def main(argv: list[str] | None = None) -> int:
    proc, _ = Proc.from_argv(target_hz=TICK_HZ, default_proc="game_controller")

    active_teams = list(proc.profile.active_teams)
    game_cfg = _game_config(proc.profile.tuning.get("game"))
    haptic_cfg = _haptic_config(proc.profile.tuning.get("haptic"))
    robot_show_poses = _load_robot_show_poses_deg()
    pub = bus.make_pub(proc.ctx)
    control_rep = bus.make_rep(proc.ctx)
    proc.use_heartbeat_pub(pub)

    # P2 ships team-A only; team-B wiring is symmetric and lands when
    # the second arm joins.
    if "a" not in active_teams:
        banner(proc.proc, "no active teams; will only emit heartbeat + skeleton state.full")

    collision_enabled = (
        isinstance(proc.profile.subsystems.get("collision_workers"), dict)
        and int(proc.profile.subsystems["collision_workers"].get("count", 0)) > 0
    )

    # Per-team state. P2 builds only `a`; the structure generalizes.
    teams: dict[str, dict] = {}
    for team in active_teams:
        planner = InProcessPlanner(ctx=proc.ctx, profile=proc.profile,
                                    team=team,
                                    collision_enabled=collision_enabled)
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
            "bucket_values": list(game_cfg["sim_bucket_values"].get(team, DEFAULT_BUCKET_VALUES)),
            "score": int(sum(game_cfg["sim_bucket_values"].get(team, DEFAULT_BUCKET_VALUES))),
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
        # Cache the last reply per source so a UI retry with the same
        # request_id can be acknowledged without reapplying the action.
        "last_request_id_by_source": {},
        "last_reply_by_source": {},
    }

    def tick(p: Proc) -> None:
        nonlocal state_seq
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
        soft_paused = bool(control_state.get("soft_pause", False))
        for team, st in teams.items():
            _drain_latest(st["sub_haptic"], on_msg=lambda b, s=st: _update_haptic_state(s, b))
            _drain_latest(st["sub_actual"], on_msg=lambda b, s=st: _update_actual_state(s, b))

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

            _publish_haptic_command(pub, p.proc, team, st, haptic_cfg)

            robot_status = st.get("robot_status", {})
            robot_fault_active = bool(robot_status.get("fault_active", False))
            if robot_fault_active or soft_paused:
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
                env.update({
                    "team": team,
                    "q_target_rad": list(st["last_q"]),
                    "clamps": {
                        "path": 1.0,
                        "prox": 1.0,
                        "final": 1.0,
                    },
                })
                bus.publish(pub, f"cmd.robot.target.{team}", env)
                continue

            if stage_state["stage"] == "play":
                st["score"] = int(sum(st["bucket_values"]))
            else:
                _tick_conclusion_team(st, dt, game_cfg, robot_show_poses.get(team, {}), stage_state)
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
                env.update({
                    "team": team,
                    "q_target_rad": list(st["last_q"]),
                    "clamps": {
                        "path": 1.0,
                        "prox": 1.0,
                        "final": 1.0,
                    },
                })
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
            st["last_prox_probe_offsets_deg"] = list(info.get("prox_probe_offsets_deg") or [])
            raw_hits = info.get("prox_hits") if isinstance(info.get("prox_hits"), list) else []
            st["last_prox_hits"] = [
                [bool(v) for v in axis_hits] if isinstance(axis_hits, list) else []
                for axis_hits in raw_hits[:6]
            ]
            while len(st["last_prox_hits"]) < 6:
                st["last_prox_hits"].append([])
            raw_ages = info.get("prox_age_ticks") if isinstance(info.get("prox_age_ticks"), list) else []
            st["last_prox_age_ticks"] = [int(v) for v in raw_ages[:6]] + [9999] * max(0, 6 - len(raw_ages[:6]))
            st["last_prox_age_ticks"] = st["last_prox_age_ticks"][:6]

            env = bus.make_envelope(p.proc)
            env.update({
                "team": team,
                "q_target_rad": q_target,
                "clamps": {
                    "path": st["last_path_scalar"],
                    "prox": st["last_prox_scalar"],
                    "final": st["last_final_scalar"],
                },
            })
            bus.publish(pub, f"cmd.robot.target.{team}", env)

        # Emit a (very small) state.full snapshot per BUS.md 禮6.1.
        paused_teams = [
            team for team, st in teams.items()
            if bool(st.get("robot_status", {}).get("fault_active", False))
        ]
        pause_reason = None
        for team in paused_teams:
            reason = teams[team].get("robot_status", {}).get("fault_reason")
            if isinstance(reason, str) and reason:
                pause_reason = f"{team}:{reason}"
                break
        if pause_reason is None and soft_paused:
            pause_reason = "soft_estop"

        paused = bool(paused_teams) or soft_paused

        _update_stage_pause_tracking(stage_state, paused, now_ns)
        if not paused:
            _tick_stage_state(stage_state, teams, game_cfg, now_ns)

        countdown_s = _stage_countdown_s(stage_state, game_cfg, now_ns)

        env = bus.make_envelope(p.proc, with_wall=True, seq=state_seq)
        env.update({
            "stage": "paused" if paused else stage_state["stage"],
            "active_stage": stage_state["stage"],
            "paused": paused,
            "pause_reason": pause_reason,
            "soft_estop": soft_paused,
            "countdown_s": countdown_s,
            "game_duration_s": game_cfg["duration_s"],
            "sum_score_rate_unit_per_s": game_cfg["sum_score_rate_unit_per_s"],
            "stage_entered_mono_ns": stage_state["stage_entered_mono_ns"],
            "tutorial_entered_wall_ns": None,
            "teams": {
                team: {
                    "robot": {
                        "q_target_rad": st["last_target"],
                        "q_rad": st["last_q"] if st["last_q"] is not None else [0.0] * 6,
                        "status": st.get("robot_status", {}),
                    },
                    "haptic": {
                        "dial_pos_rad": st["last_dial"],
                        "dial_vel_rad_s": st["last_dial_vel"],
                        "connected": st["last_haptic_connected"],
                        "board_loop_hz": st["last_haptic_loop_hz"],
                        "bounds_min_rad": list(haptic_cfg["bounds_min_rad"]),
                        "bounds_max_rad": list(haptic_cfg["bounds_max_rad"]),
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
                        "bucket_open_triggered": st["conclusion_bucket_open_triggered"],
                        "done": st["conclusion_done"],
                    },
                } for team, st in teams.items()
            },
        })
        bus.publish(pub, "state.full", env)
        state_seq += 1

    def teardown(_: Proc) -> None:
        for st in teams.values():
            st["planner"].close()
            st["sub_haptic"].close(0)
            st["sub_actual"].close(0)
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
        state["last_dial_vel"] = [float(v) for v in dial_vel[:6]] + [0.0] * max(0, 6 - len(dial_vel[:6]))
        state["last_dial_vel"] = state["last_dial_vel"][:6]
    connected = body.get("board_connected")
    if isinstance(connected, list):
        state["last_haptic_connected"] = [bool(v) for v in connected[:6]] + [False] * max(0, 6 - len(connected[:6]))
        state["last_haptic_connected"] = state["last_haptic_connected"][:6]
    loop_hz = body.get("board_loop_hz")
    if isinstance(loop_hz, list):
        state["last_haptic_loop_hz"] = [float(v) for v in loop_hz[:6]] + [0.0] * max(0, 6 - len(loop_hz[:6]))
        state["last_haptic_loop_hz"] = state["last_haptic_loop_hz"][:6]


def _game_config(node: Any) -> dict[str, float]:
    data = node if isinstance(node, dict) else {}
    return {
        "duration_s": _coerce_positive_float(data.get("duration_s"), 240.0),
        "sum_score_rate_unit_per_s": _coerce_positive_float(data.get("sum_score_rate_unit_per_s"), 100.0),
        "sim_bucket_values": _coerce_team_bucket_values(data.get("sim_bucket_values")),
    }


def _haptic_config(node: Any) -> dict[str, Any]:
    data = node if isinstance(node, dict) else {}
    return {
        "bounds_min_rad": [
            math.radians(v) for v in _coerce_float_list(data.get("bounds_deg_min"), DEFAULT_HAPTIC_BOUNDS_DEG_MIN)
        ],
        "bounds_max_rad": [
            math.radians(v) for v in _coerce_float_list(data.get("bounds_deg_max"), DEFAULT_HAPTIC_BOUNDS_DEG_MAX)
        ],
    }


def _publish_haptic_command(pub: zmq.Socket, producer: str, team: str, state: dict[str, Any], haptic_cfg: dict[str, Any]) -> None:
    env = bus.make_envelope(producer)
    env.update({
        "team": team,
        "tracking_target_rad": list(state["last_q"]),
        "bounds_min_rad": list(haptic_cfg["bounds_min_rad"]),
        "bounds_max_rad": list(haptic_cfg["bounds_max_rad"]),
    })
    bus.publish(pub, f"cmd.haptic.{team}", env)


def _tick_stage_state(stage_state: dict[str, Any], teams: dict[str, dict], game_cfg: dict[str, float], now_ns: int) -> None:
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


def _reset_for_play(stage_state: dict[str, Any], teams: dict[str, dict], game_cfg: dict[str, float], now_ns: int) -> None:
    stage_state["stage"] = "play"
    stage_state["stage_entered_mono_ns"] = now_ns
    stage_state["winner_team"] = None
    stage_state["pause_started_mono_ns"] = None
    stage_state["paused_total_ns"] = 0

    sim_bucket_values = game_cfg.get("sim_bucket_values") if isinstance(game_cfg, dict) else {}
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


def _stage_countdown_s(stage_state: dict[str, Any], game_cfg: dict[str, float], now_ns: int) -> int:
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
        (now_ns - int(stage_state["stage_entered_mono_ns"]) - paused_total_ns - active_pause_ns) / 1e9,
    )


def _update_stage_pause_tracking(stage_state: dict[str, Any], paused: bool, now_ns: int) -> None:
    pause_started_ns = stage_state.get("pause_started_mono_ns")
    if paused:
        if pause_started_ns is None:
            stage_state["pause_started_mono_ns"] = now_ns
        return
    if pause_started_ns is None:
        return
    stage_state["paused_total_ns"] = int(stage_state.get("paused_total_ns") or 0) + (now_ns - int(pause_started_ns))
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


def _force_conclusion(stage_state: dict[str, Any], teams: dict[str, dict], now_ns: int) -> None:
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
    request = body if isinstance(body, dict) else {}
    source = request.get("source") if isinstance(request.get("source"), str) else "unknown"
    request_id = request.get("request_id") if isinstance(request.get("request_id"), (int, str)) else None

    last_request_id_by_source = control_state.setdefault("last_request_id_by_source", {})
    last_reply_by_source = control_state.setdefault("last_reply_by_source", {})
    if request_id is not None and last_request_id_by_source.get(source) == request_id:
        cached = last_reply_by_source.get(source)
        if isinstance(cached, dict):
            return dict(cached)

    ok, error, action = _apply_ui_game_control(control_state, stage_state, teams, request, now_ns)
    reply = bus.make_envelope("game_controller", with_wall=True)
    reply.update({
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
    })
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
    action = body.get("action") if isinstance(body, dict) else None
    if not isinstance(action, str):
        return False, "missing action", None
    if action == "play_resume":
        control_state["soft_pause"] = False
    elif action == "soft_estop":
        control_state["soft_pause"] = True
    elif action == "end_game":
        control_state["soft_pause"] = False
        _force_conclusion(stage_state, teams, now_ns)
    else:
        return False, f"unsupported action: {action}", action
    control_state["last_action"] = action
    control_state["last_action_ts_mono_ns"] = now_ns
    return True, None, action


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
        accumulated_units = float(state.get("conclusion_sum_remainder_units", 0.0)) + (game_cfg["sum_score_rate_unit_per_s"] * dt)
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
            next_bucket_index = int(state.get("conclusion_active_bucket_index") or 0) + 1
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
                state["conclusion_target_pose_name"] = "robot_win_pose" if state["team"] == winner_team else "robot_lose_pose"
            state["conclusion_target_pose_deg"] = pose_cfg.get(state["conclusion_target_pose_name"], list(DEFAULT_LOOK_POSE_DEG))
            state["conclusion_phase_started_mono_ns"] = now_ns
            # TODO(conclusion-motion): replace this pose bookkeeping with a
            # collision-free motion plan once the dedicated planner lands.
        return

    if phase == "winner_pose":
        state["conclusion_done"] = True


def _set_bucket_pose_phase(state: dict[str, Any], now_ns: int, pose_cfg: dict[str, list[float]]) -> None:
    bucket_index = int(state.get("conclusion_active_bucket_index") or 0)
    pose_names = ["robot_lookb1_pose", "robot_lookb2_pose", "robot_lookb3_pose"]
    pose_name = pose_names[min(bucket_index, len(pose_names) - 1)]
    state["conclusion_phase"] = "sum_bucket"
    state["conclusion_target_pose_name"] = pose_name
    state["conclusion_target_pose_deg"] = pose_cfg.get(pose_name, list(DEFAULT_LOOK_POSE_DEG))
    state["conclusion_phase_started_mono_ns"] = now_ns
    # TODO(conclusion-motion): insert the collision-free move to the
    # bucket-look pose here. Until that planner exists, the robot stays
    # frozen and the controller advances directly into score summation.


def _set_announcement_phase(state: dict[str, Any], now_ns: int, pose_cfg: dict[str, list[float]]) -> None:
    state["conclusion_phase"] = "announcement_pose"
    state["conclusion_target_pose_name"] = "robot_announcement_pose"
    state["conclusion_target_pose_deg"] = pose_cfg.get("robot_announcement_pose", list(DEFAULT_LOOK_POSE_DEG))
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
            name: _coerce_deg_pose(node.get(name), fallback[name])
            for name in fallback
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
        out.extend(fallback[len(out):6])
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
        out.extend(float(v) for v in fallback[len(out):6])
    return out[:6]


if __name__ == "__main__":
    sys.exit(main())
