"""game_controller entry point ??see __init__.py."""

from __future__ import annotations

import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

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


def main(argv: list[str] | None = None) -> int:
    proc, _ = Proc.from_argv(target_hz=TICK_HZ, default_proc="game_controller")

    active_teams = list(proc.profile.active_teams)
    pub = bus.make_pub(proc.ctx)
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
            "sub_haptic": sub,
            "sub_actual": actual_sub,
            "last_dial": [0.0] * 6,
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
            "robot_status": {},
            "last_tick_t": time.perf_counter(),
        }
    banner(proc.proc, f"teams={active_teams} collision_check={collision_enabled}")

    state_seq = 0
    started_mono_ns = time.perf_counter_ns()

    def tick(p: Proc) -> None:
        nonlocal state_seq
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
                continue

            robot_status = st.get("robot_status", {})
            robot_fault_active = bool(robot_status.get("fault_active", False))
            if robot_fault_active:
                st["last_target"] = list(st["last_q"])
                st["last_collision"] = False
                st["last_first_hit"] = None
                st["last_path_scalar"] = 1.0
                st["last_prox_scalar"] = 1.0
                st["last_final_scalar"] = 1.0

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

        env = bus.make_envelope(p.proc, with_wall=True, seq=state_seq)
        env.update({
            "stage": "paused" if paused_teams else "play",
            "paused": bool(paused_teams),
            "pause_reason": pause_reason,
            "stage_entered_mono_ns": started_mono_ns,
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
                        "connected": st["last_haptic_connected"],
                        "board_loop_hz": st["last_haptic_loop_hz"],
                    },
                    "collision": {
                        "in_collision": st["last_collision"],
                        "first_hit": st["last_first_hit"],
                        "path_scalar": st["last_path_scalar"],
                        "prox_scalar": st["last_prox_scalar"],
                        "final_scalar": st["last_final_scalar"],
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


def _update_actual_state(state: dict, body: dict) -> None:
    state["last_q"] = body.get("q_rad", state["last_q"])
    robot_status = body.get("robot_status")
    if isinstance(robot_status, dict):
        state["robot_status"] = robot_status


def _update_haptic_state(state: dict, body: dict) -> None:
    state["last_dial"] = body.get("dial_pos_rad", state["last_dial"])
    connected = body.get("board_connected")
    if isinstance(connected, list):
        state["last_haptic_connected"] = [bool(v) for v in connected[:6]] + [False] * max(0, 6 - len(connected[:6]))
        state["last_haptic_connected"] = state["last_haptic_connected"][:6]
    loop_hz = body.get("board_loop_hz")
    if isinstance(loop_hz, list):
        state["last_haptic_loop_hz"] = [float(v) for v in loop_hz[:6]] + [0.0] * max(0, 6 - len(loop_hz[:6]))
        state["last_haptic_loop_hz"] = state["last_haptic_loop_hz"][:6]


if __name__ == "__main__":
    sys.exit(main())
