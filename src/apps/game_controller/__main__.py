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
            # last_q starts as None; planner only re-seeds once a real
            # telem.robot.actual.<team> has actually arrived. Without
            # this guard the very first tick would seed the planner's
            # integrator with all-zero (the default) and the robot
            # would snap to the in-pedestal pose.
            "last_q": None,
            "last_target": list(planner.q_cur),
            "last_collision": False,
            "last_first_hit": None,
            "last_path_scalar": 1.0,
            "last_prox_scalar": 1.0,
            "last_final_scalar": 1.0,
            "last_tick_t": time.perf_counter(),
        }
    banner(proc.proc, f"teams={active_teams} collision_check={collision_enabled}")

    state_seq = 0
    started_mono_ns = time.perf_counter_ns()

    def tick(p: Proc) -> None:
        nonlocal state_seq
        for team, st in teams.items():
            _drain_latest(st["sub_haptic"], on_msg=lambda b, s=st:
                          s.update(last_dial=b.get("dial_pos_rad", s["last_dial"])))
            _drain_latest(st["sub_actual"], on_msg=lambda b, s=st:
                          s.update(last_q=b.get("q_rad", s["last_q"])))

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
        env = bus.make_envelope(p.proc, with_wall=True, seq=state_seq)
        env.update({
            "stage": "play",
            "stage_entered_mono_ns": started_mono_ns,
            "tutorial_entered_wall_ns": None,
            "teams": {
                team: {
                    "robot": {
                        "q_target_rad": st["last_target"],
                        "q_rad": st["last_q"] if st["last_q"] is not None else [0.0] * 6,
                    },
                    "haptic": {
                        "dial_pos_rad": st["last_dial"],
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


if __name__ == "__main__":
    sys.exit(main())
