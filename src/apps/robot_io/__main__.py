"""robot_io entry point."""

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


# Internal tick rate. We poll the cmd sub at this rate and step
# pybullet at its own STEP_HZ inside the impl; telemetry is rate-limited
# below.
TICK_HZ = 200.0
TELEM_PERIOD_S = 1.0 / 100.0  # 100 Hz per BUS.md 禮5.3


def main(argv: list[str] | None = None) -> int:
    proc, _ = Proc.from_argv(target_hz=TICK_HZ, default_proc="robot_io.a")
    team = proc.proc.split(".")[-1]
    impl_name = proc.profile.subsystems.get("robot_io", {}).get(team)
    if impl_name is None:
        print(f"[{proc.proc}] no impl configured for team {team!r}",
              file=sys.stderr, flush=True)
        return 2

    headless = bool(proc.profile.tuning.get("robot", {}).get("headless", False))
    initial_pose_deg = proc.profile.tuning.get("robot", {}).get(
        "initial_pose_deg", [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
    )
    import math as _math
    initial_pose_rad = [_math.radians(float(v)) for v in initial_pose_deg]

    pub = bus.make_pub(proc.ctx)
    proc.use_heartbeat_pub(pub)

    impl = _make_impl(impl_name, headless=headless, initial_pose_rad=initial_pose_rad)
    banner(proc.proc, f"impl={impl_name} headless={headless}")

    # CONFLATE ??only the freshest target matters; older targets are stale.
    # NOTE: ZMQ CONFLATE is documented as incompatible with SUB-side
    # subscription filtering ??combining them silently drops every
    # message. We drain the queue inside the tick instead and act on
    # the last sample, which gets us the same "latest wins" semantics
    # without the trap.
    sub = bus.make_sub(proc.ctx, topics=[f"cmd.robot.target.{team}"])
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    telem_topic = f"telem.robot.actual.{team}"
    next_telem = time.perf_counter()
    seq = 0

    def tick(p: Proc) -> None:
        nonlocal next_telem, seq
        # Drain every pending cmd; keep only the last (latest wins).
        latest_q = None
        events = dict(poller.poll(1))
        if sub in events:
            while True:
                try:
                    _, body = bus.recv(sub, flags=zmq.NOBLOCK)
                    q = body.get("q_target_rad")
                    if isinstance(q, list):
                        latest_q = q
                except zmq.Again:
                    break
        if latest_q is not None:
            impl.set_target([float(x) for x in latest_q])

        impl.maybe_step()

        now = time.perf_counter()
        if now >= next_telem:
            next_telem += TELEM_PERIOD_S
            if next_telem < now:
                next_telem = now + TELEM_PERIOD_S
            q, qd = impl.read_state()
            env = bus.make_envelope(p.proc, seq=seq)
            env.update({"team": team, "q_rad": q, "qd_rad_s": qd, "rtde_ok": True})
            bus.publish(pub, telem_topic, env)
            seq += 1

    def teardown(_: Proc) -> None:
        sub.close(0)
        impl.close()

    return proc.run(tick, teardown=teardown)


def _make_impl(name: str, *, headless: bool, initial_pose_rad=None):
    if name == "sim_pybullet":
        from subsystems.robot.sim_pybullet import SimPybulletRobot
        return SimPybulletRobot(headless=headless, initial_pose_rad=initial_pose_rad)
    raise NotImplementedError(f"robot_io impl {name!r} not available yet")


if __name__ == "__main__":
    sys.exit(main())
