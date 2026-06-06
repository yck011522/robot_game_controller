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
from subsystems.robot.joint_limits import clamp_joint_target_rad, resolve_joint_limits_rad  # noqa: E402


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
    robot_hw = proc.profile.hardware.get("robot", {}) or {}
    robot_cfg = robot_hw.get(team, {}) or {}
    q_min, q_max = resolve_joint_limits_rad(proc.profile.tuning.get("robot", {}), axes=6)
    import math as _math
    initial_pose_rad = [_math.radians(float(v)) for v in initial_pose_deg]

    pub = bus.make_pub(proc.ctx)
    proc.use_heartbeat_pub(pub)

    impl = _make_impl(
        impl_name,
        team=team,
        headless=headless,
        initial_pose_rad=initial_pose_rad,
        robot_cfg=robot_cfg,
    )
    banner(proc.proc, f"impl={impl_name} headless={headless}")

    # CONFLATE ??only the freshest target matters; older targets are stale.
    # NOTE: ZMQ CONFLATE is documented as incompatible with SUB-side
    # subscription filtering ??combining them silently drops every
    # message. We drain the queue inside the tick instead and act on
    # the last sample, which gets us the same "latest wins" semantics
    # without the trap.
    sub = bus.make_sub(proc.ctx, topics=[f"cmd.robot.target.{team}"])
    state_sub = bus.make_sub(proc.ctx, topics=["state.full"])
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    poller.register(state_sub, zmq.POLLIN)

    # TODO(safety): P3/P4 bring-up intentionally bypasses the safety-barrier
    # interlock so the real-robot stack can keep running during migration.
    # Re-enable the startup block and runtime command gate once
    # SafetyBarrierController lands in its dedicated later phase.
    _read_startup_barrier_ok(state_sub)

    telem_topic = f"telem.robot.actual.{team}"
    next_telem = time.perf_counter()
    seq = 0

    def tick(p: Proc) -> None:
        nonlocal next_telem, seq
        # Drain every pending cmd; keep only the last (latest wins).
        latest_q = None
        latest_clamps = None
        events = dict(poller.poll(1))
        if state_sub in events:
            # TODO(safety): consume and discard state.full for now so this
            # socket stays drained until the real barrier controller exists.
            _drain_latest(state_sub, on_msg=lambda _b: None)
        if sub in events:
            while True:
                try:
                    _, body = bus.recv(sub, flags=zmq.NOBLOCK)
                    q = body.get("q_target_rad")
                    if isinstance(q, list):
                        latest_q = q
                    c = body.get("clamps")
                    if isinstance(c, dict):
                        latest_clamps = c
                except zmq.Again:
                    break
        if latest_q is not None:
            impl.set_target(clamp_joint_target_rad(latest_q, q_min, q_max, axes=6))
        if latest_clamps is not None and hasattr(impl, "set_clamps"):
            impl.set_clamps(latest_clamps)

        impl.maybe_step()

        now = time.perf_counter()
        if now >= next_telem:
            next_telem += TELEM_PERIOD_S
            if next_telem < now:
                next_telem = now + TELEM_PERIOD_S
            q, qd = impl.read_state()
            robot_status = {}
            if hasattr(impl, "status_snapshot"):
                snapshot = impl.status_snapshot()
                if isinstance(snapshot, dict):
                    robot_status = snapshot
            env = bus.make_envelope(p.proc, seq=seq)
            env.update({
                "team": team,
                "q_rad": q,
                "qd_rad_s": qd,
                "rtde_ok": bool(getattr(impl, "rtde_ok", True)),
                "robot_status": robot_status,
            })
            bus.publish(pub, telem_topic, env)
            seq += 1

    def teardown(_: Proc) -> None:
        sub.close(0)
        state_sub.close(0)
        impl.close()

    return proc.run(tick, teardown=teardown)


def _make_impl(name: str, *, team: str, headless: bool,
               initial_pose_rad=None, robot_cfg: dict | None = None):
    if name == "sim_pybullet":
        from subsystems.robot.robot_sim_pybullet import SimPybulletRobot
        return SimPybulletRobot(headless=headless, initial_pose_rad=initial_pose_rad)
    if name == "real_rtde":
        from subsystems.robot.robot_real_rtde import RealRtdeRobot
        robot_cfg = robot_cfg or {}
        host = robot_cfg.get("host")
        if not isinstance(host, str) or not host:
            raise ValueError(f"robot_io.{team} real_rtde requires hardware.robot.{team}.host")
        port = robot_cfg.get("port")
        return RealRtdeRobot(host=host, port=port, servo_hz=TICK_HZ)
    raise NotImplementedError(f"robot_io impl {name!r} not available yet")


def _extract_barrier_ok(body: dict) -> bool | None:
    safety = body.get("safety")
    if not isinstance(safety, dict):
        return None
    barrier = safety.get("barrier")
    if not isinstance(barrier, dict):
        return None
    ok = barrier.get("ok")
    return ok if isinstance(ok, bool) else None


def _read_startup_barrier_ok(sub: zmq.Socket, timeout_s: float = 0.5) -> bool | None:
    deadline = time.perf_counter() + timeout_s
    latest = None
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    while time.perf_counter() < deadline:
        wait_ms = max(1, int((deadline - time.perf_counter()) * 1000.0))
        events = dict(poller.poll(wait_ms))
        if sub in events:
            latest_ref = {"value": latest}
            _drain_latest(sub, on_msg=lambda b: _startup_barrier_sink(b, latest_ref))
            latest = latest_ref["value"]
            if latest is False:
                break
    return latest


def _startup_barrier_sink(body: dict, latest_ref: dict) -> None:
    latest_ref["value"] = _extract_barrier_ok(body)


def _drain_latest(sub: zmq.Socket, *, on_msg) -> None:
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
