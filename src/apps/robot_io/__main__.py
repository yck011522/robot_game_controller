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
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.device_connection import require_robot_endpoint  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from subsystems.robot.joint_limits import clamp_joint_target_rad, resolve_joint_limits_rad  # noqa: E402


# Internal tick rate. We poll the cmd sub at this rate and step
# pybullet at its own STEP_HZ inside the impl; telemetry is rate-limited
# below.
DEFAULT_TICK_HZ = 200.0
TELEM_PERIOD_S = 1.0 / 100.0  # 100 Hz per BUS.md 禮5.3
DEFAULT_RECOVERY_TIMEOUT_S = 4.0
DEFAULT_SAFETY_TELEM_AGE_MAX_MS = 1100.0


def main(argv: list[str] | None = None) -> int:
    """Run one robot I/O process and bridge bus traffic into the selected backend."""
    args, _ = parse_proc_args(argv, default_proc="robot_io.a")
    profile = load_profile(args.profile_path)
    target_hz = profile.subsystem_float("robot_io", "fps_target", default_runtime_setting("robot_io", "fps_target", DEFAULT_TICK_HZ))
    proc = Proc(args, profile, target_hz=target_hz)
    team = proc.proc.split(".")[-1]
    impl_name = proc.profile.subsystems.get("robot_io", {}).get(team)
    if impl_name is None:
        print(f"[{proc.proc}] no impl configured for team {team!r}",
              file=sys.stderr, flush=True)
        return 2

    headless = bool(proc.profile.tuning.get("robot", {}).get("headless", False))
    # Optional per-team override: {a: bool, b: bool}. Lets a two-team sim run
    # one team headless (DIRECT) and the other with the PyBullet GUI, since
    # PyBullet only comfortably supports one visible debug-visualizer window
    # at a time on most machines. Falls back to the shared `headless` value
    # above when this team has no entry (or the block is absent entirely).
    headless_by_team = proc.profile.tuning.get("robot", {}).get("headless_by_team")
    if isinstance(headless_by_team, dict) and team in headless_by_team:
        headless = bool(headless_by_team[team])
    initial_pose_deg = proc.profile.tuning.get("robot", {}).get(
        "initial_pose_deg", [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
    )
    q_min, q_max = resolve_joint_limits_rad(proc.profile.tuning.get("robot", {}), axes=6)
    safety_enabled = proc.profile.subsystem_impl("safety_barrier_controller") is not None
    safety_telem_age_max_s = (
        default_runtime_setting(
            "safety_barrier_controller",
            "telem_age_max",
            DEFAULT_SAFETY_TELEM_AGE_MAX_MS,
        )
        or DEFAULT_SAFETY_TELEM_AGE_MAX_MS
    ) / 1000.0
    import math as _math
    initial_pose_rad = [_math.radians(float(v)) for v in initial_pose_deg]

    pub = bus.make_pub(proc.ctx)
    proc.use_heartbeat_pub(pub)

    impl = _make_impl(
        impl_name,
        team=team,
        headless=headless,
        initial_pose_rad=initial_pose_rad,
        servo_hz=target_hz,
    )
    banner(proc.proc, f"impl={impl_name} headless={headless}")

    # CONFLATE ??only the freshest target matters; older targets are stale.
    # NOTE: ZMQ CONFLATE is documented as incompatible with SUB-side
    # subscription filtering ??combining them silently drops every
    # message. We drain the queue inside the tick instead and act on
    # the last sample, which gets us the same "latest wins" semantics
    # without the trap.
    sub = bus.make_sub(proc.ctx, topics=[f"cmd.robot.target.{team}"])
    recover_sub = bus.make_sub(proc.ctx, topics=[f"cmd.robot.recover.{team}"])
    state_sub = bus.make_sub(proc.ctx, topics=["state.full"])
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    poller.register(recover_sub, zmq.POLLIN)
    poller.register(state_sub, zmq.POLLIN)

    # TODO(safety): P3/P4 bring-up intentionally bypasses the safety-barrier
    # interlock so the real-robot stack can keep running during migration.
    # Re-enable the startup block and runtime command gate once
    # SafetyBarrierController lands in its dedicated later phase.
    _read_startup_barrier_ok(state_sub)

    telem_topic = f"telem.robot.actual.{team}"
    next_telem = time.perf_counter()
    seq = 0
    latest_barrier_state = {
        "ok": True if not safety_enabled else None,
        "last_state_recv_mono_s": None,
    }
    # Passive diagnostics for pause/recovery backlog investigations. These
    # counters do not alter latest-wins command handling.
    command_queue_state = {
        "last_drain_count": 0,
        "max_drain_count": 0,
        "total_received": 0,
        "last_drain_ms": 0.0,
        "max_drain_ms": 0.0,
        "last_target_rad": None,
    }

    def tick(p: Proc) -> None:
        nonlocal next_telem, seq
        # Drain every pending cmd; keep only the last (latest wins).
        latest_q = None
        latest_clamps = None
        latest_recover_timeout_s = None
        command_queue_state["last_drain_count"] = 0
        command_queue_state["last_drain_ms"] = 0.0
        events = dict(poller.poll(1))
        if state_sub in events:
            _drain_latest(state_sub, on_msg=lambda body: _update_barrier_state(latest_barrier_state, body))
        if sub in events:
            drain_started_s = time.perf_counter()
            drain_count = 0
            while True:
                try:
                    _, body = bus.recv(sub, flags=zmq.NOBLOCK)
                    drain_count += 1
                    q = body.get("q_target_rad")
                    if isinstance(q, list):
                        latest_q = q
                    c = body.get("clamps")
                    if isinstance(c, dict):
                        latest_clamps = c
                except zmq.Again:
                    break
            drain_ms = (time.perf_counter() - drain_started_s) * 1000.0
            command_queue_state["last_drain_count"] = drain_count
            command_queue_state["max_drain_count"] = max(
                int(command_queue_state["max_drain_count"]), drain_count
            )
            command_queue_state["total_received"] = (
                int(command_queue_state["total_received"]) + drain_count
            )
            command_queue_state["last_drain_ms"] = drain_ms
            command_queue_state["max_drain_ms"] = max(
                float(command_queue_state["max_drain_ms"]), drain_ms
            )
            if latest_q is not None:
                command_queue_state["last_target_rad"] = list(latest_q[:6])
        if recover_sub in events:
            while True:
                try:
                    _, body = bus.recv(recover_sub, flags=zmq.NOBLOCK)
                    timeout_s = body.get("timeout_s")
                    if isinstance(timeout_s, (int, float)):
                        latest_recover_timeout_s = float(timeout_s)
                    else:
                        latest_recover_timeout_s = DEFAULT_RECOVERY_TIMEOUT_S
                except zmq.Again:
                    break
        safety_allows_motion = _safety_allows_motion(
            latest_barrier_state,
            enabled=safety_enabled,
            stale_after_s=safety_telem_age_max_s,
        )
        if latest_q is not None and safety_allows_motion:
            impl.set_target(clamp_joint_target_rad(latest_q, q_min, q_max, axes=6))
        if latest_clamps is not None and safety_allows_motion and hasattr(impl, "set_clamps"):
            impl.set_clamps(latest_clamps)
        if latest_recover_timeout_s is not None and hasattr(impl, "request_recovery"):
            impl.request_recovery(timeout_s=max(0.1, latest_recover_timeout_s))

        if safety_allows_motion:
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
            # with_wall=True: gameplay_recorder aligns telem.robot.actual samples
            # across streams/processes using this wall clock (BUS.md 4.1);
            # ts_mono_ns alone is process-local and unsuitable for that.
            env = bus.make_envelope(p.proc, with_wall=True, seq=seq)
            env.update({
                "team": team,
                "q_rad": q,
                "qd_rad_s": qd,
                "rtde_ok": bool(getattr(impl, "rtde_ok", True)),
                "robot_status": robot_status,
                "command_queue": {
                    "last_drain_count": int(command_queue_state["last_drain_count"]),
                    "max_drain_count": int(command_queue_state["max_drain_count"]),
                    "total_received": int(command_queue_state["total_received"]),
                    "last_drain_ms": float(command_queue_state["last_drain_ms"]),
                    "max_drain_ms": float(command_queue_state["max_drain_ms"]),
                    "last_target_rad": command_queue_state["last_target_rad"],
                },
            })
            bus.publish(pub, telem_topic, env)
            seq += 1

    def teardown(_: Proc) -> None:
        sub.close(0)
        recover_sub.close(0)
        state_sub.close(0)
        impl.close()

    return proc.run(tick, teardown=teardown)


def _make_impl(
    name: str,
    *,
    team: str,
    headless: bool,
    initial_pose_rad=None,
    servo_hz: float = DEFAULT_TICK_HZ,
):
    """Construct the configured robot backend for this process."""
    if name == "sim_pybullet":
        from subsystems.robot.robot_sim_pybullet import SimPybulletRobot
        return SimPybulletRobot(headless=headless, initial_pose_rad=initial_pose_rad)
    if name == "real_rtde":
        from subsystems.robot.robot_real_rtde import RealRtdeRobot
        endpoint = require_robot_endpoint(team)
        return RealRtdeRobot(host=endpoint.host, port=endpoint.port, servo_hz=servo_hz)
    raise NotImplementedError(f"robot_io impl {name!r} not available yet")


def _extract_barrier_ok(body: dict) -> bool | None:
    """Extract the latest safety barrier boolean from a `state.full` payload."""
    safety = body.get("safety")
    if not isinstance(safety, dict):
        return None
    barrier = safety.get("barrier")
    if not isinstance(barrier, dict):
        return None
    ok = barrier.get("ok")
    return ok if isinstance(ok, bool) else None


def _update_barrier_state(state: dict, body: dict) -> None:
    """Update RobotIO's local safety-barrier cache from one `state.full` sample."""

    ok = _extract_barrier_ok(body)
    if ok is None:
        return
    state["ok"] = ok
    state["last_state_recv_mono_s"] = time.perf_counter()


def _safety_allows_motion(state: dict, *, enabled: bool, stale_after_s: float) -> bool:
    """Return whether RobotIO may send/step robot targets this tick."""

    if not enabled:
        return True
    last_recv = state.get("last_state_recv_mono_s")
    if not isinstance(last_recv, float):
        return False
    if (time.perf_counter() - last_recv) > stale_after_s:
        return False
    return bool(state.get("ok", False))


def _read_startup_barrier_ok(sub: zmq.Socket, timeout_s: float = 0.5) -> bool | None:
    """Read the latest startup barrier state without blocking normal bring-up for long."""
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
    """Store the latest parsed barrier state for the startup wait helper."""
    latest_ref["value"] = _extract_barrier_ok(body)


def _drain_latest(sub: zmq.Socket, *, on_msg) -> None:
    """Drain a SUB socket and hand only the freshest payload to the caller."""
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
