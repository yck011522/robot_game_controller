"""haptic_io entry point — see __init__.py for impl dispatch."""

from __future__ import annotations

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq

from core import bus  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    args, _ = parse_proc_args(argv, default_proc="haptic_io.a")
    profile = load_profile(args.profile_path)
    target_hz = profile.subsystem_float("haptic_io", "fps_target", default_runtime_setting("haptic_io", "fps_target", 50.0))
    proc = Proc(args, profile, target_hz=target_hz)
    # team = last char of `haptic_io.a` / `haptic_io.b`.
    team = proc.proc.split(".")[-1]
    impl_name = proc.profile.subsystems.get("haptic_io", {}).get(team)
    if impl_name is None:
        print(f"[{proc.proc}] no impl configured for team {team!r}",
              file=sys.stderr, flush=True)
        return 2

    impl = _make_impl(impl_name, team=team, profile=profile)
    pub = bus.make_pub(proc.ctx)
    actual_sub = bus.make_sub(proc.ctx, topics=[f"telem.robot.actual.{team}"])
    cmd_sub = bus.make_sub(proc.ctx, topics=[f"cmd.haptic.{team}"])
    reseat_sub = bus.make_sub(proc.ctx, topics=[f"cmd.haptic.reseat.{team}"])
    seed_ref = {"done": False}
    # The Proc scaffold also creates a PUB for heartbeats; reusing this
    # one keeps us to a single PUB per process.
    proc.use_heartbeat_pub(pub)
    banner(proc.proc, f"impl={impl_name} team={team}")
    topic = f"telem.haptic.{team}"

    def tick(p: Proc) -> None:
        _drain_latest(actual_sub, on_msg=lambda b: _handle_robot_actual(impl, b, seed_ref))
        _drain_latest(cmd_sub, on_msg=lambda b: _apply_command(impl, b))
        _drain_latest(reseat_sub, on_msg=lambda b: _apply_reseat_request(impl, b))
        sample = impl.sample()
        env = bus.make_envelope(p.proc)
        env.update({"team": team, **sample})
        bus.publish(pub, topic, env)

    def teardown(_: Proc) -> None:
        actual_sub.close(0)
        cmd_sub.close(0)
        reseat_sub.close(0)
        close = getattr(impl, "close", None)
        if callable(close):
            close()

    return proc.run(tick, teardown=teardown)


def _make_impl(name: str, *, team: str, profile):
    if name == "sim_scripted":
        from subsystems.haptic.sim_scripted import ScriptedHaptic
        return ScriptedHaptic()
    if name == "sim_keyboard":
        from subsystems.haptic.sim_keyboard import KeyboardHaptic
        return KeyboardHaptic()
    if name == "real":
        from subsystems.haptic.real import RealHaptic
        return RealHaptic(team=team, profile=profile)
    raise NotImplementedError(f"haptic_io impl {name!r} not available yet")


def _drain_latest(sub, *, on_msg) -> None:
    last = None
    while True:
        try:
            _, body = bus.recv(sub, flags=zmq.NOBLOCK)
            last = body
        except zmq.Again:
            break
    if last is not None:
        on_msg(last)


def _handle_robot_actual(impl, body: dict, seed_ref: dict) -> None:
    q = body.get("q_rad")
    if not isinstance(q, list) or len(q) < 6:
        return
    update_actual = getattr(impl, "update_robot_actual", None)
    if callable(update_actual):
        update_actual([float(v) for v in q[:6]])
        return
    if seed_ref["done"]:
        return
    seed = getattr(impl, "set_current_position", None)
    if callable(seed):
        seed([float(v) for v in q[:6]])
    seed_ref["done"] = True


def _apply_command(impl, body: dict) -> None:
    apply = getattr(impl, "apply_command", None)
    if callable(apply):
        apply(body)


def _apply_reseat_request(impl, body: dict) -> None:
    q = body.get("current_pos_rad") if isinstance(body, dict) else None
    if not isinstance(q, list) or len(q) < 6:
        return
    reseat = getattr(impl, "request_reseat", None)
    if callable(reseat):
        reseat([float(v) for v in q[:6]])


if __name__ == "__main__":
    sys.exit(main())
