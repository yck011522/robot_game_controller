"""haptic_io entry point — see __init__.py for impl dispatch."""

from __future__ import annotations

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from core import bus  # noqa: E402
from core.proc import Proc, banner  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    proc, _ = Proc.from_argv(target_hz=50.0, default_proc="haptic_io.a")
    # team = last char of `haptic_io.a` / `haptic_io.b`.
    team = proc.proc.split(".")[-1]
    impl_name = proc.profile.subsystems.get("haptic_io", {}).get(team)
    if impl_name is None:
        print(f"[{proc.proc}] no impl configured for team {team!r}",
              file=sys.stderr, flush=True)
        return 2

    impl = _make_impl(impl_name)
    pub = bus.make_pub(proc.ctx)
    # The Proc scaffold also creates a PUB for heartbeats; reusing this
    # one keeps us to a single PUB per process.
    proc.use_heartbeat_pub(pub)
    banner(proc.proc, f"impl={impl_name} team={team}")
    topic = f"telem.haptic.{team}"

    def tick(p: Proc) -> None:
        sample = impl.sample()
        env = bus.make_envelope(p.proc)
        env.update({"team": team, **sample})
        bus.publish(pub, topic, env)

    def teardown(_: Proc) -> None:
        close = getattr(impl, "close", None)
        if callable(close):
            close()

    return proc.run(tick, teardown=teardown)


def _make_impl(name: str):
    if name == "sim_scripted":
        from subsystems.haptic.sim_scripted import ScriptedHaptic
        return ScriptedHaptic()
    if name == "sim_keyboard":
        from subsystems.haptic.sim_keyboard import KeyboardHaptic
        return KeyboardHaptic()
    raise NotImplementedError(f"haptic_io impl {name!r} not available yet")


if __name__ == "__main__":
    sys.exit(main())
