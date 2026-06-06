"""Bus broker entry point. Run via `python -m apps.bus_broker --profile ... --proc bus_broker`.

See [`apps.bus_broker.__init__`](./__init__.py) for the mechanism
overview and [SUPERVISOR.md 禮3](../../../docs/architecture/SUPERVISOR.md#3-spawn-contract)
for the CLI contract this child obeys.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

# When the launcher spawns us it sets PYTHONPATH=src so `import core` /
# `import apps` works. When a developer launches this module directly
# from the repo root (`python -m apps.bus_broker ...`), PYTHONPATH may
# not be set, so we self-bootstrap by adding repo_root/src to sys.path.
# `parents[2]` from src/apps/bus_broker/__main__.py is the `src/` dir.
_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402


DEFAULT_HEARTBEAT_HZ = 1.0  # BUS.md 禮5.5: every long-lived process emits 1 Hz.


def _run_proxy(ctx: zmq.Context, stop: threading.Event, bound: threading.Event) -> None:
    """The XSUB/XPUB forwarder loop. Runs in a daemon thread.

    The two sockets are *bound* (not connected) ??the broker is the only
    process that binds these endpoints; every other process connects to
    them. `zmq.proxy()` then blocks forever shuffling frames between
    them. On shutdown we close the underlying context (back in `main`'s
    finally block), which causes both sockets to error out and `proxy()`
    to raise `ContextTerminated`, letting this thread exit.
    """
    xsub = ctx.socket(zmq.XSUB)
    xpub = ctx.socket(zmq.XPUB)
    # Brief bind-retry: when tests run back-to-back the previous
    # broker's port may still be in Windows TIME_WAIT. ~2 s is enough
    # for the OS to release it.
    deadline = time.perf_counter() + 3.0
    while True:
        try:
            xsub.bind(bus.BUS_XSUB_BIND)
            xpub.bind(bus.BUS_XPUB_BIND)
            bound.set()
            break
        except zmq.ZMQError:
            if time.perf_counter() > deadline:
                xsub.close(0)
                xpub.close(0)
                raise
            time.sleep(0.2)
    try:
        zmq.proxy(xsub, xpub)
    except zmq.ContextTerminated:
        # Expected on shutdown ??`ctx.destroy(linger=0)` is the trigger.
        pass
    except zmq.ZMQError as exc:
        if exc.errno not in (zmq.ETERM, zmq.ENOTSOCK):
            raise
    finally:
        # `close(0)` = linger=0; drop any queued messages immediately
        # so a hung peer doesn't keep the socket alive.
        for sock in (xsub, xpub):
            try:
                sock.close(0)
            except zmq.ZMQError as exc:
                if exc.errno != zmq.ENOTSOCK:
                    raise


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="XSUB/XPUB bus broker")
    ap.add_argument("--profile", required=True, help="path to profile YAML")
    ap.add_argument("--proc", default="bus_broker",
                    help="canonical process name (used as the producer field "
                         "on every message and as <proc> in heartbeat.<proc>)")
    args = ap.parse_args(argv)

    # Parse + validate the profile up front. We don't read any fields
    # from it ourselves, but every child does this on startup so a
    # broken YAML fails loud at spawn time instead of silently producing
    # heartbeats with the rest of the system misbehaving.
    profile = load_profile(args.profile)

    # One ZMQ context per process. `Context.instance()` returns the
    # process-global singleton so other code in this process (the
    # heartbeat PUB below, plus anything imported from `core.bus`)
    # shares it. That's important: the heartbeat PUB has to live in the
    # same context as the XSUB proxy bind, or the in-process loopback
    # path is slower than necessary.
    ctx = zmq.Context.instance()
    stop = threading.Event()
    bound = threading.Event()

    # Start the proxy first so its binds are listening before our own
    # PUB tries to connect back to XSUB.
    proxy_thread = threading.Thread(target=_run_proxy, args=(ctx, stop, bound),
                                     name="bus_proxy", daemon=True)
    proxy_thread.start()
    # Wait for the proxy thread to actually bind. The bind itself can
    # take up to a few seconds when a previous broker's port is still
    # in Windows TIME_WAIT (back-to-back test runs hit this). If it
    # never binds, fail loud ??better than silently flooding queued
    # heartbeats once the bind eventually succeeds.
    if not bound.wait(timeout=5.0):
        print(f"[{args.proc}] proxy failed to bind within 5s", flush=True)
        return 1
    # 50 ms is enough for the kernel to register the binds. ZMQ's
    # connect/bind ordering is forgiving, but PUB/SUB has a slow-joiner
    # problem where messages published before the subscription update
    # arrives at the publisher are silently dropped ??the sleep keeps
    # the very first heartbeat from being eaten by that race.
    time.sleep(0.05)

    pub = bus.make_pub(ctx)

    # Signal handling. The supervisor sends SIGTERM (POSIX) or
    # CTRL_BREAK_EVENT ??SIGBREAK (Windows console group). Ctrl-C in a
    # dev shell sends SIGINT. Any of these should flip the stop event
    # and let the main loop exit so we hit the orderly shutdown.
    def _shutdown(*_: object) -> None:
        stop.set()
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _shutdown)  # type: ignore[attr-defined]

    # Heartbeat cadence: fixed 1 Hz. We track `next_tick` on the
    # monotonic clock and sleep up to it each iteration. If we ever fall
    # behind (debugger pause, GC stall) we just reset the cadence rather
    # than burn cycles trying to catch up ??heartbeats are a liveness
    # signal, not a rate target.
    heartbeat_hz = profile.subsystem_float("bus_broker", "fps_target", default_runtime_setting("bus_broker", "fps_target", DEFAULT_HEARTBEAT_HZ))
    period = 1.0 / max(1e-6, heartbeat_hz)
    next_tick = time.perf_counter()
    last_tick_mono_ns = time.perf_counter_ns()
    seq = 0
    print(f"[{args.proc}] bus broker up: "
          f"XSUB {bus.BUS_XSUB_BIND}  XPUB {bus.BUS_XPUB_BIND}", flush=True)
    try:
        while not stop.is_set():
            # Measure loop_hz from the gap to the previous heartbeat;
            # this is what subscribers see, not the requested 1 Hz.
            now_mono_ns = time.perf_counter_ns()
            dt = (now_mono_ns - last_tick_mono_ns) / 1e9
            loop_hz = (1.0 / dt) if dt > 0 else 0.0
            last_tick_mono_ns = now_mono_ns

            # BUS.md 禮6.9 heartbeat schema. We have nothing to put in
            # `loop_jitter_ms_p95` or `queue_depth` yet ??the broker
            # itself doesn't run a tight control loop and has no queue
            # in user space. They are zero for now; later phases of
            # processes that do have those numbers will fill them in.
            env = bus.make_envelope(args.proc, with_wall=True, seq=seq)
            env.update({
                "pid": _pid(),
                "loop_hz": loop_hz,
                "loop_jitter_ms_p95": 0.0,
                "queue_depth": 0,
            })
            bus.publish(pub, f"heartbeat.{args.proc}", env)
            seq += 1

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                # `Event.wait(timeout)` is cancellable ??a signal that
                # sets `stop` wakes us immediately rather than burning
                # the rest of the second.
                stop.wait(sleep_for)
            else:
                # Drifted past the next tick; reset cadence.
                next_tick = time.perf_counter()
    finally:
        pub.close(0)
        # destroy() closes any sockets the proxy thread still holds, which
        # makes zmq.proxy() raise ContextTerminated and lets the daemon
        # thread exit. Plain ctx.term() would block forever on those sockets.
        ctx.destroy(linger=0)
    print(f"[{args.proc}] stopped", flush=True)
    return 0


def _pid() -> int:
    import os
    return os.getpid()


if __name__ == "__main__":
    sys.exit(main())

