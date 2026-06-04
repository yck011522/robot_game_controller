"""Bus broker entry point. Run via `python -m apps.bus_broker --profile ... --proc bus_broker`.

See module docstring in `__init__.py` and [SUPERVISOR.md §3](../../../docs/architecture/SUPERVISOR.md#3-spawn-contract).
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

# Bootstrap sys.path so this file is runnable as `python -m apps.bus_broker`
# from the repo root (the supervisor sets PYTHONPATH=src for spawned
# children; this is the fallback for direct invocation).
_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import load as load_profile  # noqa: E402


HEARTBEAT_HZ = 1.0


def _run_proxy(ctx: zmq.Context, stop: threading.Event) -> None:
    xsub = ctx.socket(zmq.XSUB)
    xpub = ctx.socket(zmq.XPUB)
    xsub.bind(bus.BUS_XSUB_BIND)
    xpub.bind(bus.BUS_XPUB_BIND)
    try:
        # zmq.proxy blocks until a socket is closed; we close them on stop.
        zmq.proxy(xsub, xpub)
    except zmq.ContextTerminated:
        pass
    finally:
        xsub.close(0)
        xpub.close(0)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="XSUB/XPUB bus broker")
    ap.add_argument("--profile", required=True, help="path to profile YAML")
    ap.add_argument("--proc", default="bus_broker",
                    help="canonical process name (used as producer field)")
    args = ap.parse_args(argv)

    # Loading the profile here mirrors what every other process does on
    # startup. We don't read any field — just confirm the YAML parses and
    # validates, so a broken profile fails loud before binding sockets.
    load_profile(args.profile)

    ctx = zmq.Context.instance()
    stop = threading.Event()

    proxy_thread = threading.Thread(target=_run_proxy, args=(ctx, stop),
                                     name="bus_proxy", daemon=True)
    proxy_thread.start()

    # Wait a beat so the proxy's binds settle before our PUB connects.
    time.sleep(0.05)
    pub = bus.make_pub(ctx)

    def _shutdown(*_: object) -> None:
        stop.set()
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _shutdown)  # type: ignore[attr-defined]

    period = 1.0 / HEARTBEAT_HZ
    next_tick = time.monotonic()
    last_tick_mono_ns = time.monotonic_ns()
    seq = 0
    print(f"[{args.proc}] bus broker up: "
          f"XSUB {bus.BUS_XSUB_BIND}  XPUB {bus.BUS_XPUB_BIND}", flush=True)
    try:
        while not stop.is_set():
            now_mono_ns = time.monotonic_ns()
            dt = (now_mono_ns - last_tick_mono_ns) / 1e9
            loop_hz = (1.0 / dt) if dt > 0 else 0.0
            last_tick_mono_ns = now_mono_ns
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
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                stop.wait(sleep_for)
            else:
                # We're behind schedule; reset the cadence so we don't burn cycles catching up.
                next_tick = time.monotonic()
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
