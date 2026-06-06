"""Collision broker ??ROUTER ??DEALER load balancer.

How this differs from the main bus broker
-----------------------------------------
The main bus uses XSUB/XPUB pub/sub. The collision channel uses
ROUTER/DEALER because it's request/reply with N planners on one side
and M workers on the other:

    JoggingPlanner(s) ??REQ??> tcp://127.0.0.1:5560 (ROUTER)
                                                     ?? zmq.proxy(...)
    CollisionWorker ? N <?REP?? tcp://127.0.0.1:5561 (DEALER)

ROUTER tags every incoming frame with the sender's identity so the
proxy can route the reply back to the right REQ socket. DEALER on the
worker-facing side round-robins requests across all connected REP
workers ??that's the actual load-balancing.

`zmq.proxy(router, dealer)` does both directions in one C-level loop.
Same shutdown trick as the main broker: `ctx.destroy(linger=0)` raises
`ContextTerminated` in the proxy thread.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.proc import banner, install_signal_handlers, parse_proc_args  # noqa: E402


DEFAULT_HEARTBEAT_HZ = 1.0


def _run_proxy(ctx: zmq.Context) -> None:
    router = ctx.socket(zmq.ROUTER)
    dealer = ctx.socket(zmq.DEALER)
    router.bind(bus.COLLISION_ROUTER_ENDPOINT)
    dealer.bind(bus.COLLISION_DEALER_ENDPOINT)
    try:
        zmq.proxy(router, dealer)
    except zmq.ContextTerminated:
        pass
    except zmq.ZMQError as exc:
        if exc.errno not in (zmq.ETERM, zmq.ENOTSOCK):
            raise
    finally:
        for sock in (router, dealer):
            try:
                sock.close(0)
            except zmq.ZMQError as exc:
                if exc.errno != zmq.ENOTSOCK:
                    raise


def main(argv: list[str] | None = None) -> int:
    args, _ = parse_proc_args(argv, default_proc="collision_broker")
    profile = load_profile(args.profile_path)

    ctx = zmq.Context.instance()
    stop = threading.Event()
    install_signal_handlers(stop.set)

    t = threading.Thread(target=_run_proxy, args=(ctx,),
                          name="collision_proxy", daemon=True)
    t.start()
    time.sleep(0.05)  # let binds settle before our heartbeat PUB connects

    pub = bus.make_pub(ctx)
    banner(args.proc,
           f"ready: ROUTER {bus.COLLISION_ROUTER_ENDPOINT} <-> "
           f"DEALER {bus.COLLISION_DEALER_ENDPOINT}")

    heartbeat_hz = profile.subsystem_float("collision_broker", "fps_target", default_runtime_setting("collision_broker", "fps_target", DEFAULT_HEARTBEAT_HZ))
    period = 1.0 / max(1e-6, heartbeat_hz)
    next_tick = time.perf_counter()
    last_mono_ns = time.perf_counter_ns()
    seq = 0
    try:
        while not stop.is_set():
            now_ns = time.perf_counter_ns()
            dt = (now_ns - last_mono_ns) / 1e9
            loop_hz = 1.0 / dt if dt > 0 else 0.0
            last_mono_ns = now_ns
            env = bus.make_envelope(args.proc, with_wall=True, seq=seq)
            env.update({"pid": _pid(), "loop_hz": loop_hz,
                        "loop_jitter_ms_p95": 0.0, "queue_depth": 0})
            bus.publish(pub, f"heartbeat.{args.proc}", env)
            seq += 1
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                stop.wait(sleep_for)
            else:
                next_tick = time.perf_counter()
    finally:
        pub.close(0)
        ctx.destroy(linger=0)
    banner(args.proc, "stopped")
    return 0


def _pid() -> int:
    import os
    return os.getpid()


if __name__ == "__main__":
    sys.exit(main())
