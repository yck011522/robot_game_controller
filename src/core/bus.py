"""ZMQ bus helpers.

Single source of truth for endpoint constants, the wire-format envelope,
and `publish` / `recv` helpers used by every process on the main bus.
Wire format and topic conventions live in
[docs/architecture/BUS.md](../../docs/architecture/BUS.md).

Topology recap (BUS.md 禮1)
--------------------------
- One **XSUB/XPUB broker** at `:5550` (XSUB, publishers connect) and
  `:5551` (XPUB, subscribers connect). Every process on the bus
  connects ??only the broker binds.
- One **collision broker** (ROUTER/DEALER) at `:5560` / `:5561` for
  the planner ??worker request-response load-balancer (P2+).
- One **UI ??GC REQ/REP** socket at `:5570` (P4+).

This module owns the endpoint strings so swapping `tcp://` for
`ipc://` later ??if profiling shows TCP overhead matters ??is a
one-place change.

Wire format (BUS.md 禮3)
-----------------------
Every message on the main bus is a two-frame ZMQ multipart:

    frame 0: topic bytes      (e.g. b"telem.haptic.a")
    frame 1: JSON body bytes  (UTF-8, compact JSON)

Topic is plain ASCII; SUB filtering is a byte-prefix match, so the
hierarchical naming in BUS.md 禮5 is what makes `setsockopt(SUBSCRIBE,
b"heartbeat.")` work as "everything starting with heartbeat.". Bodies
are always JSON objects (never bare arrays/strings) so fields can be
added without a breaking change.
"""

from __future__ import annotations

import json
import time
from typing import Any

import zmq

# ---- endpoints (BUS.md 禮2) ----------------------------------------------
# Publishers connect their PUB sockets here; the broker binds this.
BUS_XSUB_ENDPOINT = "tcp://127.0.0.1:5550"
# Subscribers connect their SUB sockets here; the broker binds this.
BUS_XPUB_ENDPOINT = "tcp://127.0.0.1:5551"

# Bind-side aliases. Identical to the connect-side strings today, but
# kept separate so a future move to e.g. `ipc://` keeps the producer /
# consumer / broker code free of conditional logic.
BUS_XSUB_BIND = "tcp://127.0.0.1:5550"
BUS_XPUB_BIND = "tcp://127.0.0.1:5551"

# Collision check load-balancer (P2+). JoggingPlanner REQs connect to
# the ROUTER endpoint; CollisionWorker REPs connect to the DEALER side.
COLLISION_ROUTER_ENDPOINT = "tcp://127.0.0.1:5560"
COLLISION_DEALER_ENDPOINT = "tcp://127.0.0.1:5561"

# UI ??GC commands (P4+).
UI_REP_ENDPOINT = "tcp://127.0.0.1:5570"

# Bind for the external Vision/Audio PC heartbeat tributary (P9+).
EXTERNAL_PUB_BIND = "tcp://0.0.0.0:5552"


# ---- envelope helpers (BUS.md 禮3) ---------------------------------------

def make_envelope(producer: str, *, with_wall: bool = False, seq: int | None = None) -> dict[str, Any]:
    """Return the standard envelope fields for a bus message.

    Args:
        producer: Canonical process name (BUS.md 禮3.1, e.g.
            `"haptic_io.a"`). Written into the body's `producer` field.
        with_wall: Add `ts_wall_ns` (wall clock, ns since Unix epoch).
            Required on `state.full` and `heartbeat.*`; optional but
            recommended on per-game state that the replay tool needs to
            align across machines. Off elsewhere to save a few bytes.
        seq: Per-publisher monotonic counter. Add only on topics where
            drop detection matters (BUS.md 禮4.4) ??`state.full`,
            `req.collision_check`, `rep.collision_result`. Leave `None`
            for everything else.

    Returns:
        A fresh dict the caller can extend with topic-specific fields.

    `ts_mono_ns` always comes from `time.perf_counter_ns()` so per-process
    jitter / loop-rate stats inside the producer are sane. It must NEVER
    be compared across processes (different epoch per process).
    """
    env: dict[str, Any] = {
        "ts_mono_ns": time.perf_counter_ns(),
        "producer": producer,
    }
    if with_wall:
        env["ts_wall_ns"] = time.time_ns()
    if seq is not None:
        env["seq"] = seq
    return env


def publish(sock: zmq.Socket, topic: str, body: dict[str, Any]) -> None:
    """Send a two-frame multipart: topic bytes, JSON body bytes.

    JSON is encoded with `separators=(",", ":")` for compactness ??these
    messages can run at 100 Hz ? multiple processes; the saved bytes
    matter at the broker.
    """
    sock.send_multipart([
        topic.encode("ascii"),
        json.dumps(body, separators=(",", ":")).encode("utf-8"),
    ])


def recv(sock: zmq.Socket, *, flags: int = 0) -> tuple[str, dict[str, Any]]:
    """Receive one two-frame multipart and return `(topic, parsed_body)`.

    `flags=zmq.NOBLOCK` raises `zmq.Again` if nothing is queued; useful
    in poll-loop drains. With the default `flags=0` the call blocks
    until a message arrives.
    """
    frames = sock.recv_multipart(flags=flags)
    topic = frames[0].decode("ascii")
    # Tolerate single-frame messages (e.g. raw subscription updates that
    # leaked through) by treating them as empty-body. Should not happen
    # under our own wire format but it's cheap insurance.
    body = json.loads(frames[1].decode("utf-8")) if len(frames) > 1 else {}
    return topic, body


# ---- socket factories ---------------------------------------------------

def make_pub(ctx: zmq.Context, *, endpoint: str = BUS_XSUB_ENDPOINT) -> zmq.Socket:
    """Create a PUB socket connected to the bus broker.

    Slow-joiner caveat: a brand-new PUB that publishes immediately can
    lose messages to subscribers whose subscriptions haven't propagated
    yet. Callers that do a one-shot send (like `tools/bus_poke.py`)
    should sleep ~200 ms between `connect()` and the first `send()`.
    Long-running publishers don't need to care.
    """
    sock = ctx.socket(zmq.PUB)
    sock.connect(endpoint)
    return sock


def make_sub(ctx: zmq.Context, *, topics: list[str] | None = None,
             endpoint: str = BUS_XPUB_ENDPOINT, conflate: bool = False) -> zmq.Socket:
    """Create a SUB socket connected to the bus broker.

    Args:
        topics: List of topic-prefix strings to subscribe to. ZMQ does
            byte-prefix matching, so `"heartbeat."` matches every
            `heartbeat.<proc>`. Pass `None` to subscribe to everything
            (used by `tools/bus_tap.py` and the EventRecorder).
        conflate: When True, the socket keeps only the latest message
            per peer in its inbound queue. **PITFALL:** ZMQ's CONFLATE
            is documented as incompatible with SUB-side subscription
            filtering ??combining a topic prefix with `conflate=True`
            silently drops every message. Either subscribe to `""`
            (everything) and filter in Python, or skip CONFLATE and
            drain the queue per tick keeping only the last sample.
            Use only on tap-style sockets that subscribe to `""`.
    """
    sock = ctx.socket(zmq.SUB)
    if conflate:
        sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(endpoint)
    if topics is None:
        sock.setsockopt(zmq.SUBSCRIBE, b"")
    else:
        for t in topics:
            sock.setsockopt(zmq.SUBSCRIBE, t.encode("ascii"))
    return sock

