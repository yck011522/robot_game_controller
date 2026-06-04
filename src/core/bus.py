"""ZMQ bus helpers.

Single source of truth for endpoint constants, the wire-format envelope,
and `publish` / `recv` helpers used by every process on the main bus.
Wire format and topic conventions live in
[docs/architecture/BUS.md](../../docs/architecture/BUS.md).
"""

from __future__ import annotations

import json
import time
from typing import Any

import zmq

# ---- endpoints (BUS.md §2) ----------------------------------------------
BUS_XSUB_ENDPOINT = "tcp://127.0.0.1:5550"   # publishers connect here
BUS_XPUB_ENDPOINT = "tcp://127.0.0.1:5551"   # subscribers connect here

BUS_XSUB_BIND = "tcp://127.0.0.1:5550"       # broker binds these
BUS_XPUB_BIND = "tcp://127.0.0.1:5551"

COLLISION_ROUTER_ENDPOINT = "tcp://127.0.0.1:5560"
COLLISION_DEALER_ENDPOINT = "tcp://127.0.0.1:5561"

UI_REP_ENDPOINT = "tcp://127.0.0.1:5570"

EXTERNAL_PUB_BIND = "tcp://0.0.0.0:5552"     # Vision/Audio PC heartbeats


# ---- envelope helpers (BUS.md §3) ---------------------------------------

def make_envelope(producer: str, *, with_wall: bool = False, seq: int | None = None) -> dict[str, Any]:
    """Return the standard envelope fields. `producer` is the canonical
    process name (BUS.md §3.1). `with_wall` adds `ts_wall_ns`; required on
    `state.full` and `heartbeat.*`, optional elsewhere. `seq` is added
    only when the caller needs drop detection (BUS.md §4.4).
    """
    env: dict[str, Any] = {
        "ts_mono_ns": time.monotonic_ns(),
        "producer": producer,
    }
    if with_wall:
        env["ts_wall_ns"] = time.time_ns()
    if seq is not None:
        env["seq"] = seq
    return env


def publish(sock: zmq.Socket, topic: str, body: dict[str, Any]) -> None:
    """Send a two-frame multipart: topic bytes, JSON body bytes."""
    sock.send_multipart([
        topic.encode("ascii"),
        json.dumps(body, separators=(",", ":")).encode("utf-8"),
    ])


def recv(sock: zmq.Socket, *, flags: int = 0) -> tuple[str, dict[str, Any]]:
    """Receive a two-frame multipart and return (topic, parsed-body)."""
    frames = sock.recv_multipart(flags=flags)
    topic = frames[0].decode("ascii")
    body = json.loads(frames[1].decode("utf-8")) if len(frames) > 1 else {}
    return topic, body


# ---- socket factories ---------------------------------------------------

def make_pub(ctx: zmq.Context, *, endpoint: str = BUS_XSUB_ENDPOINT) -> zmq.Socket:
    sock = ctx.socket(zmq.PUB)
    sock.connect(endpoint)
    return sock


def make_sub(ctx: zmq.Context, *, topics: list[str] | None = None,
             endpoint: str = BUS_XPUB_ENDPOINT, conflate: bool = False) -> zmq.Socket:
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
