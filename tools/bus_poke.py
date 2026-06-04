"""One-shot bus publisher.

Usage:  python tools/bus_poke.py <topic> '<json-body>'

Example:
  python tools/bus_poke.py test.ping '{"hello": "world"}'

A 200 ms post-connect grace period is used so the XSUB/XPUB broker has
time to propagate the subscription before the message goes out (PUB's
slow-joiner: anything sent before subscribers are wired up is dropped).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="One-shot bus publisher")
    ap.add_argument("topic", help="topic to publish on (e.g. test.ping)")
    ap.add_argument("body", help="JSON-encoded message body (must be a JSON object)")
    ap.add_argument("--producer", default="bus_poke",
                    help="value to insert into the body's 'producer' field if missing")
    ap.add_argument("--endpoint", default=bus.BUS_XSUB_ENDPOINT,
                    help="XSUB endpoint to connect to")
    ap.add_argument("--grace-ms", type=int, default=200,
                    help="ms to wait after connect before publishing (slow-joiner mitigation)")
    args = ap.parse_args(argv)

    try:
        body = json.loads(args.body)
    except json.JSONDecodeError as e:
        print(f"[bus_poke] body is not valid JSON: {e}", file=sys.stderr)
        return 2
    if not isinstance(body, dict):
        print("[bus_poke] body must be a JSON object (BUS.md 禮3)", file=sys.stderr)
        return 2
    body.setdefault("producer", args.producer)
    body.setdefault("ts_mono_ns", time.perf_counter_ns())

    ctx = zmq.Context.instance()
    pub = bus.make_pub(ctx, endpoint=args.endpoint)
    time.sleep(args.grace_ms / 1000.0)
    bus.publish(pub, args.topic, body)
    # Give the broker a moment to flush before we close the context.
    time.sleep(0.05)
    pub.close(0)
    ctx.term()
    print(f"[bus_poke] sent on {args.topic}: {json.dumps(body, separators=(',', ':'))}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
