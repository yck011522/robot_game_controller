"""SUB-all bus subscriber. Pretty-prints every message on the main bus.

Usage:  python tools/bus_tap.py                     # all topics
        python tools/bus_tap.py --topics state.full heartbeat.
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
    ap = argparse.ArgumentParser(description="SUB-all bus tap")
    ap.add_argument("--topics", nargs="*", default=None,
                    help="topic prefixes to subscribe to (default: all)")
    ap.add_argument("--endpoint", default=bus.BUS_XPUB_ENDPOINT,
                    help="XPUB endpoint to connect to")
    ap.add_argument("--compact", action="store_true",
                    help="single-line JSON output (default: indented)")
    args = ap.parse_args(argv)

    ctx = zmq.Context.instance()
    sub = bus.make_sub(ctx, topics=args.topics, endpoint=args.endpoint)
    print(f"[bus_tap] connected to {args.endpoint}; subscribed to "
          f"{args.topics if args.topics else 'ALL'}", flush=True)

    try:
        while True:
            topic, body = bus.recv(sub)
            wall = body.get("ts_wall_ns")
            wall_str = ""
            if wall is not None:
                wall_str = f" wall={time.strftime('%H:%M:%S', time.localtime(wall/1e9))}"
            head = f"{topic}{wall_str}"
            if args.compact:
                print(f"{head}  {json.dumps(body, separators=(',', ':'))}", flush=True)
            else:
                print(f"--- {head}\n{json.dumps(body, indent=2)}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        sub.close(0)
        ctx.term()
    return 0


if __name__ == "__main__":
    sys.exit(main())
