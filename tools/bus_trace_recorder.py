"""Write selected live bus messages to a timestamped JSONL trace file.

This is a short-term diagnostic tool for dense validation captures. It is
not the full EventRecorder described in docs/architecture/LOGGING.md; it
just subscribes to the existing ZMQ bus and records receive timestamps,
topic, and body for later inspection.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402


REPO_ROOT = _SRC.parent
DEFAULT_TOPICS = (
    "state.full",
    "telem.haptic.b",
    "cmd.haptic.b",
    "cmd.haptic.reseat.b",
    "cmd.robot.target.b",
    "cmd.robot.recover.b",
    "telem.robot.actual.b",
    "heartbeat.",
)


def main(argv: list[str] | None = None) -> int:
    """Record selected bus topics to JSONL until interrupted."""

    args = _parse_args(argv)
    topics = tuple(args.topic) if args.topic else DEFAULT_TOPICS
    out_path = _resolve_output_path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    ctx = zmq.Context.instance()
    sub = bus.make_sub(ctx, topics=topics)
    count = 0
    next_status_s = time.perf_counter() + 5.0
    print(f"[bus_trace] writing {out_path}", flush=True)
    print(f"[bus_trace] topics: {', '.join(topics)}", flush=True)
    try:
        with out_path.open("a", encoding="utf-8") as fh:
            while True:
                topic, body = bus.recv(sub)
                row = {
                    "ts_recv_wall_ns": time.time_ns(),
                    "ts_recv_mono_ns": time.perf_counter_ns(),
                    "topic": topic,
                    "body": body,
                }
                fh.write(json.dumps(row, separators=(",", ":")) + "\n")
                count += 1
                if count % max(1, args.flush_every) == 0:
                    fh.flush()
                now_s = time.perf_counter()
                if now_s >= next_status_s:
                    print(f"[bus_trace] recorded {count} messages", flush=True)
                    next_status_s = now_s + 5.0
    except KeyboardInterrupt:
        print(f"\n[bus_trace] stopped after {count} messages", flush=True)
        return 0
    finally:
        sub.close(0)
        ctx.destroy(linger=0)


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse CLI arguments for the trace recorder."""

    parser = argparse.ArgumentParser(description="Record live bus messages to JSONL")
    parser.add_argument(
        "--output",
        default=None,
        help="Output JSONL path. Defaults to logs/trace/bus_trace_<wall-time>.jsonl.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help=(
            "Topic prefix to subscribe to. Repeat for multiple prefixes. "
            "Defaults to Team B validation topics."
        ),
    )
    parser.add_argument(
        "--flush-every",
        type=int,
        default=1,
        help="Flush to disk every N messages. Default 1 favors crash-safe diagnostics.",
    )
    return parser.parse_args(argv)


def _resolve_output_path(value: str | None) -> Path:
    """Return the requested path or a timestamped default trace path."""

    if value:
        return Path(value).resolve()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return REPO_ROOT / "logs" / "trace" / f"bus_trace_{stamp}.jsonl"


if __name__ == "__main__":
    raise SystemExit(main())
