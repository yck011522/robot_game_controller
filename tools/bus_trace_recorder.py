"""Write selected live bus messages to one overwriteable JSONL trace file.

Typical manual runs:

    python tools/bus_trace_recorder.py --team a
    python tools/bus_trace_recorder.py --team a --duration-s 120
    python tools/bus_trace_recorder.py --topic state.full --topic cmd.robot.target.a

This is a short-term diagnostic tool for dense validation captures. It is
not the full EventRecorder described in docs/architecture/LOGGING.md; it
just subscribes to the existing ZMQ bus and records receive timestamps,
topic, and body for later inspection.
"""

from __future__ import annotations

import argparse
import json
import signal
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
DEFAULT_OUTPUT_PATH = REPO_ROOT / "logs" / "trace" / "bus_trace_latest.jsonl"


def main(argv: list[str] | None = None) -> int:
    """Record selected bus topics to JSONL until interrupted."""

    args = _parse_args(argv)
    topics = _resolve_topics(args.topic, args.team)
    out_path = _resolve_output_path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    ctx = zmq.Context.instance()
    sub = bus.make_sub(ctx, topics=topics)
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    count = 0
    stop = {"requested": False}
    deadline_s = (
        time.perf_counter() + float(args.duration_s)
        if float(args.duration_s) > 0.0
        else None
    )

    def _request_stop(*_: object) -> None:
        """Ask the blocking recorder loop to finish and close its file."""

        stop["requested"] = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _request_stop)  # type: ignore[attr-defined]

    next_status_s = time.perf_counter() + 5.0
    print(f"[bus_trace] writing {out_path}", flush=True)
    print(f"[bus_trace] topics: {', '.join(topics)}", flush=True)
    try:
        # Diagnostic captures intentionally replace the previous run. Keeping
        # one stable path makes collection and later automation predictable.
        with out_path.open("w", encoding="utf-8") as fh:
            while not stop["requested"]:
                if deadline_s is not None and time.perf_counter() >= deadline_s:
                    break
                events = dict(poller.poll(200))
                if sub not in events:
                    continue
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
        stop["requested"] = True
    finally:
        print(f"[bus_trace] stopped after {count} messages", flush=True)
        sub.close(0)
        ctx.destroy(linger=0)
    return 0


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse CLI arguments for the trace recorder."""

    parser = argparse.ArgumentParser(description="Record live bus messages to JSONL")
    parser.add_argument(
        "--output",
        default=None,
        help="Output JSONL path. Defaults to logs/trace/bus_trace_latest.jsonl.",
    )
    parser.add_argument(
        "--team",
        action="append",
        choices=("a", "b"),
        default=[],
        help="Record the standard control topics for this team. Repeatable.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Stop after this many seconds; zero records until launcher shutdown.",
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
    """Return the requested path or the stable latest-trace path."""

    if value:
        return Path(value).resolve()
    return DEFAULT_OUTPUT_PATH


def _resolve_topics(topics: list[str], teams: list[str]) -> tuple[str, ...]:
    """Build explicit or per-team diagnostic topic subscriptions."""

    if topics:
        return tuple(str(topic) for topic in topics)
    if not teams:
        return DEFAULT_TOPICS
    resolved = ["state.full", "heartbeat."]
    for team in teams:
        resolved.extend(
            [
                f"telem.haptic.{team}",
                f"cmd.haptic.{team}",
                f"cmd.haptic.reseat.{team}",
                f"cmd.robot.target.{team}",
                f"cmd.robot.recover.{team}",
                f"telem.robot.actual.{team}",
            ]
        )
    return tuple(resolved)


if __name__ == "__main__":
    raise SystemExit(main())
