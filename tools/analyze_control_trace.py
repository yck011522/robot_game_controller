"""Summarize protective-stop and resume behavior from a bus JSONL trace.

Typical runs from the repository root:

    python tools/analyze_control_trace.py
    python tools/analyze_control_trace.py --input logs/trace/bus_trace_latest.jsonl
    python tools/analyze_control_trace.py --team a --motion-threshold-deg-s 0.1

The analyzer is read-only. It correlates recorder receive timestamps rather
than producer monotonic clocks, which are intentionally process-local.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRACE = REPO_ROOT / "logs" / "trace" / "bus_trace_latest.jsonl"


def main(argv: list[str] | None = None) -> int:
    """Parse one trace and print compact recovery diagnostics."""

    args = _parse_args(argv)
    rows, malformed = _load_rows(Path(args.input))
    if not rows:
        print(f"No valid rows found in {args.input}")
        return 1

    first_ns = int(rows[0]["ts_recv_wall_ns"])
    last_ns = int(rows[-1]["ts_recv_wall_ns"])
    counts = Counter(str(row.get("topic", "")) for row in rows)
    print(
        f"Trace: {args.input}\n"
        f"Duration: {(last_ns - first_ns) / 1e9:.3f} s  "
        f"Rows: {len(rows)}  Malformed: {malformed}"
    )
    print("Topics: " + ", ".join(f"{key}={value}" for key, value in counts.items()))

    episodes = _recovery_episodes(rows, team=args.team)
    if not episodes:
        print("No protective-stop recovery episodes found.")
        return 0

    for index, episode in enumerate(episodes, 1):
        _print_episode(
            rows,
            episode,
            index=index,
            team=args.team,
            gear_ratio=_parse_gear_ratio(args.gear_ratio),
            motion_threshold_rad_s=math.radians(args.motion_threshold_deg_s),
        )
    return 0


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse trace path, team, gear ratio, and movement threshold."""

    parser = argparse.ArgumentParser(description="Analyze robot recovery bus trace")
    parser.add_argument("--input", default=str(DEFAULT_TRACE), help="Input JSONL path")
    parser.add_argument("--team", choices=("a", "b"), default="a")
    parser.add_argument(
        "--gear-ratio",
        default="0.1,0.1,0.1,0.1,0.1,0.1",
        help="Six comma-separated dial-to-robot ratios used to estimate intent.",
    )
    parser.add_argument(
        "--motion-threshold-deg-s",
        type=float,
        default=0.1,
        help="Measured joint speed considered physical motion.",
    )
    return parser.parse_args(argv)


def _load_rows(path: Path) -> tuple[list[dict[str, Any]], int]:
    """Load valid JSON objects and count malformed JSONL rows."""

    rows: list[dict[str, Any]] = []
    malformed = 0
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                malformed += 1
                continue
            if isinstance(row, dict) and isinstance(row.get("body"), dict):
                rows.append(row)
            else:
                malformed += 1
    return rows, malformed


def _recovery_episodes(
    rows: list[dict[str, Any]], *, team: str
) -> list[dict[str, int | None]]:
    """Identify protective-stop, recovery-request, and unpause timestamps."""

    actual_topic = f"telem.robot.actual.{team}"
    recover_topic = f"cmd.robot.recover.{team}"
    episodes: list[dict[str, int | None]] = []
    active: dict[str, int | None] | None = None
    was_paused = False
    for row in rows:
        timestamp_ns = int(row["ts_recv_wall_ns"])
        topic = row.get("topic")
        body = row["body"]
        if topic == actual_topic:
            status = body.get("robot_status")
            protective = bool(
                isinstance(status, dict) and status.get("protective_stopped", False)
            )
            if protective and active is None:
                active = {"fault_ns": timestamp_ns, "request_ns": None, "resume_ns": None}
        elif topic == recover_topic and active is not None:
            active["request_ns"] = timestamp_ns
        elif topic == "state.full":
            paused = bool(body.get("paused", False))
            if active is not None and was_paused and not paused:
                active["resume_ns"] = timestamp_ns
                episodes.append(active)
                active = None
            was_paused = paused
    return episodes


def _print_episode(
    rows: list[dict[str, Any]],
    episode: dict[str, int | None],
    *,
    index: int,
    team: str,
    gear_ratio: list[float],
    motion_threshold_rad_s: float,
) -> None:
    """Print timings and first-command diagnostics for one recovery episode."""

    fault_ns = int(episode["fault_ns"] or 0)
    request_ns = int(episode["request_ns"] or 0)
    resume_ns = int(episode["resume_ns"] or 0)
    print(f"\nRecovery {index}")
    print(f"  stopped -> green request: {(request_ns - fault_ns) / 1e9:.3f} s")
    print(f"  green request -> resumed: {(resume_ns - request_ns) / 1e9:.3f} s")
    print(f"  total stopped -> resumed: {(resume_ns - fault_ns) / 1e9:.3f} s")

    actual_topic = f"telem.robot.actual.{team}"
    command_topic = f"cmd.robot.target.{team}"
    haptic_topic = f"telem.haptic.{team}"
    latest_actual: list[float] | None = None
    latest_dial: list[float] | None = None
    previous_command: list[float] | None = None
    first_command: tuple[
        int,
        dict[str, Any],
        list[float] | None,
        list[float] | None,
        list[float] | None,
    ] | None = None
    first_motion_ns: int | None = None
    queue_max_count = 0
    queue_max_ms = 0.0

    for row in rows:
        timestamp_ns = int(row["ts_recv_wall_ns"])
        if timestamp_ns > resume_ns + int(20e9):
            break
        topic = row.get("topic")
        body = row["body"]
        if topic == actual_topic:
            q = body.get("q_rad")
            if isinstance(q, list) and len(q) >= 6:
                latest_actual = [float(value) for value in q[:6]]
            queue = body.get("command_queue")
            if isinstance(queue, dict):
                queue_max_count = max(queue_max_count, int(queue.get("max_drain_count", 0)))
                queue_max_ms = max(queue_max_ms, float(queue.get("max_drain_ms", 0.0)))
            qd = body.get("qd_rad_s")
            if (
                timestamp_ns >= resume_ns
                and first_motion_ns is None
                and isinstance(qd, list)
                and max(abs(float(value)) for value in qd[:6]) > motion_threshold_rad_s
            ):
                first_motion_ns = timestamp_ns
        elif topic == haptic_topic:
            dial = body.get("dial_pos_rad")
            if isinstance(dial, list) and len(dial) >= 6:
                latest_dial = [float(value) for value in dial[:6]]
        elif topic == command_topic:
            command = body.get("q_target_rad")
            if isinstance(command, list) and len(command) >= 6:
                command_q = [float(value) for value in command[:6]]
                if timestamp_ns >= resume_ns and first_command is None:
                    first_command = (
                        timestamp_ns,
                        body,
                        list(latest_actual) if latest_actual is not None else None,
                        list(previous_command) if previous_command is not None else None,
                        list(latest_dial) if latest_dial is not None else None,
                    )
                previous_command = command_q

    print(f"  largest RobotIO drain batch: {queue_max_count} messages in {queue_max_ms:.3f} ms")
    if first_command is not None:
        timestamp_ns, body, actual_q, previous_q, dial_q = first_command
        command_q = [float(value) for value in body["q_target_rad"][:6]]
        command_error = _max_delta_deg(command_q, actual_q)
        command_step = _max_delta_deg(command_q, previous_q)
        dial_error = None
        if actual_q is not None and dial_q is not None:
            desired_q = [dial_q[axis] * gear_ratio[axis] for axis in range(6)]
            dial_error = _max_delta_deg(desired_q, actual_q)
        print(f"  first command after resume: {(timestamp_ns - resume_ns) / 1e9:.3f} s")
        print(f"  first command vs actual: {command_error:.3f} deg")
        print(f"  first command step: {command_step:.3f} deg")
        if dial_error is not None:
            print(f"  haptic desired vs actual: {dial_error:.3f} deg")
    if first_motion_ns is None:
        print("  measured robot motion: not observed within 20 s")
    else:
        print(f"  measured robot motion after resume: {(first_motion_ns - resume_ns) / 1e9:.3f} s")


def _parse_gear_ratio(value: str) -> list[float]:
    """Parse exactly six comma-separated gear ratios."""

    values = [float(item.strip()) for item in value.split(",")]
    if len(values) != 6:
        raise ValueError("--gear-ratio must contain exactly six values")
    return values


def _max_delta_deg(first: list[float], second: list[float] | None) -> float:
    """Return maximum absolute per-joint difference in degrees."""

    if second is None:
        return float("nan")
    return math.degrees(max(abs(first[axis] - second[axis]) for axis in range(6)))


if __name__ == "__main__":
    raise SystemExit(main())
