"""Small console-print helpers shared by runtime processes."""

from __future__ import annotations

from datetime import datetime


def log_line(source: str, message: str) -> None:
    """Print one timestamped process log line to stdout.

    Called by runtime modules that still use lightweight console prints instead
    of the Python logging package. ``source`` becomes the bracketed channel name
    (for example ``game_controller``), and ``message`` is the already-formatted
    event text. The timestamp is local wall-clock time so operator console traces
    can be read directly while comparing nearby events.
    """

    now = datetime.now()  # Local wall-clock timestamp for human console traces.
    timestamp = f"{now:%H:%M:%S}.{now.microsecond // 1000:03d}"
    print(f"{timestamp} [{source}] {message}", flush=True)
