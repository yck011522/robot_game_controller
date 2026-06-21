"""Send a text-mode probe directly to one scoreboard bucket display.

Typical usage from the repository root (stop the launcher first):
    python tools/scoreboard_text_probe.py A1 "GAME,OVER" --mode continuous
    python tools/scoreboard_text_probe.py A1 "GAME,OVER" --mode continuous --duration-s 8
    python tools/scoreboard_text_probe.py A1 "GAME,OVER" --mode single
    python tools/scoreboard_text_probe.py A1 "GAME OVER" --mode static
    python tools/scoreboard_text_probe.py B3 "ONE,TWO,THREE" --mode continuous --dry-run

Bucket IDs are resolved through ``config/device_ports_and_addr.yaml`` so this
tool follows the installation's physical A1/A2/A3/B1/B2/B3 panel wiring. Commas
inside ``text`` are sent unchanged and split the firmware's scrolling lines.
Continuous scrolling is a firmware-global setting, although mode and text are
addressed only to the selected display.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:  # pragma: no cover - reached only outside the project environment.
    serial = None

REPO_ROOT = Path(__file__).resolve().parent.parent  # Repository root used to locate src.
SRC = REPO_ROOT / "src"  # Runtime package directory added for direct script execution.
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.device_connection import require_serial_baudrate  # noqa: E402
from subsystems.scoreboard.layout import load_scoreboard_layout  # noqa: E402
from subsystems.scoreboard.transport import (  # noqa: E402
    MODE_SCROLL_UP,
    MODE_STATIC,
    cmd_enable,
    cmd_mode,
    cmd_scroll_continuous,
    cmd_text,
)

SERIAL_SETTINGS_KEY = "scoreboard"  # Device-config section containing the normal baud rate.
VALID_BUCKETS = ("A1", "A2", "A3", "B1", "B2", "B3")  # Accepted bucket IDs.
OPEN_SETTLE_S = 0.2  # Delay after opening USB serial before transmitting commands.
INTER_COMMAND_DELAY_S = 0.02  # Quiet gap between probe commands on the RS485 adapter.


def _bucket_id(value: str) -> str:
    """Normalize and validate a command-line bucket ID."""

    bucket = str(value).strip().upper()  # Canonical ID used by the wiring map.
    if bucket not in VALID_BUCKETS:
        choices = ", ".join(VALID_BUCKETS)  # Human-readable argparse error detail.
        raise argparse.ArgumentTypeError(f"bucket must be one of: {choices}")
    return bucket


def _text(value: str) -> str:
    """Validate text for the scoreboard's quoted ASCII line protocol."""

    if not value:
        raise argparse.ArgumentTypeError("text must not be empty")
    if any(character in value for character in ('"', "\r", "\n")):
        raise argparse.ArgumentTypeError("text cannot contain quotes or newlines")
    try:
        value.encode("ascii")
    except UnicodeEncodeError as exc:
        raise argparse.ArgumentTypeError("text must contain ASCII characters only") from exc
    return value


def build_commands(display: int, text: str, mode: str) -> list[bytes]:
    """Build the ordered firmware commands for one display probe.

    ``single`` and ``continuous`` select scroll-up mode while explicitly
    disabling or enabling the firmware-global repeat flag. ``static`` leaves
    the repeat flag unchanged because per-display mode 0 stops animation.
    """

    commands: list[bytes] = []  # Ordered lines written to the scoreboard bus.
    if mode == "single":
        commands.append(cmd_scroll_continuous(False))
        display_mode = MODE_SCROLL_UP  # One finite upward animation.
    elif mode == "continuous":
        commands.append(cmd_scroll_continuous(True))
        display_mode = MODE_SCROLL_UP  # Repeated upward animations.
    elif mode == "static":
        display_mode = MODE_STATIC  # Fixed text with no scrolling.
    else:
        raise ValueError(f"unsupported scoreboard mode: {mode!r}")

    commands.extend(
        (
            cmd_enable(display, True),
            cmd_mode(display, display_mode),
            cmd_text(display, text),
        )
    )
    return commands


def _write_commands(handle: object, commands: list[bytes]) -> None:
    """Write and flush an ordered command sequence to an open serial handle."""

    for command in commands:
        print(f"TX {command.decode('ascii').rstrip()}")
        handle.write(command)  # type: ignore[attr-defined]  # pyserial-compatible handle.
        handle.flush()  # type: ignore[attr-defined]  # Ensure the line leaves the OS buffer.
        time.sleep(INTER_COMMAND_DELAY_S)


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser for the scoreboard probe."""

    parser = argparse.ArgumentParser(
        description="Send static or scrolling text directly to one scoreboard bucket."
    )
    parser.add_argument(
        "bucket",
        type=_bucket_id,
        help="logical bucket ID: A1, A2, A3, B1, B2, or B3",
    )
    parser.add_argument(
        "text",
        type=_text,
        help='ASCII text; commas split scroll lines, for example "GAME,OVER"',
    )
    parser.add_argument(
        "--mode",
        choices=("static", "single", "continuous"),
        default="static",
        help="display behavior (default: static)",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="seconds to wait before returning this display to static; 0 leaves it running",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="override the configured scoreboard COM port",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=None,
        help="override the configured scoreboard baud rate",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print resolved wiring and commands without opening the COM port",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Resolve wiring, transmit the requested probe, and optionally stop it."""

    parser = build_parser()  # Parser also owns user-facing validation errors.
    args = parser.parse_args(argv)  # Parsed runtime controls for this probe.
    if args.duration_s < 0.0:
        parser.error("--duration-s must be zero or greater")

    layout = load_scoreboard_layout()  # Installation-specific COM port and bucket mapping.
    display = layout.bucket_displays[args.bucket]  # Physical 1-based display address.
    port = str(args.port or layout.port)  # Selected serial device, normally COM40.
    baud = int(args.baud or require_serial_baudrate(SERIAL_SETTINGS_KEY))
    commands = build_commands(display, args.text, args.mode)  # Exact wire sequence.

    print(
        f"bucket={args.bucket} display={display} port={port} baud={baud} "
        f"mode={args.mode} duration_s={args.duration_s:g}"
    )
    if args.dry_run:
        for command in commands:
            print(f"TX {command.decode('ascii').rstrip()}")
        if args.duration_s > 0.0:
            print(f"WAIT {args.duration_s:g}s")
            print(f"TX {cmd_mode(display, MODE_STATIC).decode('ascii').rstrip()}")
        return 0

    if serial is None:
        parser.error("pyserial is not installed in this Python environment")

    try:
        with serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1,
            write_timeout=1.0,
        ) as handle:
            time.sleep(OPEN_SETTLE_S)
            _write_commands(handle, commands)
            if args.duration_s > 0.0:
                print(f"Waiting {args.duration_s:g}s before returning display {display} to static")
                time.sleep(args.duration_s)
                _write_commands(handle, [cmd_mode(display, MODE_STATIC)])
    except (serial.SerialException, OSError, TimeoutError) as exc:
        print(f"scoreboard probe failed: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
