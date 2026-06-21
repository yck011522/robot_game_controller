"""Read one haptic dial controller's firmware configuration over USB serial.

This utility is read-only: it sends version, identity, and parameter query
commands but never writes a parameter or changes the controller identity.

Typical usage from the repository root:
    python tools/read_haptic_config.py COM48
    python tools/read_haptic_config.py --port COM48
    python tools/read_haptic_config.py COM48 --timeout 2.0
    python tools/read_haptic_config.py COM48 --baud 115200 --raw

Stop the launcher before running this utility so the selected COM port is not
already owned by ``haptic_io``.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

try:
    import serial
except ImportError:  # pragma: no cover - only reached outside the project environment.
    serial = None

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.device_connection import require_serial_baudrate  # noqa: E402


SERIAL_SETTINGS_KEY = "haptic_dial"  # device config section containing the normal dial baud rate
DEFAULT_TIMEOUT_S = 1.5  # maximum wait for each individual firmware response
OPEN_SETTLE_S = 0.2  # short delay after port open for USB serial and firmware telemetry to settle


@dataclass(frozen=True)
class ParameterDefinition:
    """Describe one queryable firmware parameter and how its wire value is displayed."""

    name: str  # exact parameter name accepted by the firmware S command
    unit: str  # human-readable unit printed after the decoded value
    scale: float = 1.0  # divisor converting the integer wire value to its engineering value


PARAMETERS = (
    ParameterDefinition("tracking_kp", "", 1000.0),
    ParameterDefinition("tracking_kd", "", 1000.0),
    ParameterDefinition("tracking_max_torque", "A", 1000.0),
    ParameterDefinition("bounds_kp", "", 1000.0),
    ParameterDefinition("bounds_max_torque", "A", 1000.0),
    ParameterDefinition("detent_kp", "", 1000.0),
    ParameterDefinition("detent_distance", "decideg", 1000.0),
    ParameterDefinition("detent_max_torque", "A", 1000.0),
    ParameterDefinition("vibration_amplitude", "A", 1000.0),
    ParameterDefinition("vibration_pulse_interval_ms", "ms"),
    ParameterDefinition("oob_kick_amplitude", "A", 1000.0),
    ParameterDefinition("oob_kick_pulse_interval_ms", "ms"),
    ParameterDefinition("enable_tracking", "bool"),
    ParameterDefinition("enable_bounds_restoration", "bool"),
    ParameterDefinition("enable_oob_kick", "bool"),
    ParameterDefinition("enable_detent", "bool"),
    ParameterDefinition("enable_vibration", "bool"),
    ParameterDefinition("telemetry_interval", "ms"),
)

STATUS_NAMES = (
    "tracking",
    "bounds_restoration",
    "oob_kick",
    "detent",
    "vibration",
    "outside_bounds",
    "fault",
)


class HapticConfigReader:
    """Send sequenced read queries while filtering interleaved telemetry lines."""

    def __init__(self, ser, *, timeout_s: float, show_raw: bool = False) -> None:
        self._serial = ser  # already-open pyserial-compatible transport
        self._timeout_s = timeout_s  # response deadline applied independently to each query
        self._show_raw = show_raw  # whether every received non-empty line is printed
        self._seq = 0  # monotonically increasing host command sequence number
        self.latest_telemetry: list[str] | None = None  # newest valid eight-field T frame seen

    def query_version(self) -> str:
        """Return the firmware version reported by a ``V`` query."""

        seq = self._next_seq()
        fields = self._query(f"V,{seq}", lambda parts: parts[:2] == ["V", str(seq)])
        if len(fields) < 3:
            raise RuntimeError(f"malformed version response: {','.join(fields)}")
        return ",".join(fields[2:])

    def query_identity(self) -> int:
        """Return the persistent dial ID reported by an ``I`` query."""

        seq = self._next_seq()
        fields = self._query(f"I,{seq}", lambda parts: parts[:2] == ["I", str(seq)])
        if len(fields) != 3:
            raise RuntimeError(f"malformed identity response: {','.join(fields)}")
        return int(fields[2])

    def query_parameter(self, name: str) -> str:
        """Return one parameter's raw wire value without modifying it."""

        seq = self._next_seq()
        expected = ["S", str(seq), name]
        fields = self._query(
            f"S,{seq},{name}", lambda parts: parts[:3] == expected
        )
        if len(fields) != 4:
            raise RuntimeError(f"malformed parameter response: {','.join(fields)}")
        return fields[3]

    def _next_seq(self) -> int:
        """Allocate the next host sequence number for response matching."""

        self._seq += 1
        return self._seq

    def _query(self, command: str, matches: Callable[[list[str]], bool]) -> list[str]:
        """Send one ASCII command and return its matching comma-separated response."""

        self._serial.write((command + "\n").encode("ascii"))
        deadline = time.monotonic() + self._timeout_s  # absolute deadline unaffected by telemetry traffic
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip().rstrip("\r")
            if not line:
                continue
            if self._show_raw:
                print(f"  RX {line}")
            parts = line.split(",")
            if parts[0] == "T" and len(parts) == 8:
                self.latest_telemetry = parts
            if matches(parts):
                return parts
        raise TimeoutError(f"no matching response to {command!r} within {self._timeout_s:.1f}s")


def format_parameter(definition: ParameterDefinition, raw_value: str) -> str:
    """Format one raw firmware value with its decoded engineering value."""

    if raw_value == "?":
        return "? (unsupported by firmware)"
    try:
        numeric_value = int(raw_value)  # protocol parameter values are decimal integers
    except ValueError:
        return f"{raw_value} (non-integer response)"
    if definition.unit == "bool":
        state = "ON" if numeric_value else "OFF"
        return f"{raw_value} ({state})"
    decoded_value = numeric_value / definition.scale
    decoded_text = f"{decoded_value:g}"
    suffix = f" {definition.unit}" if definition.unit else ""
    return f"{raw_value} ({decoded_text}{suffix})"


def format_status(telemetry: list[str] | None) -> str:
    """Decode the latest telemetry status bitfield, if one was observed."""

    if telemetry is None:
        return "not observed"
    try:
        status_bits = int(telemetry[7])  # decimal firmware bitfield from the final T-frame field
    except (IndexError, ValueError):
        return "malformed telemetry"
    states = [
        f"{name}={'ON' if status_bits & (1 << bit) else 'OFF'}"
        for bit, name in enumerate(STATUS_NAMES)
    ]
    return f"{status_bits} (" + ", ".join(states) + ")"


def resolve_port(args: argparse.Namespace, parser: argparse.ArgumentParser) -> str:
    """Resolve the positional/optional COM-port forms and reject conflicts."""

    positional = args.com_port  # convenient form used by normal operator commands
    optional = args.port  # explicit --port form useful in scripts
    if positional and optional and positional.upper() != optional.upper():
        parser.error("positional COM port and --port specify different controllers")
    selected = optional or positional
    if not selected:
        parser.error("a COM port is required, for example COM48")
    return str(selected).upper()


def read_controller(port: str, *, baudrate: int, timeout_s: float, show_raw: bool) -> int:
    """Open one controller, query its complete configuration, and print a report."""

    ser = serial.Serial()  # configure control lines before open to avoid an avoidable ESP reset
    ser.port = port
    ser.baudrate = baudrate
    ser.timeout = min(0.1, timeout_s)  # short readline polling interval within the overall query deadline
    ser.write_timeout = timeout_s
    ser.dtr = False
    ser.rts = False

    try:
        ser.open()
        time.sleep(OPEN_SETTLE_S)
        ser.reset_input_buffer()
        reader = HapticConfigReader(ser, timeout_s=timeout_s, show_raw=show_raw)
        firmware_version = reader.query_version()
        dial_id = reader.query_identity()
        values = {
            definition.name: reader.query_parameter(definition.name)
            for definition in PARAMETERS
        }

        print(f"Haptic controller on {port} at {baudrate} baud")
        print(f"Firmware version : {firmware_version}")
        print(f"Dial ID          : {dial_id}")
        print("Parameters:")
        for definition in PARAMETERS:
            formatted = format_parameter(definition, values[definition.name])
            print(f"  {definition.name:<31} {formatted}")
        print(f"Latest status    : {format_status(reader.latest_telemetry)}")
        return 0
    finally:
        if ser.is_open:
            ser.close()


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser for direct and scripted use."""

    parser = argparse.ArgumentParser(
        description="Read all persistent/runtime settings from one haptic dial controller."
    )
    parser.add_argument("com_port", nargs="?", help="controller port, for example COM48")
    parser.add_argument("--port", help="controller port as an explicit named argument")
    parser.add_argument(
        "--baud",
        type=int,
        default=None,
        help="serial baud rate; defaults to serial_settings.haptic_dial.baudrate",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"seconds to wait for each response (default: {DEFAULT_TIMEOUT_S})",
    )
    parser.add_argument("--raw", action="store_true", help="print every received serial line")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Parse CLI arguments and run the read-only controller query."""

    parser = build_parser()
    args = parser.parse_args(argv)
    port = resolve_port(args, parser)
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    if args.baud is not None and args.baud <= 0:
        parser.error("--baud must be greater than zero")
    if serial is None:
        print("error: pyserial is not installed in this Python environment", file=sys.stderr)
        return 2

    baudrate = args.baud or require_serial_baudrate(SERIAL_SETTINGS_KEY)
    try:
        return read_controller(
            port, baudrate=baudrate, timeout_s=args.timeout, show_raw=args.raw
        )
    except (serial.SerialException, OSError, TimeoutError, RuntimeError, ValueError) as exc:
        print(f"error reading {port}: {exc}", file=sys.stderr)
        print("Make sure the launcher is stopped and the controller is connected.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
