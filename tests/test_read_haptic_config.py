from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.read_haptic_config import (  # noqa: E402
    HapticConfigReader,
    ParameterDefinition,
    format_parameter,
    format_status,
)


class FakeSerial:
    """Minimal serial transport that returns telemetry before each query response."""

    def __init__(self) -> None:
        self.pending: list[bytes] = []  # response lines waiting for the reader
        self.writes: list[str] = []  # host commands captured without newline terminators

    def write(self, payload: bytes) -> int:
        """Capture one command and enqueue the corresponding firmware response."""

        command = payload.decode("ascii").strip()
        self.writes.append(command)
        kind, seq, *fields = command.split(",")
        self.pending.append(b"T,21,42,100,0,0,1000,27\n")
        if kind == "V":
            response = f"V,{seq},0.3.0\n"
        elif kind == "I":
            response = f"I,{seq},21\n"
        else:
            response = f"S,{seq},{fields[0]},10000\n"
        self.pending.append(response.encode("ascii"))
        return len(payload)

    def readline(self) -> bytes:
        """Return the next queued firmware line."""

        return self.pending.pop(0) if self.pending else b""


class HapticConfigReaderTests(unittest.TestCase):
    """Verify query framing, telemetry filtering, and value presentation."""

    def test_reader_filters_telemetry_and_queries_without_writes(self) -> None:
        """Queries must omit values and tolerate telemetry before responses."""

        fake = FakeSerial()
        reader = HapticConfigReader(fake, timeout_s=0.1)

        self.assertEqual(reader.query_version(), "0.3.0")
        self.assertEqual(reader.query_identity(), 21)
        self.assertEqual(reader.query_parameter("tracking_kp"), "10000")
        self.assertEqual(fake.writes, ["V,1", "I,2", "S,3,tracking_kp"])
        self.assertTrue(format_status(reader.latest_telemetry).startswith("27 (tracking=ON"))

    def test_parameter_formatting(self) -> None:
        """Fixed-point, Boolean, and unsupported values should be unambiguous."""

        gain = ParameterDefinition("tracking_kp", "", 1000.0)
        enabled = ParameterDefinition("enable_tracking", "bool")

        self.assertEqual(format_parameter(gain, "10000"), "10000 (10)")
        self.assertEqual(format_parameter(enabled, "0"), "0 (OFF)")
        self.assertEqual(format_parameter(gain, "?"), "? (unsupported by firmware)")


if __name__ == "__main__":
    unittest.main()
