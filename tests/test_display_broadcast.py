"""Tests for the UDP display broadcaster wire format and config loader.

Run (pytest is not installed in this env; use unittest):

    $env:PYTHONPATH = "src"
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m unittest \
        tests.test_display_broadcast -v
"""

from __future__ import annotations

import socket
import sys
import unittest
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from core.device_connection import (  # noqa: E402
    load_display_broadcast,
    resolve_display_players,
)
from core.display_protocol import decode_datagram, encode_datagram  # noqa: E402


class DatagramFormatTests(unittest.TestCase):
    """Round-trip and robustness of the datagram envelope."""

    def test_round_trip_preserves_state_and_header(self) -> None:
        state = {"active_stage": "play", "teams": {"a": {"score": 7}}}
        raw = encode_datagram(state, seq=42, ts_wall_ns=123)
        msg = decode_datagram(raw)
        self.assertIsNotNone(msg)
        assert msg is not None
        self.assertEqual(msg["v"], 1)
        self.assertEqual(msg["seq"], 42)
        self.assertEqual(msg["ts_wall_ns"], 123)
        self.assertEqual(msg["state"], state)

    def test_decode_rejects_garbage(self) -> None:
        self.assertIsNone(decode_datagram(b"\xff\xff not json"))
        self.assertIsNone(decode_datagram(b"{}"))  # missing state/version

    def test_decode_rejects_wrong_version(self) -> None:
        bad = b'{"v":999,"seq":1,"ts_wall_ns":0,"state":{}}'
        self.assertIsNone(decode_datagram(bad))


class UdpLoopbackTests(unittest.TestCase):
    """Send one datagram over the loopback and decode it on the receiver."""

    def test_send_and_receive_localhost(self) -> None:
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rx.bind(("127.0.0.1", 0))  # ephemeral port
        port = rx.getsockname()[1]
        rx.settimeout(1.0)

        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            state = {"active_stage": "tutorial", "teams": {"a": {}}}
            tx.sendto(encode_datagram(state, seq=1), ("127.0.0.1", port))
            raw, _ = rx.recvfrom(1 << 16)
        finally:
            tx.close()
            rx.close()
        msg = decode_datagram(raw)
        self.assertIsNotNone(msg)
        assert msg is not None
        self.assertEqual(msg["state"]["active_stage"], "tutorial")


class ConfigLoaderTests(unittest.TestCase):
    """The display_broadcast block parses and maps hostnames to players."""

    def test_load_endpoint(self) -> None:
        db = load_display_broadcast()
        self.assertTrue(db.dest)
        self.assertGreater(db.port, 0)
        self.assertIn("rpi5-11", db.hosts)
        self.assertEqual(db.hosts["rpi5-11"], ("a1", "a2"))

    def test_resolve_players_case_insensitive(self) -> None:
        self.assertEqual(resolve_display_players("RPI5-11"), ("a1", "a2"))
        self.assertIsNone(resolve_display_players("not-a-pi"))


if __name__ == "__main__":
    unittest.main()
