"""UDP broadcast publisher for game state.

Runs its own daemon thread at a configurable frequency (default 50 Hz) and
broadcasts the full game state as a JSON datagram to the subnet broadcast
address.  Receivers (Raspberry Pi display nodes) simply bind a UDP socket to
the configured port and process incoming packets — no handshake, no
connection, no per-device addressing required.

Thread safety:
    StatePublisher only calls settings.snapshot() which is lock-protected.
    No mutable state is shared with other threads beyond that read.

Usage:
    publisher = StatePublisher(settings)
    publisher.start()
    ...
    publisher.stop()

Network setup:
    See NETWORK_PROTOCOL.md at the repository root for full details on
    subnet configuration, packet format, and receiver implementation.
"""

import json
import socket
import threading
import time
import sys
from typing import Optional

from game_settings import GameSettings

# ---------------------------------------------------------------------------
# Published motor IDs (Team 1 joints)
# ---------------------------------------------------------------------------
_MOTOR_IDS = list(range(11, 17))
_BUCKET_IDS = [11, 12, 13, 21, 22, 23]

# Protocol version — increment when the payload schema changes so receivers
# can detect incompatible versions and display a warning.
PROTOCOL_VERSION = 1


class StatePublisher:
    """Broadcasts game state as UDP JSON datagrams on its own thread.

    Parameters
    ----------
    settings:
        Shared GameSettings register (read-only from this class).
    broadcast_addr:
        Destination address for UDP broadcasts.  Use the subnet broadcast
        address, e.g. ``"192.168.1.255"``, or ``"255.255.255.255"`` for
        a limited broadcast (works on most LANs without a router).
    port:
        UDP port.  All receivers must ``bind("", port)``.
    publish_hz:
        Publish loop target frequency.  50 Hz is a good default;
        100 Hz is fine on Gigabit Ethernet.
    """

    def __init__(
        self,
        settings: GameSettings,
        broadcast_addr: str = "255.255.255.255",
        port: int = 9000,
        publish_hz: float = 50.0,
    ):
        self._settings = settings
        self._broadcast_addr = broadcast_addr
        self._port = port
        self._publish_hz = publish_hz

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Performance tracking
        self._actual_hz: float = 0.0
        self._tx_count: int = 0
        self._tx_bytes: int = 0
        self._measure_start: float = 0.0

        # Open socket once at construction — safe to reuse across send calls
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Set a send buffer large enough for burst traffic (default is usually fine)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

    # --- Lifecycle ---------------------------------------------------------

    def start(self):
        """Start the publish loop thread."""
        self._stop_event.clear()
        self._measure_start = time.time()
        self._tx_count = 0
        self._thread = threading.Thread(
            target=self._publish_loop, name="state-publisher", daemon=True
        )
        self._thread.start()

    def stop(self):
        """Stop the publish loop and close the socket."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self._sock.close()
        except Exception:
            pass

    # --- Properties --------------------------------------------------------

    @property
    def actual_hz(self) -> float:
        """Observed publish frequency over the last measurement window."""
        return self._actual_hz

    @property
    def broadcast_addr(self) -> str:
        return self._broadcast_addr

    @property
    def port(self) -> int:
        return self._port

    @property
    def publish_hz(self) -> float:
        return self._publish_hz

    @publish_hz.setter
    def publish_hz(self, value: float):
        self._publish_hz = max(1.0, float(value))

    # --- Payload assembly --------------------------------------------------

    def _build_payload(self, snap: dict) -> dict:
        """Assemble the JSON payload from a settings snapshot.

        Keep all keys stable — adding new top-level keys is a backward-
        compatible change.  Removing or renaming keys requires a protocol
        version bump.
        """
        return {
            # --- Protocol metadata ---
            "v": PROTOCOL_VERSION,  # protocol version
            "ts": time.time(),  # Unix timestamp (float, seconds)
            # --- Game stage ---
            "stage": snap.get("current_stage", "Idle"),
            "countdown_s": snap.get("stage_countdown_s", 0),
            "estop": snap.get("emergency_stop", False),
            # --- Joints ---
            # Each key is the motor ID (as string for JSON compatibility).
            # dial_deg : rate-limited clamped dial position (degrees)
            # robot_deg: actual robot joint position (degrees)
            # Extend here with velocity_dps, torque_nm, etc. when available.
            "joints": {
                str(mid): {
                    "dial_deg": snap.get("clamped_deg", {}).get(mid, 0.0),
                    "robot_deg": snap.get("robot_actual_deg", {}).get(mid, 0.0),
                    # Future fields (uncomment when data is available):
                    # "velocity_dps": snap.get("robot_velocity_dps", {}).get(mid, 0.0),
                    # "torque_nm": snap.get("robot_torque_nm", {}).get(mid, 0.0),
                }
                for mid in _MOTOR_IDS
            },
            # --- Scoring ---
            "scores": {
                "team1": snap.get("team1_score", 0.0),
                "team2": snap.get("team2_score", 0.0),
                "high": snap.get("high_score", 0.0),
                "high_holder": snap.get("high_score_holder", ""),
            },
            # --- Per-bucket weights (grams) ---
            # Keys are bucket IDs as strings: "11","12","13" (T1), "21","22","23" (T2)
            "buckets": {
                str(bid): snap.get("bucket_weights", {}).get(bid, 0.0)
                for bid in _BUCKET_IDS
            },
            # --- Multipliers (static config — useful for display rendering) ---
            "multipliers": {
                str(bid): snap.get("bucket_multipliers", {}).get(bid, 1.0)
                for bid in _BUCKET_IDS
            },
            # --- System health (optional — receivers may ignore) ---
            "health": {
                "game_loop_hz": snap.get("game_loop_hz", 0.0),
                "robot_physics_hz": snap.get("robot_physics_hz", 0.0),
                "haptic_connected": snap.get("haptic_connected_count", "0/6"),
                "weight_sensors": snap.get("weight_sensor_connected_count", "0/6"),
                "publisher_hz": self._actual_hz,
            },
        }

    # --- Publish loop ------------------------------------------------------

    def _publish_loop(self):
        dt_target = 1.0 / self._publish_hz
        last_time = time.time()

        while not self._stop_event.is_set():
            now = time.time()
            last_time = now

            # Build and send
            try:
                snap = self._settings.snapshot()
                payload = self._build_payload(snap)
                data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self._sock.sendto(data, (self._broadcast_addr, self._port))
                self._tx_count += 1
                self._tx_bytes += len(data)
            except Exception:
                pass  # Network errors are non-fatal; next packet will be sent

            # Measure actual Hz every 0.5 s
            elapsed = time.time() - self._measure_start
            if elapsed >= 0.5:
                self._actual_hz = self._tx_count / elapsed
                self._tx_count = 0
                self._tx_bytes = 0
                self._measure_start = time.time()
                # Write back to settings for UI display
                self._settings.set("publisher_hz", self._actual_hz)

            # Hybrid sleep: bulk sleep + spin-wait for precision.
            # Re-read publish_hz each iteration so runtime changes take effect.
            dt_target = 1.0 / self._publish_hz
            deadline = last_time + dt_target
            remaining = deadline - time.time()
            if remaining > 0.0015:
                self._stop_event.wait(remaining - 0.0015)
            while time.time() < deadline:
                if self._stop_event.is_set():
                    return
                time.sleep(0)  # yield GIL
