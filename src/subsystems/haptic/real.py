"""Single-dial ESP32 haptic runtime for P5.

Discovers one board per dial, keyed by persistent dial_id, then:
1. seeds the dial's current position from robot actual pose via `R`
2. sends high-rate tracking target + soft bounds via `C`
3. republishes the latest dial telemetry in the existing `telem.haptic.*` shape

The implementation stays deliberately small and process-local. `haptic_io`
already runs at the control rate, so we keep serial I/O on that loop and use
latest-wins semantics rather than spawning additional worker threads.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable

import serial
import serial.tools.list_ports

import port_registry


_CH340_VID_PIDS = {(0x1A86, 0x7522), (0x1A86, 0x7523)}
_BAUDRATE = 115200
_DISCOVERY_INTERVAL_S = 3.0
_WATCHDOG_TIMEOUT_S = 0.5
_PROBE_TIMEOUT_S = 1.5
_DEFAULT_TELEMETRY_INTERVAL_MS = 10
_DEFAULT_BOUNDS_MIN_RAD = [-math.pi] * 6
_DEFAULT_BOUNDS_MAX_RAD = [math.pi] * 6


@dataclass
class _DialTelemetry:
    angle_decideg: int = 0
    speed_decideg_s: int = 0
    torque_ma: int = 0
    foc_rate_hz: int = 0
    status_bits: int = 0
    last_control_seq: int = 0
    last_telem_mono_s: float = 0.0
    fw_version: str | None = None


class _SingleDialBoard:
    def __init__(
        self,
        *,
        port: str,
        owner: str,
        serial_factory: Callable[[], Any],
        now_fn: Callable[[], float],
        param_lines: list[tuple[str, int]],
    ) -> None:
        self.port = port
        self.dial_id: int | None = None
        self.telemetry = _DialTelemetry()
        # True after host sends an R command to align dial coordinates with
        # the latest robot pose for this connection lifecycle.
        self.startup_synced = False

        self._owner = owner
        self._serial_factory = serial_factory
        self._now = now_fn
        self._param_lines = list(param_lines)
        self._serial = None
        self._buffer = bytearray()
        self._seq = 0
        self._claimed_port = False
        # Tracks telemetry's last processed-C seq so reconnect/reboot can be
        # inferred from backward jumps.
        self._last_seen_control_seq: int | None = None
        # Modes are intentionally armed only after digital reseat completes.
        self.control_armed = False

    @property
    def connected(self) -> bool:
        if self._serial is None or not getattr(self._serial, "is_open", False):
            return False
        if self.telemetry.last_telem_mono_s <= 0.0:
            return False
        return (self._now() - self.telemetry.last_telem_mono_s) <= _WATCHDOG_TIMEOUT_S

    def connect(self, *, expected_ids: set[int]) -> bool:
        """Open serial, probe dial identity, and push connect-time params."""
        if not port_registry.acquire_port(self.port, owner=self._owner):
            return False
        self._claimed_port = True
        try:
            ser = self._serial_factory()
            ser.port = self.port
            ser.baudrate = _BAUDRATE
            ser.timeout = 0.05
            ser.write_timeout = 0.0
            ser.dtr = False
            ser.rts = False
            ser.open()
            ser.reset_input_buffer()
            self._serial = ser

            self._send_line(f"V,{self._next_seq()}")
            self._send_line(f"I,{self._next_seq()}")

            deadline = self._now() + _PROBE_TIMEOUT_S
            while self._now() < deadline:
                self.poll(block=True)
                if self.dial_id in expected_ids:
                    break
                time.sleep(0.01)

            if self.dial_id not in expected_ids:
                raise RuntimeError(f"port {self.port} did not report an expected dial id")

            ser.timeout = 0.0
            for name, value in self._param_lines:
                self._send_line(f"S,{self._next_seq()},{name},{value}")
            return True
        except Exception:
            self.close()
            return False

    def close(self) -> None:
        """Close serial and release shared port claim."""
        ser = self._serial
        self._serial = None
        if ser is not None:
            try:
                if getattr(ser, "is_open", False):
                    ser.close()
            except Exception:
                pass
        self._buffer.clear()
        self._last_seen_control_seq = None
        self.control_armed = False
        if self._claimed_port:
            port_registry.release_port(self.port)
            self._claimed_port = False

    def poll(self, *, block: bool = False) -> None:
        """Read and parse serial lines into telemetry fields."""
        ser = self._serial
        if ser is None or not getattr(ser, "is_open", False):
            return
        while True:
            try:
                waiting = int(getattr(ser, "in_waiting", 0) or 0)
                if waiting <= 0:
                    if not block:
                        break
                    chunk = ser.read(256)
                else:
                    chunk = ser.read(waiting)
                if not chunk:
                    break
                self._buffer.extend(chunk)
                self._drain_lines()
                if not block:
                    continue
            except (serial.SerialException, OSError):
                self.close()
                return
            break

    def send_control(self, *, target_rad: float, bounds_min_rad: float, bounds_max_rad: float) -> None:
        """Send C command (dial-space target and bounds)."""
        if self._serial is None or self.dial_id is None:
            return
        self._send_line(
            f"C,{self._next_seq()},{_rad_to_decideg(target_rad)},{_rad_to_decideg(bounds_min_rad)},{_rad_to_decideg(bounds_max_rad)}"
        )

    def send_set_current_position(self, current_rad: float) -> None:
        """Send R command to rebase logical dial angle without physical jump."""
        if self._serial is None or self.dial_id is None:
            return
        self._send_line(f"R,{self._next_seq()},{_rad_to_decideg(current_rad)}")

    def send_param(self, name: str, value: int) -> None:
        """Send one S command write."""
        if self._serial is None or self.dial_id is None:
            return
        self._send_line(f"S,{self._next_seq()},{name},{int(value)}")

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _send_line(self, line: str) -> None:
        ser = self._serial
        if ser is None or not getattr(ser, "is_open", False):
            return
        try:
            ser.write((line + "\n").encode("ascii"))
        except (serial.SerialException, OSError):
            self.close()

    def _drain_lines(self) -> None:
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                return
            raw = bytes(self._buffer[:newline])
            del self._buffer[:newline + 1]
            line = raw.decode("ascii", errors="ignore").strip().rstrip("\r")
            if not line:
                continue
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        parts = line.split(",")
        kind = parts[0]
        try:
            if kind == "T" and len(parts) == 8:
                self.dial_id = int(parts[1])
                self.telemetry.last_control_seq = int(parts[2])
                self.telemetry.angle_decideg = int(parts[3])
                self.telemetry.speed_decideg_s = int(parts[4])
                self.telemetry.torque_ma = int(parts[5])
                self.telemetry.foc_rate_hz = int(parts[6])
                self.telemetry.status_bits = int(parts[7])
                self.telemetry.last_telem_mono_s = self._now()
                return
            if kind == "I" and len(parts) == 3:
                self.dial_id = int(parts[2])
                return
            if kind == "V" and len(parts) >= 3:
                self.telemetry.fw_version = ",".join(parts[2:])
        except (TypeError, ValueError):
            return


class RealHaptic:
    def __init__(
        self,
        *,
        team: str,
        profile,
        serial_factory: Callable[[], Any] | None = None,
        list_ports_fn: Callable[[], list[Any]] | None = None,
        now_fn: Callable[[], float] | None = None,
    ) -> None:
        # Profile selects one six-dial team; ids are fixed per team namespace.
        self._team = team
        self._profile = profile
        self._serial_factory = serial_factory or serial.Serial
        self._list_ports_fn = list_ports_fn or serial.tools.list_ports.comports
        self._now = now_fn or time.monotonic
        self._expected_dial_ids = _team_dial_ids(team)
        self._connections: dict[int, _SingleDialBoard] = {}
        self._last_discovery_s = 0.0
        # True after first cmd.haptic.* arrives; before that, tracking target
        # is derived from robot actual so startup remains coherent.
        self._has_runtime_command = False

        serial_ports = profile.hardware.get("serial_ports", {}) if isinstance(profile.hardware, dict) else {}
        configured_ports = serial_ports.get(f"haptic_{team}") if isinstance(serial_ports, dict) else None
        self._configured_ports = [str(v).strip() for v in configured_ports] if isinstance(configured_ports, list) else []
        self._param_lines = _build_param_lines(profile.tuning.get("haptic", {}))
        self._enable_lines = _build_enable_lines(profile.tuning.get("haptic", {}))

        haptic_tuning = profile.tuning.get("haptic", {}) if isinstance(profile.tuning, dict) else {}
        self._gear_ratio = _normalize_gear_ratio(haptic_tuning.get("gear_ratio"))
        self._tracking_target_rad = [0.0] * 6
        self._bounds_min_rad = _default_bounds_rad(haptic_tuning.get("bounds_deg_min"), _DEFAULT_BOUNDS_MIN_RAD)
        self._bounds_max_rad = _default_bounds_rad(haptic_tuning.get("bounds_deg_max"), _DEFAULT_BOUNDS_MAX_RAD)
        self._latest_robot_actual_rad: list[float] | None = None
        # Pending reseat payload in dial space, applied once per connected id.
        self._pending_reseat_dial_rad: list[float] | None = None
        self._pending_reseat_ids: set[int] = set()

    def apply_command(self, body: dict[str, Any]) -> None:
        """Apply high-rate cmd.haptic payload from game_controller."""
        targets = body.get("tracking_target_rad") if isinstance(body, dict) else None
        if isinstance(targets, list) and len(targets) >= 6:
            self._tracking_target_rad = [float(v) for v in targets[:6]]
            self._has_runtime_command = True
        lower = body.get("bounds_min_rad") if isinstance(body, dict) else None
        upper = body.get("bounds_max_rad") if isinstance(body, dict) else None
        if isinstance(lower, list) and len(lower) >= 6:
            self._bounds_min_rad = [float(v) for v in lower[:6]]
        if isinstance(upper, list) and len(upper) >= 6:
            self._bounds_max_rad = [float(v) for v in upper[:6]]

    def update_robot_actual(self, q_rad: list[float]) -> None:
        """Receive robot actual pose; used for startup fallback tracking."""
        if len(q_rad) < 6:
            return
        self._latest_robot_actual_rad = [float(v) for v in q_rad[:6]]
        if not self._has_runtime_command:
            self._tracking_target_rad = [
                self._robot_to_dial(idx, q)
                for idx, q in enumerate(self._latest_robot_actual_rad)
            ]

    def request_reseat(self, q_rad: list[float]) -> None:
        """Queue reseat from robot-space radians (legacy path)."""
        if len(q_rad) < 6:
            return
        self._pending_reseat_dial_rad = [
            self._robot_to_dial(idx, float(v))
            for idx, v in enumerate(q_rad[:6])
        ]
        self._pending_reseat_ids = set(self._connections.keys())

    def request_reseat_dial(self, q_dial_rad: list[float]) -> None:
        """Queue reseat from dial-space radians (preferred path)."""
        if len(q_dial_rad) < 6:
            return
        self._pending_reseat_dial_rad = [float(v) for v in q_dial_rad[:6]]
        self._pending_reseat_ids = set(self._connections.keys())

    def sample(self) -> dict[str, Any]:
        """Main control/telemetry step called once per haptic_io tick."""
        self._refresh_connections()
        for board in self._connections.values():
            board.poll()

        for idx, dial_id in enumerate(self._expected_dial_ids):
            board = self._connections.get(dial_id)
            if board is None:
                continue
            self._handle_sequence_reset(board)
            if self._latest_robot_actual_rad is not None and not board.startup_synced:
                seed_dial_rad = self._robot_to_dial(idx, self._latest_robot_actual_rad[idx])
                board.send_set_current_position(seed_dial_rad)
                # Keep C target aligned with the digital reseat to avoid a startup yank.
                self._tracking_target_rad[idx] = seed_dial_rad
                board.startup_synced = True
            if self._pending_reseat_dial_rad is not None and dial_id in self._pending_reseat_ids:
                reseat_dial_rad = float(self._pending_reseat_dial_rad[idx])
                board.send_set_current_position(reseat_dial_rad)
                self._tracking_target_rad[idx] = reseat_dial_rad
                self._pending_reseat_ids.discard(dial_id)
            board.send_control(
                target_rad=self._tracking_target_rad[idx],
                bounds_min_rad=self._bounds_min_rad[idx],
                bounds_max_rad=self._bounds_max_rad[idx],
            )
            # Arm force-producing modes only after a reseat + aligned control
            # command has been sent on this connection lifecycle.
            if board.startup_synced and not board.control_armed:
                for name, value in self._enable_lines:
                    board.send_param(name, value)
                board.control_armed = True

        dial_pos_rad: list[float] = []
        dial_vel_rad_s: list[float] = []
        board_connected: list[bool] = []
        board_loop_hz: list[int] = []
        for dial_id in self._expected_dial_ids:
            board = self._connections.get(dial_id)
            telem = board.telemetry if board is not None else _DialTelemetry()
            dial_pos_rad.append(_decideg_to_rad(telem.angle_decideg))
            dial_vel_rad_s.append(_decideg_to_rad(telem.speed_decideg_s))
            board_connected.append(bool(board and board.connected))
            board_loop_hz.append(int(telem.foc_rate_hz))

        return {
            "dial_pos_rad": dial_pos_rad,
            "dial_vel_rad_s": dial_vel_rad_s,
            "board_connected": board_connected,
            "board_loop_hz": board_loop_hz,
        }

    def close(self) -> None:
        """Close all active board connections."""
        for board in self._connections.values():
            board.close()
        self._connections.clear()

    def _refresh_connections(self) -> None:
        """Drop stale boards and periodically discover/reconnect missing ones."""
        stale = [dial_id for dial_id, board in self._connections.items() if board._serial is None or not board.connected]
        for dial_id in stale:
            board = self._connections.pop(dial_id)
            board.close()

        if len(self._connections) >= len(self._expected_dial_ids):
            return

        now = self._now()
        if (now - self._last_discovery_s) < _DISCOVERY_INTERVAL_S:
            return
        self._last_discovery_s = now

        known_ports = {board.port for board in self._connections.values()}
        expected_ids = set(self._expected_dial_ids)
        for port in self._candidate_ports():
            if port in known_ports:
                continue
            board = _SingleDialBoard(
                port=port,
                owner=f"haptic_io.{self._team}",
                serial_factory=self._serial_factory,
                now_fn=self._now,
                param_lines=self._param_lines,
            )
            if not board.connect(expected_ids=expected_ids):
                continue
            if board.dial_id is None or board.dial_id not in expected_ids:
                board.close()
                continue
            if board.dial_id in self._connections:
                board.close()
                continue
            self._connections[board.dial_id] = board
            known_ports.add(port)

    def _candidate_ports(self) -> list[str]:
        """Return configured ports, else auto-discovered CH340 ports."""
        if self._configured_ports:
            return list(self._configured_ports)
        ports: list[str] = []
        for info in self._list_ports_fn():
            vid = getattr(info, "vid", None)
            pid = getattr(info, "pid", None)
            device = getattr(info, "device", None)
            if (vid, pid) not in _CH340_VID_PIDS or not isinstance(device, str) or not device:
                continue
            if port_registry.is_port_claimed(device):
                continue
            ports.append(device)
        return ports

    def _robot_to_dial(self, idx: int, robot_rad: float) -> float:
        """Convert robot joint radians to dial radians via per-axis gear ratio."""
        ratio = float(self._gear_ratio[idx]) if idx < len(self._gear_ratio) else 1.0
        if abs(ratio) < 1e-9:
            ratio = 1.0
        return float(robot_rad) / ratio

    def _handle_sequence_reset(self, board: _SingleDialBoard) -> None:
        """Detect firmware restart by control-seq rollback and re-arm startup sync."""
        seq_now = int(board.telemetry.last_control_seq)
        seq_prev = board._last_seen_control_seq
        board._last_seen_control_seq = seq_now
        if seq_prev is None:
            return
        # Firmware can restart while USB stays enumerated; if the reported
        # processed-C sequence jumps backwards, force a fresh digital reseat.
        if seq_now < seq_prev and (seq_prev - seq_now) > 10:
            board.startup_synced = False
            board.control_armed = False


def _team_dial_ids(team: str) -> list[int]:
    if team == "a":
        return [11, 12, 13, 14, 15, 16]
    if team == "b":
        return [21, 22, 23, 24, 25, 26]
    raise ValueError(f"unsupported haptic team {team!r}")


def _default_bounds_rad(value: Any, fallback: list[float]) -> list[float]:
    if not isinstance(value, list):
        return list(fallback)
    out: list[float] = []
    for idx, item in enumerate(value[:6]):
        try:
            out.append(math.radians(float(item)))
        except (TypeError, ValueError):
            out.append(float(fallback[idx]))
    if len(out) < 6:
        out.extend(float(v) for v in fallback[len(out):6])
    return out[:6]


def _build_param_lines(node: Any) -> list[tuple[str, int]]:
    data = node if isinstance(node, dict) else {}
    out: list[tuple[str, int]] = []
    fixed_x1000 = (
        "tracking_kp",
        "tracking_kd",
        "tracking_max_torque",
        "bounds_kp",
        "bounds_max_torque",
        "detent_kp",
        "detent_distance",
        "detent_max_torque",
        "vibration_amplitude",
        "oob_kick_amplitude",
    )
    for key in fixed_x1000:
        if key in data:
            out.append((key, int(round(float(data[key]) * 1000.0))))

    integer_fields = (
        "vibration_pulse_interval_ms",
        "oob_kick_pulse_interval_ms",
        "telemetry_interval_ms",
    )
    for key in integer_fields:
        if key in data:
            wire_key = "telemetry_interval" if key == "telemetry_interval_ms" else key
            out.append((wire_key, int(round(float(data[key])))))

    oob_kick = data.get("oob_kick") if isinstance(data.get("oob_kick"), dict) else {}
    if "amplitude" in oob_kick:
        out.append(("oob_kick_amplitude", int(round(float(oob_kick["amplitude"]) * 1000.0))))
    if "pulse_interval_ms" in oob_kick:
        out.append(("oob_kick_pulse_interval_ms", int(round(float(oob_kick["pulse_interval_ms"])))))

    # Safety-first connect behavior: hold force-producing modes disabled
    # until the host has reseated dial coordinates to robot actual.
    out.append(("enable_tracking", 0))
    out.append(("enable_bounds_restoration", 0))
    out.append(("enable_oob_kick", 0))

    if not any(name == "telemetry_interval" for name, _ in out):
        out.append(("telemetry_interval", _DEFAULT_TELEMETRY_INTERVAL_MS))
    return out


def _build_enable_lines(node: Any) -> list[tuple[str, int]]:
    data = node if isinstance(node, dict) else {}
    oob_kick = data.get("oob_kick") if isinstance(data.get("oob_kick"), dict) else {}
    return [
        ("enable_tracking", 1),
        ("enable_bounds_restoration", 1),
        ("enable_oob_kick", 1 if bool(oob_kick.get("enabled", False)) else 0),
    ]


def _normalize_gear_ratio(value: Any) -> list[float]:
    if not isinstance(value, list):
        return [1.0] * 6
    out: list[float] = []
    for item in value[:6]:
        try:
            ratio = float(item)
        except (TypeError, ValueError):
            ratio = 1.0
        if abs(ratio) < 1e-9:
            ratio = 1.0
        out.append(ratio)
    while len(out) < 6:
        out.append(1.0)
    return out[:6]


def _rad_to_decideg(value: float) -> int:
    return int(round(math.degrees(float(value)) * 10.0))


def _decideg_to_rad(value: int) -> float:
    return math.radians(float(value) / 10.0)