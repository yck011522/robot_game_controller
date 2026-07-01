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

from core.device_connection import require_serial_baudrate, resolve_serial_ports
import port_registry


_SERIAL_SETTINGS_KEY = "haptic_dial"  # config key that owns this firmware's serial speed
_DISCOVERY_INTERVAL_S = 3.0
_WATCHDOG_TIMEOUT_S = 0.5
_PROBE_TIMEOUT_S = 1.5
_IDENTITY_AUDIT_TIMEOUT_S = 1.5
_DEFAULT_TELEMETRY_INTERVAL_MS = 10
_DEFAULT_BOUNDS_MIN_RAD = [-math.pi] * 6
_DEFAULT_BOUNDS_MAX_RAD = [math.pi] * 6
_PARAM_RETRY_INTERVAL_S = 0.25


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


@dataclass
class _PendingParameterWrite:
    """One runtime parameter write waiting for firmware readback confirmation."""

    target_wire_value: int
    seq: int | None = None
    last_send_mono_s: float = 0.0
    attempts: int = 0


class _SingleDialBoard:
    def __init__(
        self,
        *,
        port: str,
        owner: str,
        baudrate: int,
        serial_factory: Callable[[], Any],
        now_fn: Callable[[], float],
        param_lines: list[tuple[str, int]],
    ) -> None:
        self.port = port
        self.baudrate = baudrate  # baudrate loaded from config/device_ports_and_addr.yaml for this firmware
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
        self._param_responses: dict[int, tuple[str, str]] = {}
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
            ser.baudrate = self.baudrate
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

    def send_param(self, name: str, value: int) -> int | None:
        """Send one S command write."""
        if self._serial is None or self.dial_id is None:
            return None
        seq = self._next_seq()
        self._send_line(f"S,{seq},{name},{int(value)}")
        return seq

    def pop_param_response(self, seq: int) -> tuple[str, str] | None:
        """Return and clear one parsed S response for ``seq`` when present."""

        return self._param_responses.pop(int(seq), None)

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
                return
            if kind == "S" and len(parts) >= 4:
                self._param_responses[int(parts[1])] = (parts[2], parts[3])
        except (TypeError, ValueError):
            return


class RealHaptic:
    def __init__(
        self,
        *,
        team: str,
        profile,
        serial_factory: Callable[[], Any] | None = None,
        now_fn: Callable[[], float] | None = None,
    ) -> None:
        # Profile selects one six-dial team; ids are fixed per team namespace.
        self._team = team
        self._profile = profile
        self._serial_factory = serial_factory or serial.Serial
        self._now = now_fn or time.monotonic
        self._expected_dial_ids = _team_dial_ids(team)
        self._connections: dict[int, _SingleDialBoard] = {}
        self._last_discovery_s = 0.0
        # True after first cmd.haptic.* arrives; before that, tracking target
        # is derived from robot actual so startup remains coherent.
        self._has_runtime_command = False

        port_resolution = resolve_serial_ports(f"haptic_{team}")
        self._baudrate = require_serial_baudrate(_SERIAL_SETTINGS_KEY)
        self._configured_ports = list(port_resolution.ports)
        self._param_lines = _build_param_lines(profile.tuning.get("haptic", {}))
        self._enable_lines = _build_enable_lines(profile.tuning.get("haptic", {}))

        haptic_tuning = profile.tuning.get("haptic", {}) if isinstance(profile.tuning, dict) else {}
        self._gear_ratio = _normalize_gear_ratio(haptic_tuning.get("gear_ratio"))
        self._tracking_target_rad = [0.0] * 6
        self._default_tracking_kp_wire = _fixed_x1000_value(
            haptic_tuning.get("tracking_kp"), 10.0
        )
        self._runtime_param_targets: dict[str, int] = {}
        self._pending_param_writes: dict[int, dict[str, _PendingParameterWrite]] = {}
        self._confirmed_param_values: dict[int, dict[str, int]] = {}
        bounds_min_robot_rad = _default_bounds_rad(haptic_tuning.get("bounds_deg_min"), _DEFAULT_BOUNDS_MIN_RAD)
        bounds_max_robot_rad = _default_bounds_rad(haptic_tuning.get("bounds_deg_max"), _DEFAULT_BOUNDS_MAX_RAD)
        self._bounds_min_rad, self._bounds_max_rad = _robot_bounds_to_dial_bounds_rad(
            bounds_min_robot_rad, bounds_max_robot_rad, self._gear_ratio
        )
        self._latest_robot_actual_rad: list[float] | None = None
        # Pending reseat payload in dial space, applied once per connected id.
        self._pending_reseat_dial_rad: list[float] | None = None
        self._pending_reseat_ids: set[int] = set()

        self._audit_startup_identities_or_raise()

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

    def request_parameter(self, name: str, value: float) -> None:
        """Queue a sparse runtime parameter write for every connected dial.

        ``haptic_io`` calls this for stage-level parameter changes requested by
        the game controller. The real backend handles firmware ``S`` writes,
        readback matching, and retry timing per dial.
        """

        wire_value = _runtime_parameter_wire_value(name, value)
        if wire_value is None:
            return
        self._runtime_param_targets[name] = wire_value
        for dial_id in self._connections:
            self._queue_parameter_write(dial_id, name, wire_value)

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
            self._tick_parameter_writes(board, dial_id)
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
        # Diagnostic-only raw firmware fields (observe; no behavior change).
        # These let a bus trace distinguish a serial-corruption glitch (raw
        # decideg jumps but firmware seq advances normally) from a firmware
        # reboot (seq rolls backward) when chasing spurious dial values.
        dial_pos_decideg: list[int] = []  # raw integer angle straight from the T frame
        dial_seq: list[int] = []          # firmware last-processed C-command sequence
        dial_status_bits: list[int] = []  # firmware status bitfield (fault/mode flags)
        for dial_id in self._expected_dial_ids:
            board = self._connections.get(dial_id)
            telem = board.telemetry if board is not None else _DialTelemetry()
            dial_pos_rad.append(_decideg_to_rad(telem.angle_decideg))
            dial_vel_rad_s.append(_decideg_to_rad(telem.speed_decideg_s))
            board_connected.append(bool(board and board.connected))
            board_loop_hz.append(int(telem.foc_rate_hz))
            dial_pos_decideg.append(int(telem.angle_decideg))
            dial_seq.append(int(telem.last_control_seq))
            dial_status_bits.append(int(telem.status_bits))

        return {
            "dial_pos_rad": dial_pos_rad,
            "dial_vel_rad_s": dial_vel_rad_s,
            "board_connected": board_connected,
            "board_loop_hz": board_loop_hz,
            "dial_pos_decideg": dial_pos_decideg,
            "dial_seq": dial_seq,
            "dial_status_bits": dial_status_bits,
        }

    def close(self) -> None:
        """Close all active board connections."""

        self._restore_default_parameters_before_close()
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
                baudrate=self._baudrate,
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
            for name, wire_value in self._runtime_param_targets.items():
                self._queue_parameter_write(board.dial_id, name, wire_value)
            known_ports.add(port)

    def _candidate_ports(self) -> list[str]:
        """Return configured ports; runtime never scans unrelated COM ports."""

        return list(self._configured_ports)

    def _audit_startup_identities_or_raise(self) -> None:
        """Verify configured team ports expose the expected six dial IDs.

        Audit runs only when exactly six team ports are configured. If exactly
        one expected ID is missing and exactly one board reports a default
        identity (0 or 1), the backend auto-repairs that board's ID silently
        before entering the regular control loop.
        """

        if len(self._configured_ports) != len(self._expected_dial_ids):
            return

        expected_ids = set(self._expected_dial_ids)
        observed = self._probe_identities_on_configured_ports()
        unresponsive = [
            port for port in self._configured_ports if observed.get(port) is None
        ]
        if unresponsive:
            raise RuntimeError(
                f"haptic_{self._team} startup identity audit failed: "
                f"no identity response from {unresponsive}"
            )

        observed_ids = [int(observed[port]) for port in self._configured_ports if observed.get(port) is not None]
        if self._is_identity_set_valid(observed_ids, expected_ids):
            return

        if self._attempt_single_default_id_repair(observed, expected_ids):
            return

        missing_ids = sorted(expected_ids - set(observed_ids))
        unexpected = [
            f"{port}:{dial_id}"
            for port, dial_id in observed.items()
            if dial_id is not None and int(dial_id) not in expected_ids
        ]
        raise RuntimeError(
            f"haptic_{self._team} startup identity audit failed: "
            f"expected={sorted(expected_ids)} observed={observed_ids} "
            f"missing={missing_ids} unexpected={unexpected}"
        )

    def _probe_identities_on_configured_ports(self) -> dict[str, int | None]:
        """Read one dial identity from each configured COM port."""

        out: dict[str, int | None] = {}
        for port in self._configured_ports:
            out[port] = _probe_port_identity(
                port=port,
                baudrate=self._baudrate,
                serial_factory=self._serial_factory,
                now_fn=self._now,
                timeout_s=_IDENTITY_AUDIT_TIMEOUT_S,
            )
        return out

    def _is_identity_set_valid(self, observed_ids: list[int], expected_ids: set[int]) -> bool:
        """Return True when observed IDs exactly match the expected team set."""

        if len(observed_ids) != len(self._expected_dial_ids):
            return False
        if len(set(observed_ids)) != len(observed_ids):
            return False
        return set(observed_ids) == expected_ids

    def _attempt_single_default_id_repair(
        self,
        observed: dict[str, int | None],
        expected_ids: set[int],
    ) -> bool:
        """Try one safe auto-repair for a single missing team identity.

        Safe repair rule:
        - all six configured ports responded,
        - exactly one expected ID is missing,
        - exactly one board reports a default identity (0 or 1).
        """

        if any(value is None for value in observed.values()):
            return False

        observed_ids = [int(observed[port]) for port in self._configured_ports]
        missing_ids = sorted(expected_ids - set(observed_ids))
        if len(missing_ids) != 1:
            return False
        missing_id = int(missing_ids[0])

        default_id_ports = [
            port
            for port, dial_id in observed.items()
            if dial_id is not None and int(dial_id) in (0, 1)
        ]
        if len(default_id_ports) != 1:
            return False

        repair_port = default_id_ports[0]
        print(
            f"[haptic_io.{self._team}] identity audit: repairing {repair_port} "
            f"to missing dial_id={missing_id}",
            flush=True,
        )
        if not _set_port_identity(
            port=repair_port,
            baudrate=self._baudrate,
            target_dial_id=missing_id,
            serial_factory=self._serial_factory,
            now_fn=self._now,
            timeout_s=_IDENTITY_AUDIT_TIMEOUT_S,
        ):
            return False

        recheck = self._probe_identities_on_configured_ports()
        if any(value is None for value in recheck.values()):
            return False
        recheck_ids = [int(recheck[port]) for port in self._configured_ports]
        if not self._is_identity_set_valid(recheck_ids, expected_ids):
            return False
        print(
            f"[haptic_io.{self._team}] identity audit: auto-repair succeeded "
            f"for {repair_port} -> dial_id {missing_id}",
            flush=True,
        )
        return True

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
            if board.dial_id is not None:
                self._confirmed_param_values.pop(board.dial_id, None)
                for name, wire_value in self._runtime_param_targets.items():
                    self._queue_parameter_write(board.dial_id, name, wire_value)

    def _queue_parameter_write(
        self, dial_id: int, name: str, target_wire_value: int
    ) -> None:
        """Mark one board parameter as needing firmware confirmation."""

        confirmed = self._confirmed_param_values.setdefault(dial_id, {})
        if confirmed.get(name) == int(target_wire_value):
            return
        pending = self._pending_param_writes.setdefault(dial_id, {})
        pending[name] = _PendingParameterWrite(target_wire_value=int(target_wire_value))

    def _tick_parameter_writes(
        self, board: _SingleDialBoard, dial_id: int
    ) -> None:
        """Advance pending runtime-parameter writes for one connected board."""

        pending = self._pending_param_writes.setdefault(dial_id, {})
        confirmed = self._confirmed_param_values.setdefault(dial_id, {})
        for name, wire_value in self._runtime_param_targets.items():
            if confirmed.get(name) != wire_value and name not in pending:
                pending[name] = _PendingParameterWrite(target_wire_value=wire_value)

        now = self._now()
        for name, item in list(pending.items()):
            if item.seq is not None:
                response = board.pop_param_response(item.seq)
                if response is not None:
                    response_name, response_value = response
                    if (
                        response_name == name
                        and _response_value_matches(response_value, item.target_wire_value)
                    ):
                        confirmed[name] = item.target_wire_value
                        del pending[name]
                        continue
                    item.seq = None
                    item.last_send_mono_s = 0.0

            if item.seq is not None and (now - item.last_send_mono_s) < _PARAM_RETRY_INTERVAL_S:
                continue
            seq = board.send_param(name, item.target_wire_value)
            if seq is None:
                continue
            item.seq = seq
            item.last_send_mono_s = now
            item.attempts += 1

    def _restore_default_parameters_before_close(self) -> None:
        """Best-effort restore of persistent parameters before serial close."""

        for board in self._connections.values():
            board.send_param("tracking_kp", self._default_tracking_kp_wire)


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


def _robot_bounds_to_dial_bounds_rad(
    bounds_min_robot_rad: list[float],
    bounds_max_robot_rad: list[float],
    gear_ratio: list[float],
) -> tuple[list[float], list[float]]:
    """Convert profile robot-joint bounds to dial-space firmware bounds.

    Called during `RealHaptic` initialization for the pre-command fallback
    bounds. Runtime `cmd.haptic.*` bounds are already dial-space and are
    copied directly by `apply_command`.
    """

    out_min: list[float] = []
    out_max: list[float] = []
    for axis in range(6):
        # Negative gearing reverses endpoint order; sorting keeps firmware
        # control frames valid with min <= max.
        gear = float(gear_ratio[axis]) if axis < len(gear_ratio) else 1.0
        if abs(gear) < 1e-9:
            gear = 1.0
        lo_robot = float(bounds_min_robot_rad[axis])
        hi_robot = float(bounds_max_robot_rad[axis])
        lo_dial = lo_robot / gear
        hi_dial = hi_robot / gear
        out_min.append(min(lo_dial, hi_dial))
        out_max.append(max(lo_dial, hi_dial))
    return out_min, out_max


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
            out.append((key, _fixed_x1000_value(data[key], 0.0)))

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
    """Return force-mode enable writes sent after startup reseat completes."""
    data = node if isinstance(node, dict) else {}
    oob_kick = data.get("oob_kick") if isinstance(data.get("oob_kick"), dict) else {}
    # Protocol default is enabled; profiles may explicitly disable it with
    # tuning.haptic.oob_kick.enabled: false.
    oob_kick_enabled = bool(oob_kick.get("enabled", True))
    return [
        ("enable_tracking", 1),
        ("enable_bounds_restoration", 1),
        ("enable_oob_kick", 1 if oob_kick_enabled else 0),
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


def _fixed_x1000_value(value: Any, default: float) -> int:
    """Convert a float-like config value into firmware x1000 fixed point."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        number = float(default)
    return int(round(number * 1000.0))


def _runtime_parameter_wire_value(name: str, value: float) -> int | None:
    """Return the firmware wire value for supported sparse runtime params."""

    if name != "tracking_kp":
        return None
    return _fixed_x1000_value(value, 10.0)


def _response_value_matches(response_value: str, target_wire_value: int) -> bool:
    """Return True when an S response value confirms the requested target."""

    try:
        return int(response_value) == int(target_wire_value)
    except (TypeError, ValueError):
        return False


def _rad_to_decideg(value: float) -> int:
    return int(round(math.degrees(float(value)) * 10.0))


def _decideg_to_rad(value: int) -> float:
    return math.radians(float(value) / 10.0)


def _probe_port_identity(
    *,
    port: str,
    baudrate: int,
    serial_factory: Callable[[], Any],
    now_fn: Callable[[], float],
    timeout_s: float,
) -> int | None:
    """Open one port and return its current dial_id, or None on timeout/error."""

    try:
        with _opened_serial_for_identity(
            port=port,
            baudrate=baudrate,
            serial_factory=serial_factory,
        ) as ser:
            query_seq = 1
            _serial_write_ascii_line(ser, f"I,{query_seq}")
            return _wait_for_identity_response(
                ser,
                now_fn=now_fn,
                timeout_s=timeout_s,
                expected_query_seq=query_seq,
            )
    except Exception:
        return None


def _set_port_identity(
    *,
    port: str,
    baudrate: int,
    target_dial_id: int,
    serial_factory: Callable[[], Any],
    now_fn: Callable[[], float],
    timeout_s: float,
) -> bool:
    """Set one port's dial_id and verify the persisted value via readback."""

    try:
        with _opened_serial_for_identity(
            port=port,
            baudrate=baudrate,
            serial_factory=serial_factory,
        ) as ser:
            set_seq = 1
            _serial_write_ascii_line(ser, f"I,{set_seq},{int(target_dial_id)}")
            set_value = _wait_for_identity_response(
                ser,
                now_fn=now_fn,
                timeout_s=timeout_s,
                expected_query_seq=set_seq,
            )
            if set_value != int(target_dial_id):
                return False

            verify_seq = 2
            _serial_write_ascii_line(ser, f"I,{verify_seq}")
            verify_value = _wait_for_identity_response(
                ser,
                now_fn=now_fn,
                timeout_s=timeout_s,
                expected_query_seq=verify_seq,
            )
            return verify_value == int(target_dial_id)
    except Exception:
        return False


class _opened_serial_for_identity:
    """Small context manager for short identity query/set transactions."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        serial_factory: Callable[[], Any],
    ) -> None:
        self._port = port
        self._baudrate = int(baudrate)
        self._serial_factory = serial_factory
        self._ser = None

    def __enter__(self):
        ser = self._serial_factory()
        ser.port = self._port
        ser.baudrate = self._baudrate
        ser.timeout = 0.05
        ser.write_timeout = 0.0
        ser.dtr = False
        ser.rts = False
        ser.open()
        ser.reset_input_buffer()
        self._ser = ser
        return ser

    def __exit__(self, exc_type, exc, tb) -> None:
        ser = self._ser
        self._ser = None
        if ser is not None:
            try:
                if getattr(ser, "is_open", False):
                    ser.close()
            except Exception:
                pass


def _serial_write_ascii_line(ser: Any, line: str) -> None:
    """Write one newline-terminated ASCII protocol frame."""

    ser.write((line + "\n").encode("ascii"))


def _wait_for_identity_response(
    ser: Any,
    *,
    now_fn: Callable[[], float],
    timeout_s: float,
    expected_query_seq: int,
) -> int | None:
    """Read serial lines until identity query response or timeout.

    Accepts telemetry (`T`) as a fallback identity source because firmware may
    emit it before the explicit query reply on busy startup links.
    """

    deadline = float(now_fn()) + float(timeout_s)
    buffer = bytearray()
    fallback_telem_id: int | None = None
    expected_seq_text = str(int(expected_query_seq))

    while float(now_fn()) < deadline:
        try:
            waiting = int(getattr(ser, "in_waiting", 0) or 0)
            read_size = waiting if waiting > 0 else 256
            chunk = ser.read(read_size)
        except Exception:
            return fallback_telem_id

        if not chunk:
            time.sleep(0.01)
            continue
        buffer.extend(chunk)

        while True:
            newline = buffer.find(b"\n")
            if newline < 0:
                break
            raw = bytes(buffer[:newline])
            del buffer[: newline + 1]
            line = raw.decode("ascii", errors="ignore").strip().rstrip("\r")
            if not line:
                continue
            parts = line.split(",")
            if parts[0] == "I" and len(parts) >= 3 and parts[1] == expected_seq_text:
                try:
                    return int(parts[2])
                except (TypeError, ValueError):
                    continue
            if parts[0] == "T" and len(parts) >= 2:
                try:
                    fallback_telem_id = int(parts[1])
                except (TypeError, ValueError):
                    continue

    return fallback_telem_id
