"""Communication layer for ESP32 haptic controllers.

Each ESP32 board drives two FOC motors and communicates over USB serial.
This module handles:
  - Automatic discovery of controllers by USB VID/PID
  - Continuous telemetry reading (angle, speed, torque per motor)
  - Periodic control command sending at 50 Hz
  - Auto-reconnection on disconnect

The upper-level application interacts through a register model:
  - Write targets via set_control() — the internal writer thread sends them at 50 Hz
  - Read telemetry via get_telemetry() — returns frozen snapshots updated by internal reader threads

Usage:
    system = HapticSystem(expected_motor_ids=[11, 12, 13, 14, 15, 16])
    system.start()

    # Wait for all motors to appear
    while not system.all_connected:
        time.sleep(0.1)

    # Read motor positions
    telem = system.get_telemetry(11)
    print(telem.angle)

    # Set control targets (in decidegrees)
    system.set_control(11, position=1800, min_bound=-3600, max_bound=3600)

    # Shutdown
    system.stop()
"""

import time
import threading
import logging
from dataclasses import dataclass
from typing import Optional

import serial
import serial.tools.list_ports

import port_registry

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# CH340 USB VID/PID pairs for auto-detection
_CH340_VID_PIDS = {(0x1A86, 0x7522), (0x1A86, 0x7523)}

_BAUDRATE = 230400
_HAPTIC_UPDATE_HZ = 50
_DISCOVERY_INTERVAL_S = 3.0  # seconds between discovery scans
_WATCHDOG_TIMEOUT_S = 0.5  # no telemetry for this long = disconnected
_PROBE_TIMEOUT_S = 1.5  # timeout when probing a new port
_FULL_RANGE_DECIDEG = 1_080_000  # ±30 rotations, used as default "no bounds"


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TelemetryFrame:
    """Immutable snapshot of one motor's state at a point in time.

    Passed across thread boundaries. The upper application receives these
    from HapticSystem.get_telemetry() and get_all_telemetry().
    """

    motor_id: int
    angle: int  # decidegrees
    speed: int  # decidegrees / s
    torque: int  # milliamps
    timestamp: float  # time.time() when the frame was received


@dataclass
class _MotorControl:
    """Mutable write-register for one motor (internal)."""

    position: int = 0  # decidegrees
    min_bound: Optional[int] = None  # decidegrees, None = unset
    max_bound: Optional[int] = None  # decidegrees, None = unset


# ---------------------------------------------------------------------------
# _BoardConnection — one per ESP32 board (internal)
# ---------------------------------------------------------------------------


class _BoardConnection:
    """Manages serial I/O with one ESP32 board (2 motors).

    Owns two threads:
      - Reader: blocking readline(), parses T frames, updates telemetry registers
      - Writer: 50 Hz loop, sends C commands from control registers + queued one-off cmds

    Not used directly by the upper application.
    """

    def __init__(self, port: str, motor_id_0: int, motor_id_1: int):
        self.port = port
        self.motor_id_0 = motor_id_0
        self.motor_id_1 = motor_id_1

        self._serial: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._writer_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # --- Read registers (written by reader thread, read by anyone) ---
        self._telemetry_lock = threading.Lock()
        self._telem_0: Optional[TelemetryFrame] = None
        self._telem_1: Optional[TelemetryFrame] = None
        self._last_telem_time: float = 0.0
        self._foc_rate: int = 0
        self._last_seq_ack: int = 0

        # --- Write registers (written by upper app, read by writer thread) ---
        self._control_lock = threading.Lock()
        self._control_0 = _MotorControl()
        self._control_1 = _MotorControl()
        self._control_active = False  # True after first set_control() call

        # Sequence counter for C commands
        self._seq_counter = 0
        self._seq_lock = threading.Lock()

        # Queue for one-off commands (S, I, V, E) sent by the writer thread
        self._cmd_queue: list[str] = []
        self._cmd_queue_lock = threading.Lock()

    # --- Properties --------------------------------------------------------

    @property
    def motor_ids(self) -> tuple[int, int]:
        return (self.motor_id_0, self.motor_id_1)

    @property
    def is_connected(self) -> bool:
        if self._stop_event.is_set():
            return False
        if self._serial is None or not self._serial.is_open:
            return False
        with self._telemetry_lock:
            if self._last_telem_time == 0.0:
                return False
            return (time.time() - self._last_telem_time) < _WATCHDOG_TIMEOUT_S

    # --- Lifecycle ---------------------------------------------------------

    def connect(self):
        """Open serial port and start reader/writer threads."""
        self._stop_event.clear()
        # Open without toggling DTR/RTS to avoid resetting the ESP32
        self._serial = serial.Serial()
        self._serial.port = self.port
        self._serial.baudrate = _BAUDRATE
        self._serial.timeout = 0.1
        self._serial.dtr = False
        self._serial.rts = False
        self._serial.open()
        self._serial.reset_input_buffer()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name=f"haptic-reader-{self.port}",
            daemon=True,
        )
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name=f"haptic-writer-{self.port}",
            daemon=True,
        )
        self._reader_thread.start()
        self._writer_thread.start()
        logger.info(
            "Connected to %s (motors %d, %d)",
            self.port,
            self.motor_id_0,
            self.motor_id_1,
        )

    def disconnect(self):
        """Signal threads to stop, wait for them, close port."""
        self._stop_event.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        if self._writer_thread and self._writer_thread.is_alive():
            self._writer_thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        logger.info(
            "Disconnected from %s (motors %d, %d)",
            self.port,
            self.motor_id_0,
            self.motor_id_1,
        )

    # --- Read interface (telemetry) ----------------------------------------

    def get_telemetry(self, motor_id: int) -> Optional[TelemetryFrame]:
        """Return the latest TelemetryFrame for *motor_id*, or None."""
        with self._telemetry_lock:
            if motor_id == self.motor_id_0:
                return self._telem_0
            elif motor_id == self.motor_id_1:
                return self._telem_1
        return None

    # --- Write interface (control) -----------------------------------------

    def set_control(
        self,
        motor_id: int,
        position: int,
        min_bound: Optional[int] = None,
        max_bound: Optional[int] = None,
    ):
        """Update the control register for *motor_id*."""
        with self._control_lock:
            if motor_id == self.motor_id_0:
                ctrl = self._control_0
            elif motor_id == self.motor_id_1:
                ctrl = self._control_1
            else:
                return
            ctrl.position = position
            ctrl.min_bound = min_bound
            ctrl.max_bound = max_bound
            self._control_active = True

    def queue_command(self, cmd: str):
        """Enqueue a raw command line to be sent by the writer thread."""
        with self._cmd_queue_lock:
            self._cmd_queue.append(cmd)

    # --- Internals ---------------------------------------------------------

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq_counter += 1
            return self._seq_counter

    def _send_raw(self, line: str):
        """Write one line to the serial port."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.write((line + "\n").encode("ascii"))
            except (serial.SerialException, OSError) as e:
                logger.warning("Write error on %s: %s", self.port, e)
                self._stop_event.set()

    # --- Reader thread -----------------------------------------------------

    def _reader_loop(self):
        logger.debug("Reader started for %s", self.port)
        # Discard the first line — it may be a partial frame from mid-transmission
        try:
            if self._serial and self._serial.is_open:
                self._serial.readline()
        except (serial.SerialException, OSError):
            pass
        # Now read lines until stopped or error occurs
        while not self._stop_event.is_set():
            try:
                if not self._serial or not self._serial.is_open:
                    break
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                self._parse_line(line)
            except (serial.SerialException, OSError) as e:
                logger.warning("Read error on %s: %s", self.port, e)
                self._stop_event.set()
                break
            except Exception as e:
                logger.debug("Parse error on %s: %s", self.port, e)
        logger.debug("Reader stopped for %s", self.port)

    def _parse_line(self, line: str):
        if line.startswith("T,"):
            self._parse_telemetry(line)
        # V, I, S, E responses could be handled here in the future

    def _parse_telemetry(self, line: str):
        """Parse: T,<id0>,<id1>,<seq>,<ang0>,<ang1>,<spd0>,<spd1>,<tor0>,<tor1>,<foc>"""
        try:
            parts = line.split(",")
            if len(parts) != 11:
                return
            now = time.time()
            # parts[0] == "T"
            id0 = int(parts[1])
            id1 = int(parts[2])
            seq = int(parts[3])
            ang0 = int(parts[4])
            ang1 = int(parts[5])
            spd0 = int(parts[6])
            spd1 = int(parts[7])
            tor0 = int(parts[8])
            tor1 = int(parts[9])
            foc = int(parts[10])

            frame_0 = TelemetryFrame(
                motor_id=id0, angle=ang0, speed=spd0, torque=tor0, timestamp=now
            )
            frame_1 = TelemetryFrame(
                motor_id=id1, angle=ang1, speed=spd1, torque=tor1, timestamp=now
            )

            with self._telemetry_lock:
                self._telem_0 = frame_0
                self._telem_1 = frame_1
                self._last_telem_time = now
                self._foc_rate = foc
                self._last_seq_ack = seq
        except (ValueError, IndexError) as e:
            logger.debug("Telemetry parse error on %s: %s  line=%s", self.port, e, line)

    # --- Writer thread -----------------------------------------------------

    def _writer_loop(self):
        logger.debug("Writer started for %s", self.port)
        while not self._stop_event.is_set():
            cycle_start = time.time()

            # 1. Flush queued one-off commands
            with self._cmd_queue_lock:
                queued = list(self._cmd_queue)
                self._cmd_queue.clear()
            for cmd in queued:
                self._send_raw(cmd)

            # 2. Send C command (only if upper app has set targets)
            if self._control_active:
                seq = self._next_seq()
                with self._control_lock:
                    pos0 = self._control_0.position
                    pos1 = self._control_1.position
                    min0 = self._control_0.min_bound
                    max0 = self._control_0.max_bound
                    min1 = self._control_1.min_bound
                    max1 = self._control_1.max_bound

                cmd = f"C,{seq},{pos0},{pos1}"

                has_bounds_0 = min0 is not None and max0 is not None
                has_bounds_1 = min1 is not None and max1 is not None
                if has_bounds_0 or has_bounds_1:
                    # Protocol requires motor 0 bounds before motor 1 bounds
                    b_min0 = min0 if min0 is not None else -_FULL_RANGE_DECIDEG
                    b_max0 = max0 if max0 is not None else _FULL_RANGE_DECIDEG
                    cmd += f",{b_min0},{b_max0}"
                    if has_bounds_1:
                        cmd += f",{min1},{max1}"

                self._send_raw(cmd)

            # 3. Sleep for remainder of cycle
            elapsed = time.time() - cycle_start
            sleep_time = (1.0 / _HAPTIC_UPDATE_HZ) - elapsed
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)

        logger.debug("Writer stopped for %s", self.port)


# ---------------------------------------------------------------------------
# HapticSystem — public API for the upper-level application
# ---------------------------------------------------------------------------


class HapticSystem:
    """Manages all haptic controllers for one team's set of motors.

    Create with the list of motor IDs you expect to find. Call start() to
    begin background discovery. The system automatically finds controllers,
    connects to them, and reconnects if they disappear.

    The upper application does NOT need to know about COM ports or boards.
    """

    def __init__(
        self,
        expected_motor_ids: list[int],
        motor_bounds: Optional[dict[int, tuple[int, int]]] = None,
    ):
        self.expected_motor_ids = set(expected_motor_ids)

        # port → _BoardConnection
        self._boards: dict[str, _BoardConnection] = {}
        # motor_id → _BoardConnection
        self._motor_to_board: dict[int, _BoardConnection] = {}

        # Bounds applied to each motor on connect: {motor_id: (min_decideg, max_decideg)}
        self._motor_bounds: dict[int, tuple[int, int]] = (
            dict(motor_bounds) if motor_bounds else {}
        )

        self._discovery_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()  # protects _boards and _motor_to_board

    # --- Lifecycle ---------------------------------------------------------

    def start(self):
        """Start background discovery and communication. Non-blocking."""
        self._stop_event.clear()
        self._discovery_thread = threading.Thread(
            target=self._discovery_loop,
            name="haptic-discovery",
            daemon=True,
        )
        self._discovery_thread.start()
        logger.info(
            "HapticSystem started, looking for motors: %s",
            sorted(self.expected_motor_ids),
        )

    def stop(self):
        """Stop all communication and threads."""
        self._stop_event.set()
        if self._discovery_thread and self._discovery_thread.is_alive():
            self._discovery_thread.join(timeout=5.0)
        with self._lock:
            for board in self._boards.values():
                board.disconnect()
            self._boards.clear()
            self._motor_to_board.clear()
        logger.info("HapticSystem stopped")

    # --- Read interface ----------------------------------------------------

    def get_telemetry(self, motor_id: int) -> Optional[TelemetryFrame]:
        """Get the latest telemetry for a motor. Returns None if not connected."""
        with self._lock:
            board = self._motor_to_board.get(motor_id)
        if board is None:
            return None
        return board.get_telemetry(motor_id)

    def get_all_telemetry(self) -> dict[int, Optional[TelemetryFrame]]:
        """Get telemetry for all expected motors. Unconnected motors map to None."""
        result: dict[int, Optional[TelemetryFrame]] = {}
        for motor_id in self.expected_motor_ids:
            result[motor_id] = self.get_telemetry(motor_id)
        return result

    # --- Write interface ---------------------------------------------------

    def set_control(
        self,
        motor_id: int,
        position: int,
        min_bound: Optional[int] = None,
        max_bound: Optional[int] = None,
    ):
        """Set the control target for a motor (decidegrees).

        The internal writer thread will include this in the next C command.
        Silently ignored if the motor is not currently connected.
        """
        with self._lock:
            board = self._motor_to_board.get(motor_id)
        if board:
            board.set_control(motor_id, position, min_bound, max_bound)

    def set_motor_param(self, motor_id: int, param_base_name: str, value: int):
        """Set a per-motor parameter (e.g. 'tracking_kp').

        Automatically appends '_0' or '_1' based on the motor's position on
        its board. Uses the S command.
        """
        with self._lock:
            board = self._motor_to_board.get(motor_id)
        if board is None:
            return
        suffix = "_0" if motor_id == board.motor_id_0 else "_1"
        full_name = param_base_name + suffix
        seq = board._next_seq()
        board.queue_command(f"S,{seq},{full_name},{value}")

    def set_board_param(self, motor_id: int, param_name: str, value: int):
        """Set a board-level parameter (e.g. 'telemetry_interval').

        *motor_id* is only used to find the board; *param_name* is sent as-is.
        """
        with self._lock:
            board = self._motor_to_board.get(motor_id)
        if board is None:
            return
        seq = board._next_seq()
        board.queue_command(f"S,{seq},{param_name},{value}")

    # --- Connection status -------------------------------------------------

    def is_motor_connected(self, motor_id: int) -> bool:
        with self._lock:
            board = self._motor_to_board.get(motor_id)
        return board is not None and board.is_connected

    @property
    def all_connected(self) -> bool:
        """True when every expected motor is connected and receiving telemetry."""
        return all(self.is_motor_connected(mid) for mid in self.expected_motor_ids)

    @property
    def connected_motor_ids(self) -> set[int]:
        """Set of motor IDs that are currently connected."""
        return {mid for mid in self.expected_motor_ids if self.is_motor_connected(mid)}

    # --- Background discovery ----------------------------------------------

    def _discovery_loop(self):
        """Periodically scan for controllers and clean up disconnected ones."""
        logger.debug("Discovery thread started")
        while not self._stop_event.is_set():
            try:
                self._cleanup_disconnected()
                self._scan_and_connect()
            except Exception as e:
                logger.error("Discovery error: %s", e, exc_info=True)
            # Wait before next scan (interruptible by stop_event)
            self._stop_event.wait(_DISCOVERY_INTERVAL_S)
        logger.debug("Discovery thread stopped")

    def _cleanup_disconnected(self):
        """Remove boards whose telemetry has timed out."""
        with self._lock:
            dead_ports = [
                port for port, board in self._boards.items() if not board.is_connected
            ]
            for port in dead_ports:
                board = self._boards.pop(port)
                for mid in board.motor_ids:
                    self._motor_to_board.pop(mid, None)
                logger.info(
                    "Removed disconnected board on %s (motors %d, %d)",
                    port,
                    board.motor_id_0,
                    board.motor_id_1,
                )
                board.disconnect()

    def _scan_and_connect(self):
        """Scan COM ports for new controllers with expected motor IDs."""
        managed_ports = self._get_managed_ports()

        # Find CH340 ports we're not already managing
        candidates = []
        for info in serial.tools.list_ports.comports():
            if (info.vid, info.pid) in _CH340_VID_PIDS:
                if info.device not in managed_ports:
                    candidates.append(info.device)

        for port in candidates:
            if self._stop_event.is_set():
                return
            # Acquire exclusive access before probing
            if not port_registry.acquire_port(port, owner="haptic"):
                continue
            try:
                probe_result = self._probe_port(port)
            finally:
                port_registry.release_port(port)
            if probe_result is None:
                continue

            id0, id1 = probe_result

            # Only connect if at least one motor ID is in our expected set
            if (
                id0 not in self.expected_motor_ids
                and id1 not in self.expected_motor_ids
            ):
                logger.debug(
                    "Ignoring %s: motors %d,%d not in expected set", port, id0, id1
                )
                continue

            board = _BoardConnection(port, id0, id1)
            board.connect()

            # Apply default bounds on connect so dials have correct range immediately
            self._apply_motor_bounds(board)

            with self._lock:
                self._boards[port] = board
                if id0 in self.expected_motor_ids:
                    self._motor_to_board[id0] = board
                if id1 in self.expected_motor_ids:
                    self._motor_to_board[id1] = board

            logger.info("Discovered motors %d, %d on %s", id0, id1, port)

    def _get_managed_ports(self) -> set[str]:
        with self._lock:
            return set(self._boards.keys())

    def _apply_motor_bounds(self, board: _BoardConnection):
        """Send configured bounds to a newly connected board."""
        for motor_id in board.motor_ids:
            bounds = self._motor_bounds.get(motor_id)
            if bounds:
                min_b, max_b = bounds
                # Use position=0 with bounds; the main loop will update
                # position shortly. The brief tracking-to-0 force is minimal.
                board.set_control(
                    motor_id, position=0, min_bound=min_b, max_bound=max_b
                )
                logger.info(
                    "Applied bounds to motor %d: [%d, %d] decideg",
                    motor_id,
                    min_b,
                    max_b,
                )

    @staticmethod
    def _probe_port(port: str) -> Optional[tuple[int, int]]:
        """Open *port*, send V and I queries, return (motor_id_0, motor_id_1) or None."""
        try:
            # Open without toggling DTR/RTS to avoid resetting the ESP32
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = _BAUDRATE
            ser.timeout = 0.5
            ser.dtr = False
            ser.rts = False
            ser.open()
            ser.reset_input_buffer()
            time.sleep(0.1)  # let telemetry start flowing

            ser.write(b"V,1\n")
            ser.write(b"I,2\n")

            version = None
            motor_ids = None
            deadline = time.time() + _PROBE_TIMEOUT_S

            while time.time() < deadline:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()

                if line.startswith("V,1,"):
                    parts = line.split(",")
                    if len(parts) >= 3:
                        version = parts[2]
                elif line.startswith("I,2,"):
                    parts = line.split(",")
                    if len(parts) >= 4:
                        motor_ids = (int(parts[2]), int(parts[3]))

                if version is not None and motor_ids is not None:
                    break

            ser.close()

            if motor_ids:
                logger.debug(
                    "Probed %s: version=%s motors=%s", port, version, motor_ids
                )
                return motor_ids
            else:
                logger.debug("Probed %s: no valid identity response", port)
                return None

        except (serial.SerialException, OSError, ValueError) as e:
            logger.debug("Probe failed for %s: %s", port, e)
            return None


# ---------------------------------------------------------------------------
# SimulatedHapticSystem — drop-in replacement for testing without hardware
# ---------------------------------------------------------------------------


class SimulatedHapticSystem:
    """Simulated haptic controllers for software testing without hardware.

    Drop-in replacement for HapticSystem. Reads simulated dial angles from
    GameSettings.sim_dial_angles (joint degrees) and converts them to
    TelemetryFrame objects in decidegrees (dial space) using the gear ratio.

    All motors report as connected immediately on start().
    """

    def __init__(
        self,
        expected_motor_ids: list[int],
        settings,
        motor_bounds: Optional[dict[int, tuple[int, int]]] = None,
    ):
        self.expected_motor_ids = set(expected_motor_ids)
        self._settings = settings
        self._motor_bounds = dict(motor_bounds) if motor_bounds else {}
        self._started = False

    def start(self):
        self._started = True
        logger.info(
            "SimulatedHapticSystem started (motors: %s)",
            sorted(self.expected_motor_ids),
        )

    def stop(self):
        self._started = False
        logger.info("SimulatedHapticSystem stopped")

    def get_telemetry(self, motor_id: int) -> Optional[TelemetryFrame]:
        if not self._started or motor_id not in self.expected_motor_ids:
            return None
        sim_angles = self._settings.get("sim_dial_angles")
        joint_deg = sim_angles.get(motor_id, 0.0)
        gear_ratio = self._settings.get("gear_ratio")
        decideg = int(round(joint_deg * gear_ratio * 10.0))
        return TelemetryFrame(
            motor_id=motor_id,
            angle=decideg,
            speed=0,
            torque=0,
            timestamp=time.time(),
        )

    def get_all_telemetry(self) -> dict[int, Optional[TelemetryFrame]]:
        return {mid: self.get_telemetry(mid) for mid in self.expected_motor_ids}

    def set_control(
        self,
        motor_id: int,
        position: int,
        min_bound: Optional[int] = None,
        max_bound: Optional[int] = None,
    ):
        pass  # No physical motors to drive

    def set_motor_param(self, motor_id: int, param_base_name: str, value: int):
        pass

    def set_board_param(self, motor_id: int, param_name: str, value: int):
        pass

    def is_motor_connected(self, motor_id: int) -> bool:
        return self._started and motor_id in self.expected_motor_ids

    @property
    def all_connected(self) -> bool:
        return self._started

    @property
    def connected_motor_ids(self) -> set[int]:
        if self._started:
            return set(self.expected_motor_ids)
        return set()


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    TEAM_1_MOTORS = [11, 12, 13, 14, 15, 16]

    system = HapticSystem(expected_motor_ids=TEAM_1_MOTORS)
    system.start()

    print(f"Looking for motors: {TEAM_1_MOTORS}")
    print("Waiting for all motors to connect...")

    try:
        # --- Wait for connection ---
        while not system.all_connected:
            connected = sorted(system.connected_motor_ids)
            print(
                f"\r  Connected: {connected} ({len(connected)}/{len(TEAM_1_MOTORS)})",
                end="",
                flush=True,
            )
            time.sleep(0.5)

        print(f"\nAll {len(TEAM_1_MOTORS)} motors connected!\n")

        # --- Stream telemetry ---
        header = "  ".join(f" M{mid:<3}" for mid in TEAM_1_MOTORS)
        print(f"  {header}")
        print(f"  {'--------' * len(TEAM_1_MOTORS)}")

        while True:
            telemetry = system.get_all_telemetry()
            values = []
            for mid in TEAM_1_MOTORS:
                t = telemetry.get(mid)
                if t is not None:
                    # Display angle in degrees (1 decimal)
                    deg = t.angle / 10.0
                    values.append(f"{deg:>7.1f}")
                else:
                    values.append("   --- ")
            print(f"\r  {'  '.join(values)}", end="", flush=True)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        system.stop()
        print("Done.")
