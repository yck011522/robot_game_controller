"""RS485 LED controller communication layer.

Handles:
  - RS485 adapter discovery and connection
  - Command generation for the LED protocol
  - LEDStrip abstraction: logical strips (11, 12, 21, ..., 82) mapped to physical
    controllers and channels
  - Individual LED control per strip (0-27 LEDs, each with RGB)

The 8 physical controllers are arranged around the arena with 2 strips each (16 total).
Logical strip IDs follow column + position: e.g., strip 11 = column 1, strip 1;
strip 21 = column 2, strip 1; etc.

Strip XY mapping (center of arena = 0,0):
  11: (-2367, 0),    12: (-2272, 0)
  21: (-1868, 1480), 22: (-1773, 1480)
  31: (-1868, -1480), 32: (-1773, -1480)
  41: (-48, 1480),   42: (47, 1480)
  51: (-48, -1480),  52: (47, -1480)
  61: (1773, 1480),  62: (1868, 1480)
  71: (1773, -1480), 72: (1868, -1480)
  81: (2272, 0),     82: (2367, 0)

Protocol: RS485 at 9600 baud (or user-specified).
Command format (WS2811/12V LED strips):
  [Header: DD 55 EE] [Group: 00 00] [Device: 00 XX] [Port: 01/02] [Function: 99]
  [LED Type: 02] [Reserved: 00 00] [Length: 00 54] [Repeat: 00 01]
  [Color Data: 28 LEDs × 3 bytes (RGB)] [Tail: AA BB]
"""

import time
import threading
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Tuple, List
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

import port_registry

logger = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

_PROTOCOL_HEADER = bytes([0xDD, 0x55, 0xEE])
_PROTOCOL_TAIL = bytes([0xAA, 0xBB])
_GROUP_BROADCAST = bytes([0x00, 0x00])
_FUNCTION_DISPLAY = 0x99
_FUNCTION_QUERY_DEVICE_ADDR = 0x94
_LED_TYPE_WS2811 = 0x02
_LED_TYPE_RESERVED = bytes([0x00, 0x00])
_LED_LENGTH = 0x54  # 84 bytes = 28 LEDs × 3 RGB
_LED_REPEAT = bytes([0x00, 0x01])
_LEDS_PER_STRIP = 28
_DEFAULT_BAUDRATE = 921600
# Validated at 921600 baud: 2 ms is reliable, 1 ms produces corrupted colors.
_RECOMMENDED_MIN_INTER_COMMAND_S = 0.002
_DEFAULT_INTER_COMMAND_S = 0.002

# ─────────────────────────────────────────────────────────────────────────────
# USB-RS485 Adapter Detection
# ─────────────────────────────────────────────────────────────────────────────

# VID/PID pairs for USB-to-RS485 adapters used with LED controllers.
# NOTE: CH340 VIDs overlap with haptic ESP32 controllers. The discovery
# system differentiates by probing the LED protocol on each candidate port.
_RS485_LED_VID_PIDS = {
    (0x0403, 0x6001),  # FTDI FT232R
    (0x0403, 0x6015),  # FTDI FT-X series
    (0x10C4, 0xEA60),  # Silicon Labs CP210x
    (0x067B, 0x2303),  # Prolific PL2303
    (0x1A86, 0x7522),  # CH340
    (0x1A86, 0x7523),  # CH341
    (0x1A86, 0x55D3),  # CH343
}

_PROBE_TIMEOUT_S = 0.3        # Seconds to wait for controller reply during initial probe
_PROBE_QUICK_TIMEOUT_S = 0.020  # Short timeout for quick-probes during animation (20 ms)
_PROBE_REPLY_TERMINATOR = b'\r\n'  # All real controller replies end with \r\n
_DISCOVERY_INTERVAL_S = 5.0   # Seconds between background discovery scans
_MAX_CONTROLLER_ADDR = 8      # Probe device addresses 1..N

# Strip ID to physical controller address (1–8, corresponding to server addresses)
# Each controller drives 2 channels (DAT = channel 1, CLK = channel 2)
_STRIP_TO_CONTROLLER: Dict[int, Tuple[int, int]] = {
    11: (1, 1),  # Column 1, strip A → Controller 1, Channel 1
    12: (1, 2),  # Column 1, strip B → Controller 1, Channel 2
    21: (2, 1),
    22: (2, 2),
    31: (3, 1),
    32: (3, 2),
    41: (4, 1),
    42: (4, 2),
    51: (5, 1),
    52: (5, 2),
    61: (6, 1),
    62: (6, 2),
    71: (7, 1),
    72: (7, 2),
    81: (8, 1),
    82: (8, 2),
}

# Strip ID to XY physical location
_STRIP_LOCATIONS: Dict[int, Tuple[float, float]] = {
    11: (-2367, 0),
    12: (-2272, 0),
    21: (-1868, 1480),
    22: (-1773, 1480),
    31: (-1868, -1480),
    32: (-1773, -1480),
    41: (-48, 1480),
    42: (47, 1480),
    51: (-48, -1480),
    52: (47, -1480),
    61: (1773, 1480),
    62: (1868, 1480),
    71: (1773, -1480),
    72: (1868, -1480),
    81: (2272, 0),
    82: (2367, 0),
}


# ─────────────────────────────────────────────────────────────────────────────
# Data Classes
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Color:
    """RGB color."""
    r: int = 0
    g: int = 0
    b: int = 0

    def __post_init__(self):
        """Clamp values to 0–255."""
        self.r = max(0, min(255, self.r))
        self.g = max(0, min(255, self.g))
        self.b = max(0, min(255, self.b))

    def to_bytes(self) -> bytes:
        """Serialize to RGB bytes."""
        return bytes([self.r, self.g, self.b])

    @staticmethod
    def from_hex(hex_str: str) -> "Color":
        """Parse from hex string (e.g., '#FF0000' or 'FF0000')."""
        hex_str = hex_str.lstrip("#")
        return Color(
            r=int(hex_str[0:2], 16),
            g=int(hex_str[2:4], 16),
            b=int(hex_str[4:6], 16),
        )


# Common colors
RED = Color(255, 0, 0)
GREEN = Color(0, 255, 0)
BLUE = Color(0, 0, 255)
WHITE = Color(255, 255, 255)
OFF = Color(0, 0, 0)


@dataclass
class LEDStripState:
    """Current state of one LED strip: list of 28 Color objects."""
    strip_id: int
    leds: List[Color] = None

    def __post_init__(self):
        if self.leds is None:
            self.leds = [OFF] * _LEDS_PER_STRIP

    def copy(self) -> "LEDStripState":
        """Return a deep copy."""
        return LEDStripState(
            strip_id=self.strip_id,
            leds=[Color(c.r, c.g, c.b) for c in self.leds],
        )


# ─────────────────────────────────────────────────────────────────────────────
# Command Builder
# ─────────────────────────────────────────────────────────────────────────────

class LEDCommandBuilder:
    """Builds RS485 commands for LED strips."""

    @staticmethod
    def _u16(value: int) -> bytes:
        """Serialize an unsigned 16-bit integer in big-endian order."""
        if value < 0 or value > 0xFFFF:
            raise ValueError(f"u16 out of range: {value}")
        return bytes([(value >> 8) & 0xFF, value & 0xFF])

    @staticmethod
    def build_device_channel_command(
        device_addr: int,
        channel: int,
        colors: List[Color],
        group_addr: int = 0,
        led_type: int = _LED_TYPE_WS2811,
        repeat: int = 1,
    ) -> bytes:
        """Build command using explicit device address + channel.

        This matches the vendor-tool framing:
          [DD 55 EE][group u16][device u16][channel u8][99][led_type u8]
          [00 00][length u16][repeat u16][payload][AA BB]
        """
        if channel not in (1, 2):
            raise ValueError(f"Channel must be 1 or 2, got {channel}")
        if device_addr < 0 or device_addr > 0xFFFF:
            raise ValueError(f"Invalid device address: {device_addr}")
        if len(colors) != _LEDS_PER_STRIP:
            raise ValueError(f"Expected {_LEDS_PER_STRIP} colors, got {len(colors)}")

        color_data = b"".join(c.to_bytes() for c in colors)
        payload_len = len(color_data)

        return (
            _PROTOCOL_HEADER
            + LEDCommandBuilder._u16(group_addr)
            + LEDCommandBuilder._u16(device_addr)
            + bytes([channel])
            + bytes([_FUNCTION_DISPLAY])
            + bytes([led_type & 0xFF])
            + _LED_TYPE_RESERVED
            + LEDCommandBuilder._u16(payload_len)
            + LEDCommandBuilder._u16(repeat)
            + color_data
            + _PROTOCOL_TAIL
        )

    @staticmethod
    def build_strip_command(strip_id: int, colors: List[Color]) -> bytes:
        """
        Build a complete RS485 command for a strip.

        Args:
            strip_id: Logical strip ID (11, 12, 21, ..., 82)
            colors: List of exactly 28 Color objects

        Returns:
            Complete command as bytes
        """
        if strip_id not in _STRIP_TO_CONTROLLER:
            raise ValueError(f"Unknown strip ID: {strip_id}")

        controller_addr, channel = _STRIP_TO_CONTROLLER[strip_id]
        return LEDCommandBuilder.build_device_channel_command(
            device_addr=controller_addr,
            channel=channel,
            colors=colors,
            group_addr=0,
            led_type=_LED_TYPE_WS2811,
            repeat=1,
        )

    @staticmethod
    def build_probe_command(device_addr: int) -> bytes:
        """Build a query-device-address command (function 0x94) for discovery.

        A present controller replies with two lines::

            RecvEnd\\r\\n
            <4-digit-hex-addr>\\r\\n

        e.g. ``RecvEnd\\r\\n0001\\r\\n`` (15 bytes total) for address 1.
        No reply means the address is absent on this bus.

        Command format (21 bytes)::

            DD 55 EE  00 00  [addr_hi addr_lo]  00  94
            02  00 00  00 03  00 01  00 00 00  AA BB
        """
        return (
            _PROTOCOL_HEADER
            + _GROUP_BROADCAST
            + LEDCommandBuilder._u16(device_addr)
            + bytes([0x00])                        # port
            + bytes([_FUNCTION_QUERY_DEVICE_ADDR])  # 0x94
            + bytes([_LED_TYPE_WS2811])             # led type
            + _LED_TYPE_RESERVED                    # reserved 00 00
            + bytes([0x00, 0x03])                   # data length = 3
            + bytes([0x00, 0x01])                   # repeat = 1
            + bytes([0x00, 0x00, 0x00])             # colour data (placeholder)
            + _PROTOCOL_TAIL
        )


# ─────────────────────────────────────────────────────────────────────────────
# LEDStrip Class
# ─────────────────────────────────────────────────────────────────────────────

class LEDStrip:
    """Logical LED strip abstraction."""

    def __init__(self, strip_id: int):
        """
        Args:
            strip_id: Logical strip number (11, 12, 21, ..., 82)
        """
        if strip_id not in _STRIP_TO_CONTROLLER:
            raise ValueError(f"Unknown strip ID: {strip_id}")

        self.id = strip_id
        self.x, self.y = _STRIP_LOCATIONS[strip_id]
        self.controller_addr, self.channel = _STRIP_TO_CONTROLLER[strip_id]
        self.state = LEDStripState(strip_id)

    def set_color(self, color: Color) -> None:
        """Set all LEDs to a single color."""
        self.state.leds = [color] * _LEDS_PER_STRIP

    def set_colors(self, colors: List[Color]) -> None:
        """Set each LED individually (must be exactly 28 colors)."""
        if len(colors) != _LEDS_PER_STRIP:
            raise ValueError(f"Expected {_LEDS_PER_STRIP} colors, got {len(colors)}")
        self.state.leds = [Color(c.r, c.g, c.b) for c in colors]

    def set_fill(self, num_leds: int, color: Color, off_color: Color = None) -> None:
        """
        Fill first num_leds with color, rest with off_color (default OFF).

        Args:
            num_leds: Number of LEDs to fill (0–28)
            color: Color for filled LEDs
            off_color: Color for unfilled LEDs (default Color.OFF)
        """
        if off_color is None:
            off_color = OFF
        num_leds = max(0, min(_LEDS_PER_STRIP, num_leds))
        self.state.leds = [color] * num_leds + [off_color] * (_LEDS_PER_STRIP - num_leds)

    def get_command(self) -> bytes:
        """Generate RS485 command for current state."""
        return LEDCommandBuilder.build_strip_command(self.id, self.state.leds)


# ─────────────────────────────────────────────────────────────────────────────
# RS485 Connection Manager
# ─────────────────────────────────────────────────────────────────────────────

class RS485Connection:
    """Manages serial connection to RS485 adapter."""

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = _DEFAULT_BAUDRATE,
        write_timeout: float = 1.0,
    ):
        """
        Args:
            port: Serial port name (e.g., 'COM3'). If None, auto-detect.
            baudrate: Baud rate (default 9600)
        """
        self.port = port
        self.baudrate = baudrate
        self.write_timeout = write_timeout
        self.ser = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        """Open the serial connection. Return True if successful."""
        with self._lock:
            if self.ser and self.ser.is_open:
                return True

            port_to_try = self.port
            if not port_to_try:
                port_to_try = self._auto_detect_port()

            if not port_to_try:
                logger.error("No RS485 adapter found")
                return False

            if serial is None:
                logger.error("pyserial is not installed")
                return False

            try:
                self.ser = serial.Serial(
                    port=port_to_try,
                    baudrate=self.baudrate,
                    timeout=0.1,
                    write_timeout=self.write_timeout,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                logger.info(f"Opened RS485 connection on {port_to_try}")
                return True
            except Exception as e:
                logger.error(f"Failed to open RS485 connection: {e}")
                return False

    def close(self) -> None:
        """Close the serial connection."""
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
                logger.info("Closed RS485 connection")

    def send(self, data: bytes) -> bool:
        """Send data. Return True if successful."""
        with self._lock:
            if not self.ser or not self.ser.is_open:
                return False

            try:
                self.ser.write(data)
                self.ser.flush()
                return True
            except Exception as e:
                logger.error(f"Failed to send RS485 data: {e}")
                return False

    def receive(self, size: int = 256, timeout_s: float = 0.5) -> bytes:
        """Read available bytes from the serial port.

        Args:
            size: Maximum number of bytes to read.
            timeout_s: Read timeout in seconds.

        Returns:
            Bytes received (may be empty).
        """
        with self._lock:
            if not self.ser or not self.ser.is_open:
                return b""
            try:
                saved_timeout = self.ser.timeout
                self.ser.timeout = timeout_s
                data = self.ser.read(size)
                self.ser.timeout = saved_timeout
                return data
            except Exception as e:
                logger.error(f"Failed to receive RS485 data: {e}")
                return b""

    def flush_input(self) -> None:
        """Discard any pending input data."""
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

    @property
    def is_open(self) -> bool:
        """True if the serial port is currently open."""
        return self.ser is not None and self.ser.is_open

    @staticmethod
    def find_led_rs485_ports(
        vid_pids: set = None,
        exclude_ports: set = None,
    ) -> List[str]:
        """Find COM ports matching LED RS485 adapter VID/PIDs.

        Args:
            vid_pids: Set of (VID, PID) tuples to match.
                      Defaults to _RS485_LED_VID_PIDS.
            exclude_ports: Port device names to skip
                           (e.g. ports used by the haptic system).

        Returns:
            Sorted list of port device names (e.g. ['COM3', 'COM5']).
        """
        if serial is None:
            return []
        if vid_pids is None:
            vid_pids = _RS485_LED_VID_PIDS
        if exclude_ports is None:
            exclude_ports = set()

        ports = []
        for info in serial.tools.list_ports.comports():
            if info.device in exclude_ports:
                continue
            if info.vid is not None and info.pid is not None:
                if (info.vid, info.pid) in vid_pids:
                    ports.append(info.device)
        return sorted(ports)

    @staticmethod
    def _auto_detect_port() -> Optional[str]:
        """Auto-detect a single RS485 adapter port."""
        ports = RS485Connection.find_led_rs485_ports()
        if ports:
            return ports[0]
        # Fallback: first USB serial device
        if serial:
            for port_info in serial.tools.list_ports.comports():
                if "USB" in port_info.description:
                    return port_info.device
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Bus Connection (internal — one per RS485 bus / COM port)
# ─────────────────────────────────────────────────────────────────────────────

class _BusConnection:
    """Wraps one RS485 serial port with its own command queue and sender thread."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        inter_command_delay_s: float,
        debug_hex: bool,
    ):
        self.port = port
        self.conn = RS485Connection(port, baudrate)
        self._inter_command_delay_s = inter_command_delay_s
        self._debug_hex = debug_hex
        self._command_queue: deque[bytes] = deque()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._queue_lock = threading.Lock()
        self._bus_lock = threading.Lock()  # atomic bus access (probe vs sender)
        self.discovered_controllers: set = set()

    def open(self) -> bool:
        """Open the serial port (does NOT start the sender thread)."""
        return self.conn.open()

    def start_sender(self) -> None:
        """Start the background sender thread."""
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._sender_loop,
            name=f"led-sender-{self.port}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop sender thread and close serial port."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.conn.close()

    def queue_command(self, cmd: bytes) -> None:
        with self._queue_lock:
            self._command_queue.append(cmd)

    @property
    def is_open(self) -> bool:
        return self.conn.is_open

    def probe_controller(
        self, device_addr: int, timeout_s: float = _PROBE_TIMEOUT_S,
    ) -> bool:
        """Atomic probe: flush + send + receive under bus lock.

        Safe to call even while the sender thread is running — the bus
        lock ensures no animation command is interleaved between the
        probe send and the probe receive.  Use a short *timeout_s*
        (e.g. ``_PROBE_QUICK_TIMEOUT_S``) during animation to minimise
        disruption.
        """
        probe_cmd = LEDCommandBuilder.build_probe_command(device_addr)
        with self._bus_lock:
            self.conn.flush_input()
            if not self.conn.send(probe_cmd):
                return False
            response = self.conn.receive(size=256, timeout_s=timeout_s)

        if response and _PROBE_REPLY_TERMINATOR in response:
            text = response.decode("ascii", errors="replace").strip()
            logger.debug(
                "Probe addr %d on %s: controller detected (reply: %s)",
                device_addr, self.port, text,
            )
            return True
        elif response:
            logger.debug(
                "Probe addr %d on %s: ignoring noise (%d byte(s), "
                "no \\r\\n terminator)",
                device_addr, self.port, len(response),
            )
        return False

    def _sender_loop(self) -> None:
        interval = 1.0 / 50
        while not self._stop_event.is_set():
            cmd = None
            with self._queue_lock:
                if self._command_queue:
                    cmd = self._command_queue.popleft()

            if cmd is None:
                self._stop_event.wait(interval)
                continue

            if self._debug_hex:
                logger.info(
                    "LED TX %s (%d bytes): %s",
                    self.port, len(cmd), cmd.hex(" "),
                )

            with self._bus_lock:
                self.conn.send(cmd)

            if self._inter_command_delay_s > 0:
                self._stop_event.wait(self._inter_command_delay_s)


# ─────────────────────────────────────────────────────────────────────────────
# LED System Manager (multi-bus)
# ─────────────────────────────────────────────────────────────────────────────

class LEDSystem:
    """High-level LED strip manager with multi-bus support and auto-discovery.

    Supports multiple RS485 buses simultaneously.  Each bus connects to a
    subset of the 8 LED controllers.  The system auto-discovers which
    controllers are on which bus by probing device addresses 1–8 and
    listening for an acknowledgement frame.

    The public API is identical to the original single-bus version — callers
    do not need to know about buses:

        system.set_strip_color(11, RED)   # routed to the correct bus
    """

    def __init__(
        self,
        serial_port: Optional[str] = None,
        serial_ports: Optional[List[str]] = None,
        baudrate: int = _DEFAULT_BAUDRATE,
        inter_command_delay_s: float = _DEFAULT_INTER_COMMAND_S,
        debug_hex: bool = False,
        auto_discover: bool = True,
        exclude_ports: Optional[set] = None,
    ):
        """
        Args:
            serial_port: Single RS485 port (legacy compatibility).
            serial_ports: Explicit list of RS485 ports.
                          Takes precedence over *serial_port*.
            baudrate: RS485 baud rate.
            inter_command_delay_s: Delay between RS485 frames.
            debug_hex: If True, log outgoing frame hex.
            auto_discover: If True, run a background thread that scans for
                           RS485 adapters and probes LED controllers.
            exclude_ports: COM port names to skip during auto-discovery
                           (e.g. ports claimed by the haptic system).
        """
        self._baudrate = baudrate
        self._inter_command_delay_s = max(0.0, inter_command_delay_s)
        self._debug_hex = debug_hex
        self._auto_discover = auto_discover
        self._exclude_ports = set(exclude_ports) if exclude_ports else set()

        if 0 < self._inter_command_delay_s < _RECOMMENDED_MIN_INTER_COMMAND_S:
            logger.warning(
                "inter_command_delay_s=%.4f is below recommended %.4f; "
                "frames may corrupt or be ignored",
                self._inter_command_delay_s,
                _RECOMMENDED_MIN_INTER_COMMAND_S,
            )

        # All logical strips (always present, even before discovery)
        self.strips: Dict[int, LEDStrip] = {
            sid: LEDStrip(sid) for sid in _STRIP_TO_CONTROLLER.keys()
        }

        # Bus connections: port name → _BusConnection
        self._buses: Dict[str, _BusConnection] = {}
        # Source-of-truth: controller addr → port name
        self._controller_to_port: Dict[int, str] = {}
        # Derived from _controller_to_port: strip_id → port name
        self._strip_to_port: Dict[int, str] = {}

        # Explicit ports provided at init
        self._initial_ports: List[str] = []
        if serial_ports:
            self._initial_ports = list(serial_ports)
        elif serial_port:
            self._initial_ports = [serial_port]

        self._discovery_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    # ─── Lifecycle ────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Open bus(es) and optionally start background discovery.

        Returns True if at least one bus connects (or discovery is started).
        """
        self._stop_event.clear()
        success = False

        # 1. Connect to explicitly provided ports (full probe)
        for port in self._initial_ports:
            if self._open_and_probe_port(
                port, range(1, _MAX_CONTROLLER_ADDR + 1), _PROBE_TIMEOUT_S,
            ):
                success = True

        # 2. Start background discovery
        if self._auto_discover:
            self._discovery_thread = threading.Thread(
                target=self._discovery_loop,
                name="led-discovery",
                daemon=True,
            )
            self._discovery_thread.start()
            # When no explicit ports were given, wait for the first
            # discovery pass to complete.  Worst case is
            # _MAX_CONTROLLER_ADDR * _PROBE_TIMEOUT_S per candidate port.
            if not success:
                max_wait = (_MAX_CONTROLLER_ADDR * _PROBE_TIMEOUT_S) + 2.0
                waited = 0.0
                poll = 0.25
                while waited < max_wait:
                    time.sleep(poll)
                    waited += poll
                    with self._lock:
                        if self._buses:
                            success = True
                            break

        if not success:
            logger.warning(
                "No LED RS485 buses found. "
                "LED commands will be silently discarded."
            )

        return success

    def stop(self) -> None:
        """Stop all buses and discovery thread."""
        self._stop_event.set()
        if self._discovery_thread and self._discovery_thread.is_alive():
            self._discovery_thread.join(timeout=5.0)
        with self._lock:
            for port, bus in self._buses.items():
                bus.stop()
                port_registry.release_port(port)
            self._buses.clear()
            self._controller_to_port.clear()
            self._strip_to_port.clear()

    # ─── Public API (unchanged from single-bus) ───────────────────────────

    def set_strip_color(self, strip_id: int, color: Color) -> None:
        """Set all LEDs on a strip to a single color."""
        if strip_id not in self.strips:
            raise ValueError(f"Unknown strip ID: {strip_id}")
        self.strips[strip_id].set_color(color)
        self._queue_strip_command(strip_id)

    def set_strip_fill(
        self, strip_id: int, num_leds: int, color: Color, off_color: Color = None
    ) -> None:
        """Fill first num_leds on a strip, rest with off_color."""
        if strip_id not in self.strips:
            raise ValueError(f"Unknown strip ID: {strip_id}")
        self.strips[strip_id].set_fill(num_leds, color, off_color)
        self._queue_strip_command(strip_id)

    def set_strip_colors(self, strip_id: int, colors: List[Color]) -> None:
        """Set each LED individually on a strip."""
        if strip_id not in self.strips:
            raise ValueError(f"Unknown strip ID: {strip_id}")
        self.strips[strip_id].set_colors(colors)
        self._queue_strip_command(strip_id)

    def get_strip_state(self, strip_id: int) -> LEDStripState:
        """Get current state of a strip (snapshot)."""
        if strip_id not in self.strips:
            raise ValueError(f"Unknown strip ID: {strip_id}")
        return self.strips[strip_id].state.copy()

    def get_all_strip_states(self) -> Dict[int, LEDStripState]:
        """Get snapshot of all strips."""
        return {sid: self.strips[sid].state.copy() for sid in self.strips.keys()}

    # ─── Public API (new — discovery info) ────────────────────────────────

    def get_discovered_mapping(self) -> Dict[int, str]:
        """Return current strip_id → COM port mapping (snapshot)."""
        with self._lock:
            return dict(self._strip_to_port)

    def get_bus_info(self) -> Dict[str, set]:
        """Return port → set-of-controller-addresses mapping (snapshot)."""
        with self._lock:
            return {
                port: set(bus.discovered_controllers)
                for port, bus in self._buses.items()
            }

    # ─── Internal: command routing ────────────────────────────────────────

    def _queue_strip_command(self, strip_id: int) -> None:
        """Route a command to the correct bus based on discovered mapping."""
        cmd = self.strips[strip_id].get_command()
        with self._lock:
            port = self._strip_to_port.get(strip_id)
            if port and port in self._buses:
                self._buses[port].queue_command(cmd)
            # else: strip not yet discovered on any bus — silently discard

    # ─── Internal: bus management ─────────────────────────────────────────

    def _open_and_probe_port(
        self,
        port: str,
        addrs_to_probe,
        timeout_s: float,
    ) -> bool:
        """Open *port*, probe the given addresses, keep the bus if any reply.

        Returns True if at least one controller was found.
        """
        bus = _BusConnection(
            port, self._baudrate, self._inter_command_delay_s, self._debug_hex,
        )
        if not bus.open():
            logger.warning("Failed to open RS485 port %s", port)
            return False

        # Full probe (sender thread not yet running).
        discovered: set = set()
        for addr in sorted(addrs_to_probe):
            if self._stop_event.is_set():
                break
            if bus.probe_controller(addr, timeout_s=timeout_s):
                discovered.add(addr)

        if not discovered:
            bus.stop()
            logger.debug("Port %s: no controllers found, closing", port)
            return False

        # Found controllers — keep this bus running.
        bus.discovered_controllers = discovered
        bus.start_sender()

        with self._lock:
            self._buses[port] = bus
            for addr in discovered:
                self._register_controller(addr, port)

        logger.info(
            "Bus %s: controllers %s  →  strips %s",
            port,
            sorted(discovered),
            sorted(
                sid for sid, p in self._strip_to_port.items() if p == port
            ),
        )
        return True

    # ─── Internal: controller ↔ port mapping ──────────────────────────────

    def _register_controller(self, addr: int, port: str) -> None:
        """Map a controller address to a port.  Caller must hold ``_lock``."""
        old_port = self._controller_to_port.get(addr)
        self._controller_to_port[addr] = port
        # Update the bus-level set
        if port in self._buses:
            self._buses[port].discovered_controllers.add(addr)
        # If the controller moved from another bus, remove it there
        if old_port and old_port != port and old_port in self._buses:
            self._buses[old_port].discovered_controllers.discard(addr)
        self._rebuild_strip_to_port()

    def _unregister_port(self, port: str) -> None:
        """Remove all controller mappings for *port*.  Caller must hold ``_lock``."""
        addrs = [
            a for a, p in self._controller_to_port.items() if p == port
        ]
        for a in addrs:
            del self._controller_to_port[a]
        self._rebuild_strip_to_port()

    def _rebuild_strip_to_port(self) -> None:
        """Derive ``_strip_to_port`` from ``_controller_to_port``.  Caller must hold ``_lock``."""
        self._strip_to_port = {}
        for strip_id, (ctrl_addr, _ch) in _STRIP_TO_CONTROLLER.items():
            port = self._controller_to_port.get(ctrl_addr)
            if port:
                self._strip_to_port[strip_id] = port

    def _find_missing_controllers(self) -> set:
        """Return controller addresses not yet mapped to any port."""
        all_addrs = set(range(1, _MAX_CONTROLLER_ADDR + 1))
        with self._lock:
            return all_addrs - set(self._controller_to_port.keys())

    # ─── Internal: background discovery ───────────────────────────────────

    def _discovery_loop(self) -> None:
        """Background thread: find missing controllers.

        Strategy executed every ``_DISCOVERY_INTERVAL_S`` seconds:

        1. Remove buses whose serial port disappeared (USB unplug).
        2. Determine which controller addresses are still missing.
        3. **Quick-probe** missing addresses on already-open buses
           (20 ms timeout — does not block the animation sender).
        4. If addresses are still missing, look for **new USB COM ports**
           that match the VID/PID list, open each and do a full-timeout
           probe for the missing addresses.
        5. Close any bus that ended up with zero controllers.
        """
        logger.debug("LED discovery thread started")

        # Initial thorough scan on first run.
        self._scan_new_ports(set(range(1, _MAX_CONTROLLER_ADDR + 1)))

        while not self._stop_event.is_set():
            self._stop_event.wait(_DISCOVERY_INTERVAL_S)
            if self._stop_event.is_set():
                break
            try:
                self._cleanup_disconnected()
                missing = self._find_missing_controllers()
                if not missing:
                    continue

                logger.debug("Missing controllers: %s", sorted(missing))

                # Step 1: quick-probe on existing open buses
                found = self._quick_probe_existing_buses(missing)
                still_missing = missing - found

                # Step 2: try newly-appeared USB ports
                if still_missing:
                    self._scan_new_ports(still_missing)

                # Step 3: close buses that have no controllers left
                self._cleanup_empty_buses()
            except Exception as e:
                logger.error("LED discovery error: %s", e, exc_info=True)

        logger.debug("LED discovery thread stopped")

    def _quick_probe_existing_buses(self, missing_addrs: set) -> set:
        """Quick-probe *missing_addrs* on already-open buses (20 ms timeout).

        Returns the set of addresses that were found.
        """
        found: set = set()
        with self._lock:
            buses = list(self._buses.items())

        for addr in sorted(missing_addrs):
            if self._stop_event.is_set():
                break
            for port, bus in buses:
                if bus.probe_controller(addr, timeout_s=_PROBE_QUICK_TIMEOUT_S):
                    with self._lock:
                        self._register_controller(addr, port)
                    found.add(addr)
                    logger.info(
                        "Quick-probe found controller %d on %s", addr, port,
                    )
                    break  # found on this bus, no need to check others
        return found

    def _scan_new_ports(self, missing_addrs: set) -> None:
        """Open any new USB-RS485 ports and full-probe for *missing_addrs*."""
        with self._lock:
            managed_ports = set(self._buses.keys())

        exclude = (
            self._exclude_ports | managed_ports | port_registry.get_claimed_ports()
        )
        candidates = RS485Connection.find_led_rs485_ports(exclude_ports=exclude)

        for port in candidates:
            if self._stop_event.is_set():
                return
            if not port_registry.acquire_port(port, owner="led"):
                continue
            opened = self._open_and_probe_port(
                port, missing_addrs, _PROBE_TIMEOUT_S,
            )
            if not opened:
                # No controllers — release the port for other subsystems.
                port_registry.release_port(port)

    def _cleanup_disconnected(self) -> None:
        """Remove buses whose serial ports are no longer open."""
        with self._lock:
            dead_ports = [
                p for p, b in self._buses.items() if not b.is_open
            ]
        for port in dead_ports:
            with self._lock:
                bus = self._buses.pop(port, None)
                self._unregister_port(port)
            if bus:
                bus.stop()
                port_registry.release_port(port)
                logger.info("Removed disconnected LED bus on %s", port)

    def _cleanup_empty_buses(self) -> None:
        """Close buses that have zero controllers mapped."""
        with self._lock:
            empty_ports = [
                p for p, b in self._buses.items()
                if not b.discovered_controllers
            ]
        for port in empty_ports:
            with self._lock:
                bus = self._buses.pop(port, None)
                self._unregister_port(port)
            if bus:
                bus.stop()
                port_registry.release_port(port)
                logger.info("Closed empty LED bus on %s", port)


# ─────────────────────────────────────────────────────────────────────────────
# Standalone test: discovery + blink
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    print("=" * 60)
    print("LED RS485 Auto-Discovery Test")
    print("=" * 60)

    # --- Step 1: Enumerate USB serial devices ---
    print("\n--- USB Serial Devices ---")
    if serial:
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("  (none)")
        for p in sorted(ports, key=lambda x: x.device):
            vid_pid = ""
            if p.vid is not None and p.pid is not None:
                vid_pid = f"  VID:PID 0x{p.vid:04X}:0x{p.pid:04X}"
            print(f"  {p.device:<10}{vid_pid}  {p.description}")
    else:
        print("  pyserial not installed!")

    # --- Step 2: Filter for LED RS485 candidates ---
    print("\n--- LED RS485 Adapter Candidates ---")
    candidates = RS485Connection.find_led_rs485_ports()
    if not candidates:
        print("  No matching adapters found. Check USB connections.")
        print("  (Looked for VID/PIDs:", _RS485_LED_VID_PIDS, ")")
        raise SystemExit(1)
    for port in candidates:
        print(f"  {port}")

    # --- Step 3: Start LEDSystem with auto-discovery ---
    print("\n--- Starting LEDSystem (auto-discover) ---")
    system = LEDSystem(auto_discover=True, debug_hex=True)
    ok = system.start()
    print(f"  start() returned: {ok}")

    bus_info = system.get_bus_info()
    mapping = system.get_discovered_mapping()
    all_strip_ids = sorted(system.strips.keys())

    print("\n--- Discovery Results ---")
    if not bus_info:
        print("  No buses connected.")
    for port, addrs in sorted(bus_info.items()):
        print(f"  Bus {port}: controllers {sorted(addrs)}")
    print()
    for sid in all_strip_ids:
        port = mapping.get(sid)
        status = port if port else "(not found)"
        print(f"  Strip {sid} → {status}")

    discovered_strips = sorted(mapping.keys())
    if not discovered_strips:
        print("\nNo strips discovered — nothing to blink. Stopping.")
        system.stop()
        raise SystemExit(1)

    # --- Step 4: Blink test on discovered strips ---
    print(f"\n--- Blink Test (strips: {discovered_strips}) ---")
    BLINK_CYCLES = 3
    BLINK_ON_S = 0.5
    BLINK_OFF_S = 0.5

    import time as _time
    for cycle in range(BLINK_CYCLES):
        color = [RED, GREEN, BLUE][cycle % 3]
        color_name = ["RED", "GREEN", "BLUE"][cycle % 3]
        print(f"  Cycle {cycle + 1}/{BLINK_CYCLES}: {color_name} ON", flush=True)
        for sid in discovered_strips:
            system.set_strip_color(sid, color)
        _time.sleep(BLINK_ON_S)

        print(f"  Cycle {cycle + 1}/{BLINK_CYCLES}: OFF", flush=True)
        for sid in discovered_strips:
            system.set_strip_color(sid, OFF)
        _time.sleep(BLINK_OFF_S)

    print("\n--- Test Complete ---")
    system.stop()
    print("LEDSystem stopped.")
