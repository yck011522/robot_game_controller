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

logger = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

_PROTOCOL_HEADER = bytes([0xDD, 0x55, 0xEE])
_PROTOCOL_TAIL = bytes([0xAA, 0xBB])
_GROUP_BROADCAST = bytes([0x00, 0x00])
_FUNCTION_DISPLAY = 0x99
_LED_TYPE_WS2811 = 0x02
_LED_TYPE_RESERVED = bytes([0x00, 0x00])
_LED_LENGTH = 0x54  # 84 bytes = 28 LEDs × 3 RGB
_LED_REPEAT = bytes([0x00, 0x01])
_LEDS_PER_STRIP = 28
_DEFAULT_BAUDRATE = 921600
# Validated at 921600 baud: 2 ms is reliable, 1 ms produces corrupted colors.
_RECOMMENDED_MIN_INTER_COMMAND_S = 0.002
_DEFAULT_INTER_COMMAND_S = 0.002

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

    @staticmethod
    def _auto_detect_port() -> Optional[str]:
        """Auto-detect RS485 adapter port (heuristic: USB serial device)."""
        if not serial:
            return None

        for port_info in serial.tools.list_ports.comports():
            # Look for USB devices with common RS485 adapter VIDs
            if port_info.vid and port_info.pid:
                # CH340, CH341, FT232, etc. are common RS485 USB adapters
                return port_info.device

        # Fallback: return first USB serial device
        for port_info in serial.tools.list_ports.comports():
            if "USB" in port_info.description:
                return port_info.device

        return None


# ─────────────────────────────────────────────────────────────────────────────
# LED System Manager
# ─────────────────────────────────────────────────────────────────────────────

class LEDSystem:
    """High-level LED strip manager with thread-based command sending."""

    def __init__(
        self,
        serial_port: Optional[str] = None,
        baudrate: int = _DEFAULT_BAUDRATE,
        inter_command_delay_s: float = _DEFAULT_INTER_COMMAND_S,
        debug_hex: bool = False,
    ):
        """
        Args:
            serial_port: RS485 port (auto-detected if None)
            baudrate: RS485 baud rate
            inter_command_delay_s: Delay between RS485 frames (helps controller parsing)
            debug_hex: If True, logs outgoing frame hex
        """
        self.conn = RS485Connection(serial_port, baudrate)
        self._inter_command_delay_s = max(0.0, inter_command_delay_s)
        self._debug_hex = debug_hex
        if 0 < self._inter_command_delay_s < _RECOMMENDED_MIN_INTER_COMMAND_S:
            logger.warning(
                "inter_command_delay_s=%.4f is below recommended %.4f at 921600 baud; "
                "frames may corrupt or be ignored",
                self._inter_command_delay_s,
                _RECOMMENDED_MIN_INTER_COMMAND_S,
            )
        self.strips: Dict[int, LEDStrip] = {
            sid: LEDStrip(sid) for sid in _STRIP_TO_CONTROLLER.keys()
        }

        self._command_queue: deque[bytes] = deque()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def start(self) -> bool:
        """Open connection and start sender thread. Return True if successful."""
        if not self.conn.open():
            return False

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._sender_loop,
            name="led-sender",
            daemon=True,
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stop sender thread and close connection."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.conn.close()

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

    def _queue_strip_command(self, strip_id: int) -> None:
        """Queue a command to send for a strip."""
        with self._lock:
            cmd = self.strips[strip_id].get_command()
            self._command_queue.append(cmd)

    def _sender_loop(self) -> None:
        """Sender thread: dequeue and send commands at ~50 Hz.

        Commands are spaced by a small inter-frame delay because some controllers
        parse more reliably with short gaps between packets.
        """
        hz = 50
        interval = 1.0 / hz

        while not self._stop_event.is_set():
            cmd = None
            with self._lock:
                if self._command_queue:
                    cmd = self._command_queue.popleft()

            if cmd is None:
                self._stop_event.wait(interval)
                continue

            if self._debug_hex:
                logger.info("LED TX (%d bytes): %s", len(cmd), cmd.hex(" "))

            self.conn.send(cmd)

            if self._inter_command_delay_s > 0:
                self._stop_event.wait(self._inter_command_delay_s)
