"""Low-level RS485 serial transport for the arena light columns.

This layer is deliberately thin and (almost) stateless. The *only* state it
owns is the open serial handles - one per COM port - which is unavoidable for
a hardware resource. It holds **no** LED color memory, **no** animation, and
**no** send-pacing logic; those live in the high-level
:class:`~subsystems.light_column.controller.LedColumnController`.

Responsibilities:

* :func:`build_strip_frame` - pure function turning a strip ID + color list
  into the exact bytes the WS2811 controller firmware expects.
* :class:`LedTransport` - open/close the three configured COM ports, route a
  strip ID to its owning port, and write a frame (send-and-forget into the OS
  serial buffer).

The frame format and serial line settings are carried over unchanged from the
previously validated implementation - that part of the old code was correct.
"""

from __future__ import annotations

import logging
from typing import Sequence

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - exercised only without pyserial
    serial = None

from core.device_connection import require_serial_baudrate
from subsystems.light_column.frames import Color
from subsystems.light_column.layout import LightColumnLayout

logger = logging.getLogger(__name__)

# Config key that owns the LED RS485 transport settings in the device file.
SERIAL_SETTINGS_KEY = "light_columns"

# WS2811 display-command framing (validated vendor protocol).
_HEADER = bytes([0xDD, 0x55, 0xEE])
_TAIL = bytes([0xAA, 0xBB])
_FUNCTION_DISPLAY = 0x99
_LED_TYPE_WS2811 = 0x02
_LED_TYPE_RESERVED = bytes([0x00, 0x00])
_REPEAT = bytes([0x00, 0x01])

# Serial line settings. Send-and-forget: a short write timeout keeps a stuck
# port from stalling the high-frequency render loop for a full second.
_READ_TIMEOUT_S = 0.05
_WRITE_TIMEOUT_S = 0.1


def _u16(value: int) -> bytes:
    """Serialize an unsigned 16-bit integer in big-endian order."""

    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def build_strip_frame(strip_id: int, colors: Sequence[Color]) -> bytes:
    """Build the RS485 display command for one strip.

    Args:
        strip_id: Logical strip ID ``address * 10 + channel`` (e.g. ``42``).
        colors: One :class:`Color` per LED, in bottom-to-top order. Serialized
            in RGB wire order (red, green, blue), matching the firmware.

    Returns:
        The complete command frame as bytes, ready for :meth:`LedTransport.write`.
    """

    device_addr = strip_id // 10
    channel = strip_id % 10
    color_data = b"".join(
        bytes([c.r & 0xFF, c.g & 0xFF, c.b & 0xFF]) for c in colors
    )
    return (
        _HEADER
        + _u16(0)  # group broadcast address
        + _u16(device_addr)
        + bytes([channel])
        + bytes([_FUNCTION_DISPLAY])
        + bytes([_LED_TYPE_WS2811])
        + _LED_TYPE_RESERVED
        + _u16(len(color_data))
        + _REPEAT
        + color_data
        + _TAIL
    )


class LedTransport:
    """Own the three RS485 serial ports and route strip frames to them.

    The mapping from strip ID -> COM port is derived once from the layout's
    ``controller_addresses_by_port``. Opening is best-effort: a port that fails
    to open is logged and skipped so a single unplugged bus does not take down
    the whole process; frames for that port's strips are dropped until it is
    reopened.
    """

    def __init__(self, layout: LightColumnLayout) -> None:
        self._layout = layout
        # strip_id -> port name, and port name -> ordered strip IDs it drives.
        self._port_for_strip: dict[int, str] = {}
        self._strips_for_port: dict[str, tuple[int, ...]] = {}
        for port, addresses in layout.controller_addresses_by_port.items():
            strips: list[int] = []
            for address in addresses:
                for channel in (1, 2):
                    strip_id = address * 10 + channel
                    strips.append(strip_id)
                    self._port_for_strip[strip_id] = port
            self._strips_for_port[port] = tuple(strips)
        self._ports: tuple[str, ...] = tuple(layout.serial_ports)
        # port name -> open serial handle (only successfully opened ports).
        self._serials: dict[str, object] = {}
        self._baudrate = require_serial_baudrate(SERIAL_SETTINGS_KEY)

    def open(self) -> bool:
        """Open every configured COM port; return True if at least one opened.

        Best-effort: failures are logged, not raised, and the port is omitted
        from :meth:`ports`.
        """

        if serial is None:
            logger.error("pyserial is not installed; LED transport cannot open ports")
            return False
        for port in self._ports:
            if port in self._serials:
                continue
            try:
                handle = serial.Serial(
                    port=port,
                    baudrate=self._baudrate,
                    timeout=_READ_TIMEOUT_S,
                    write_timeout=_WRITE_TIMEOUT_S,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                handle.reset_input_buffer()
                handle.reset_output_buffer()
                self._serials[port] = handle
                logger.info(
                    "LED bus %s open (baud=%d, strips=%s)",
                    port,
                    self._baudrate,
                    list(self._strips_for_port.get(port, ())),
                )
            except Exception as exc:  # noqa: BLE001 - report and continue
                logger.error("Failed to open LED bus %s: %s", port, exc)
        return bool(self._serials)

    def close(self) -> None:
        """Close every open serial port. Latched LED colors are left as-is."""

        for port, handle in self._serials.items():
            try:
                handle.close()  # type: ignore[attr-defined]
                logger.info("LED bus %s closed", port)
            except Exception as exc:  # noqa: BLE001 - best effort on shutdown
                logger.error("Error closing LED bus %s: %s", port, exc)
        self._serials.clear()

    def ports(self) -> tuple[str, ...]:
        """Return the COM ports that are currently open and writable."""

        return tuple(port for port in self._ports if port in self._serials)

    def strips_for_port(self, port: str) -> tuple[int, ...]:
        """Return the ordered strip IDs driven by ``port``."""

        return self._strips_for_port.get(port, ())

    def port_for_strip(self, strip_id: int) -> str | None:
        """Return the COM port that drives ``strip_id`` (None if unmapped)."""

        return self._port_for_strip.get(strip_id)

    def write(self, port: str, frame: bytes) -> bool:
        """Write one frame to ``port`` and return immediately (send-and-forget).

        The bytes are handed to the OS/driver serial buffer; we deliberately do
        not flush so the render loop never blocks waiting for the line to drain.
        Returns False if the port is not open or the write raised.
        """

        handle = self._serials.get(port)
        if handle is None:
            return False
        try:
            handle.write(frame)  # type: ignore[attr-defined]
            return True
        except Exception as exc:  # noqa: BLE001 - drop frame, keep looping
            logger.error("LED write failed on %s: %s", port, exc)
            return False
