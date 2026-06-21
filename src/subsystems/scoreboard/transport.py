"""Low-level RS485/USB-serial transport for the LED scoreboard panels.

This layer is intentionally thin. It owns the single open serial handle for the
daisy-chained AtomS3 panel string (one COM port) and exposes pure helpers that
build the newline-terminated text commands the panel firmware understands.

It holds **no** display memory, **no** stage logic, and **no** send pacing;
those live in :class:`~subsystems.scoreboard.controller.ScoreboardController`.

Command syntax (verified on the hardware over RS485 at 115200 baud):

    /display/<N>/text/enable 1\\n     enable the text layer on panel N
    /display/<N>/text/enable 0\\n     disable (blank) panel N
    /display/<N>/mode 0\\n            static text
    /display/<N>/mode 1\\n            scroll-up text (commas split into lines)
    /display/<N>/text/stack "TEXT"\\n set panel N's text (quotes are literal)
    /display/<N>/color R G B\\n       text colour (each 0-255)
    /display/<N>/brightness V\\n      whole-display brightness (0-255)
    /display/<N>/text/brightness V\\n text-layer brightness (0-255)
"""

from __future__ import annotations

import logging

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - exercised only without pyserial
    serial = None

from core.device_connection import require_serial_baudrate
from subsystems.scoreboard.layout import ScoreboardLayout

logger = logging.getLogger(__name__)

# Config key that owns the scoreboard serial settings in the device file
# (``serial_settings.scoreboard.baudrate``).
SERIAL_SETTINGS_KEY = "scoreboard"

# Display "mode" argument values understood by the panel firmware.
MODE_STATIC = 0  # show text in place (used for scores, idle, tutorial, reveal)
MODE_SCROLL_UP = 1  # scroll comma-separated lines upward (used for GAME,OVER)

# Serial line settings. Send-and-forget with a short write timeout so a stuck
# port never stalls the render loop for a full second.
_READ_TIMEOUT_S = 0.05
_WRITE_TIMEOUT_S = 0.1


def cmd_enable(display: int, on: bool) -> bytes:
    """Build the text-layer enable/disable command for one panel."""

    return f"/display/{display}/text/enable {1 if on else 0}\n".encode("ascii")


def cmd_mode(display: int, mode: int) -> bytes:
    """Build the display-mode command (``MODE_STATIC`` or ``MODE_SCROLL_UP``)."""

    return f"/display/{display}/mode {int(mode)}\n".encode("ascii")


def cmd_text(display: int, text: str) -> bytes:
    """Build the set-text command for one panel.

    The firmware expects the payload wrapped in literal double quotes, e.g.
    ``/display/2/text/stack "0000"``. ``text`` is sent verbatim (ASCII), so any
    embedded comma is interpreted by the firmware as a line break in scroll mode.
    """

    return f'/display/{display}/text/stack "{text}"\n'.encode("ascii")


def cmd_color(display: int, red: int, green: int, blue: int) -> bytes:
    """Build the text-colour command for one panel.

    ``red``/``green``/``blue`` are clamped to 0..255. The firmware applies this
    to the rendered text layer (``/display/<N>/color R G B``).
    """

    r = max(0, min(255, int(red)))
    g = max(0, min(255, int(green)))
    b = max(0, min(255, int(blue)))
    return f"/display/{display}/color {r} {g} {b}\n".encode("ascii")


def cmd_brightness(display: int, value: int) -> bytes:
    """Build the whole-display brightness command (``0..255``, clamped)."""

    level = max(0, min(255, int(value)))
    return f"/display/{display}/brightness {level}\n".encode("ascii")


def cmd_text_brightness(display: int, value: int) -> bytes:
    """Build the text-layer brightness command (``0..255``, clamped)."""

    level = max(0, min(255, int(value)))
    return f"/display/{display}/text/brightness {level}\n".encode("ascii")


class ScoreboardTransport:
    """Own the single scoreboard COM port and write text command lines to it.

    Opening is best-effort: if the port fails to open it is logged and skipped
    so an unplugged scoreboard does not take down the process; writes are then
    dropped until the port is reopened.
    """

    def __init__(self, layout: ScoreboardLayout) -> None:
        self._port = layout.port
        self._baudrate = require_serial_baudrate(SERIAL_SETTINGS_KEY)
        # Open serial handle, or None while the port is closed/unavailable.
        self._serial: object | None = None

    def open(self) -> bool:
        """Open the configured COM port; return True on success."""

        if serial is None:
            logger.error("pyserial is not installed; scoreboard transport cannot open")
            return False
        if self._serial is not None:
            return True
        try:
            handle = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=_READ_TIMEOUT_S,
                write_timeout=_WRITE_TIMEOUT_S,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            handle.reset_input_buffer()
            handle.reset_output_buffer()
            self._serial = handle
            logger.info("Scoreboard bus %s open (baud=%d)", self._port, self._baudrate)
            return True
        except Exception as exc:  # noqa: BLE001 - report and continue
            logger.error("Failed to open scoreboard bus %s: %s", self._port, exc)
            return False

    def close(self) -> None:
        """Close the serial port. Panel contents are left as-is (NVS-backed)."""

        if self._serial is None:
            return
        try:
            self._serial.close()  # type: ignore[attr-defined]
            logger.info("Scoreboard bus %s closed", self._port)
        except Exception as exc:  # noqa: BLE001 - best effort on shutdown
            logger.error("Error closing scoreboard bus %s: %s", self._port, exc)
        self._serial = None

    def is_open(self) -> bool:
        """Return True while the COM port is open and writable."""

        return self._serial is not None

    def write(self, line: bytes) -> bool:
        """Write one command line and return immediately (send-and-forget).

        Returns False if the port is not open or the write raised; the caller
        treats a dropped line as transient and will re-send on the next change.
        """

        handle = self._serial
        if handle is None:
            return False
        try:
            handle.write(line)  # type: ignore[attr-defined]
            return True
        except Exception as exc:  # noqa: BLE001 - drop line, keep looping
            logger.error("Scoreboard write failed on %s: %s", self._port, exc)
            return False
