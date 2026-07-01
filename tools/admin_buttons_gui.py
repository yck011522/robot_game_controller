"""Small GUI probe for the HY-IO4400S-4NN admin button I/O unit.

Run directly:
    $env:PYTHONPATH = "src"
    python tools/admin_buttons_gui.py

Run with the validated deployment interpreter:
    $env:PYTHONPATH = "src"
    & C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe tools\\admin_buttons_gui.py

Typical workflow:
    1. Choose COM44 from the COM port dropdown.
    2. Leave baud at 115200 and slave address at 1.
    3. Click Connect.
    4. Watch DI1..DI4 update live.
    5. Toggle DO1..DO4 checkboxes to write relay coils 0..3.

Assumptions for this probe:
    - Digital inputs are read with Modbus function 02, start address 0, count 4.
    - Relay outputs are written with Modbus function 05, coil addresses 0..3.
    - The device is configured as 8N1, no parity, one stop bit.
"""

from __future__ import annotations

import inspect
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Any, Callable

try:
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - only happens when requirements are absent.
    list_ports = None

from pymodbus.client import ModbusSerialClient


REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.device_connection import (  # noqa: E402
    load_serial_settings,
    require_serial_baudrate,
    require_serial_int,
    resolve_serial_ports,
)


SERIAL_PORTS_KEY = "admin_buttons"
SERIAL_SETTINGS_KEY = "admin_buttons"
DEFAULT_TIMEOUT_S = 0.08
DEFAULT_POLL_MS = 100
INPUT_START_ADDRESS = 0
INPUT_COUNT = 4
COIL_START_ADDRESS = 0
COIL_COUNT = 4


def configured_default_port() -> str:
    """Return the configured admin-buttons COM port, or a friendly fallback."""

    try:
        ports = resolve_serial_ports(SERIAL_PORTS_KEY).ports
    except Exception:
        return "COM44"
    return ports[0] if ports else "COM44"


def configured_default_baudrate() -> int:
    """Return the configured admin-buttons baud rate, or the expected default."""

    try:
        return require_serial_baudrate(SERIAL_SETTINGS_KEY)
    except Exception:
        return 115200


def configured_default_slave_address() -> int:
    """Return the configured admin-buttons Modbus slave address."""

    try:
        return require_serial_int(SERIAL_SETTINGS_KEY, "slave_address", min_value=1)
    except Exception:
        return 1


def available_serial_ports(default_port: str) -> list[str]:
    """Return visible serial ports, keeping the configured default selectable."""

    ports: list[str] = []
    if list_ports is not None:
        ports = [port.device for port in list_ports.comports()]
    if default_port and default_port not in ports:
        ports.insert(0, default_port)
    return ports


def configured_default_timeout_s() -> float:
    """Return the configured read timeout when present."""

    try:
        settings = load_serial_settings().get(SERIAL_SETTINGS_KEY, {})
        return float(settings.get("read_timeout_s", DEFAULT_TIMEOUT_S))
    except Exception:
        return DEFAULT_TIMEOUT_S


class ModbusIoClient:
    """Tiny synchronous Modbus RTU helper for four DI bits and four coils."""

    def __init__(self, *, port: str, baudrate: int, slave_address: int, timeout_s: float) -> None:
        self.port = port  # Selected COM port, usually COM44.
        self.baudrate = int(baudrate)  # Serial speed for the HY-IO4400S-4NN.
        self.slave_address = int(slave_address)  # Modbus slave id, configured as 1.
        self.timeout_s = float(timeout_s)  # Max seconds a Modbus call may block the UI.
        self.client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout_s,
            retries=0,
        )

    def connect(self) -> None:
        """Open the serial client or raise when the adapter cannot be opened."""

        if not self.client.connect():
            raise RuntimeError(f"could not open {self.port}")

    def close(self) -> None:
        """Close the serial client."""

        self.client.close()

    def read_inputs(self) -> list[bool]:
        """Read DI1..DI4 using Modbus function 02."""

        response = self._call_modbus(
            self.client.read_discrete_inputs,
            address=INPUT_START_ADDRESS,
            count=INPUT_COUNT,
        )
        is_error = getattr(response, "isError", None)
        if callable(is_error) and is_error():
            raise RuntimeError("read_discrete_inputs returned Modbus error")
        bits = getattr(response, "bits", None)
        if not isinstance(bits, list) or len(bits) < INPUT_COUNT:
            raise RuntimeError("short input response")
        return [bool(value) for value in bits[:INPUT_COUNT]]

    def write_coil(self, coil_index: int, on: bool) -> None:
        """Write one relay output coil using Modbus function 05."""

        coil_address = COIL_START_ADDRESS + int(coil_index)
        response = self._call_modbus(
            self.client.write_coil,
            address=coil_address,
            value=bool(on),
        )
        is_error = getattr(response, "isError", None)
        if callable(is_error) and is_error():
            raise RuntimeError(f"write_coil({coil_address}) returned Modbus error")

    def _call_modbus(self, fn: Callable[..., Any], *, address: int, **kwargs: Any) -> Any:
        """Call a pymodbus method across ``slave``/``unit``/``device_id`` APIs."""

        params = inspect.signature(fn).parameters
        call_kwargs = {"address": address, **kwargs}
        if "slave" in params:
            call_kwargs["slave"] = self.slave_address
            return fn(**call_kwargs)
        if "unit" in params:
            call_kwargs["unit"] = self.slave_address
            return fn(**call_kwargs)
        if "device_id" in params:
            call_kwargs["device_id"] = self.slave_address
            return fn(**call_kwargs)
        return fn(address, *kwargs.values(), self.slave_address)


class AdminButtonsProbeApp:
    """Tkinter GUI for live DI polling and relay coil toggling."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root  # Top-level Tk window.
        self.root.title("HY-IO4400S-4NN Admin Button I/O Probe")
        self.client: ModbusIoClient | None = None  # Active Modbus client; None means disconnected.
        self.poll_after_id: str | None = None  # Tk ``after`` handle for the polling loop.
        self.default_port = configured_default_port()  # Config default, normally COM44.
        self.port_var = tk.StringVar(value=self.default_port)  # Selected COM port.
        self.baud_var = tk.StringVar(value=str(configured_default_baudrate()))  # Selected baud rate.
        self.slave_var = tk.StringVar(value=str(configured_default_slave_address()))  # Modbus slave id.
        self.timeout_var = tk.StringVar(value=f"{configured_default_timeout_s():.3f}")  # Modbus timeout seconds.
        self.poll_ms_var = tk.StringVar(value=str(DEFAULT_POLL_MS))  # UI polling period in milliseconds.
        self.status_var = tk.StringVar(value="Disconnected")  # Bottom status line.
        self.input_vars = [tk.StringVar(value="DI%d: ?" % (i + 1)) for i in range(INPUT_COUNT)]  # DI labels.
        self.input_colors = ["#5b6470"] * INPUT_COUNT  # Per-DI indicator colors.
        self.coil_vars = [tk.BooleanVar(value=False) for _ in range(COIL_COUNT)]  # DO checkbox states.
        self.coil_write_busy = False  # Prevents recursive checkbox writes when reverting a failed toggle.
        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_layout(self) -> None:
        """Build all widgets for connection, input display, and coil toggles."""

        frame = ttk.Frame(self.root, padding=14)
        frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        connection = ttk.LabelFrame(frame, text="Connection", padding=10)
        connection.grid(row=0, column=0, sticky="ew")
        connection.columnconfigure(1, weight=1)

        ttk.Label(connection, text="COM port").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)
        self.port_combo = ttk.Combobox(
            connection,
            textvariable=self.port_var,
            values=available_serial_ports(self.default_port),
            width=18,
        )
        self.port_combo.grid(row=0, column=1, sticky="ew", pady=4)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(8, 0), pady=4)

        ttk.Label(connection, text="Baud").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(connection, textvariable=self.baud_var, width=12).grid(row=1, column=1, sticky="w", pady=4)

        ttk.Label(connection, text="Address").grid(row=2, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(connection, textvariable=self.slave_var, width=12).grid(row=2, column=1, sticky="w", pady=4)

        ttk.Label(connection, text="Timeout s").grid(row=3, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(connection, textvariable=self.timeout_var, width=12).grid(row=3, column=1, sticky="w", pady=4)

        ttk.Label(connection, text="Poll ms").grid(row=4, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(connection, textvariable=self.poll_ms_var, width=12).grid(row=4, column=1, sticky="w", pady=4)

        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(10, 0))

        inputs = ttk.LabelFrame(frame, text="Digital Inputs", padding=10)
        inputs.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        self.input_canvases: list[tk.Canvas] = []
        for index in range(INPUT_COUNT):
            row = index // 2
            col = (index % 2) * 2
            canvas = tk.Canvas(inputs, width=24, height=24, highlightthickness=0)
            canvas.grid(row=row, column=col, padx=(0, 8), pady=6)
            canvas.create_oval(3, 3, 21, 21, fill=self.input_colors[index], outline="")
            self.input_canvases.append(canvas)
            ttk.Label(inputs, textvariable=self.input_vars[index], width=16).grid(
                row=row,
                column=col + 1,
                sticky="w",
                pady=6,
            )

        coils = ttk.LabelFrame(frame, text="Relay Coils", padding=10)
        coils.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        for index in range(COIL_COUNT):
            check = ttk.Checkbutton(
                coils,
                text=f"DO{index + 1} / coil {index}",
                variable=self.coil_vars[index],
                command=lambda i=index: self.on_coil_toggle(i),
            )
            check.grid(row=index // 2, column=index % 2, sticky="w", padx=(0, 18), pady=6)

        ttk.Label(frame, textvariable=self.status_var).grid(row=3, column=0, sticky="ew", pady=(12, 0))

    def refresh_ports(self) -> None:
        """Refresh the COM dropdown from pyserial's current port list."""

        ports = available_serial_ports(self.port_var.get() or self.default_port)
        self.port_combo.configure(values=ports)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        """Connect or disconnect depending on the current state."""

        if self.client is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self) -> None:
        """Open the selected Modbus serial connection and start polling."""

        try:
            client = ModbusIoClient(
                port=self.port_var.get().strip(),
                baudrate=int(self.baud_var.get()),
                slave_address=int(self.slave_var.get()),
                timeout_s=float(self.timeout_var.get()),
            )
            client.connect()
        except Exception as exc:  # noqa: BLE001 - GUI reports any connection failure.
            self.status_var.set(f"Connect failed: {exc}")
            return
        self.client = client
        self.connect_button.configure(text="Disconnect")
        self.status_var.set(f"Connected to {client.port} at {client.baudrate} baud")
        self.schedule_poll(delay_ms=1)

    def disconnect(self) -> None:
        """Stop polling, close the serial client, and update the UI."""

        if self.poll_after_id is not None:
            self.root.after_cancel(self.poll_after_id)
            self.poll_after_id = None
        if self.client is not None:
            self.client.close()
            self.client = None
        self.connect_button.configure(text="Connect")
        self.status_var.set("Disconnected")

    def schedule_poll(self, *, delay_ms: int | None = None) -> None:
        """Schedule the next input poll using Tk's event loop."""

        try:
            poll_ms = max(20, int(float(self.poll_ms_var.get())))
        except (TypeError, ValueError):
            poll_ms = DEFAULT_POLL_MS
        self.poll_after_id = self.root.after(delay_ms if delay_ms is not None else poll_ms, self.poll_once)

    def poll_once(self) -> None:
        """Read DI1..DI4 once and refresh the indicator widgets."""

        self.poll_after_id = None
        if self.client is None:
            return
        started_t = time.perf_counter()
        try:
            values = self.client.read_inputs()
            elapsed_ms = (time.perf_counter() - started_t) * 1000.0
            for index, value in enumerate(values[:INPUT_COUNT]):
                self.input_vars[index].set(f"DI{index + 1}: {'HIGH' if value else 'LOW'}")
                self.set_input_color(index, "#24a148" if value else "#5b6470")
            self.status_var.set(f"Polling OK ({elapsed_ms:.1f} ms)")
        except Exception as exc:  # noqa: BLE001 - GUI reports any poll failure.
            self.status_var.set(f"Poll failed: {exc}")
            for index in range(INPUT_COUNT):
                self.input_vars[index].set(f"DI{index + 1}: ?")
                self.set_input_color(index, "#b42318")
        self.schedule_poll()

    def set_input_color(self, index: int, color: str) -> None:
        """Paint one digital-input indicator."""

        canvas = self.input_canvases[index]
        canvas.delete("all")
        canvas.create_oval(3, 3, 21, 21, fill=color, outline="")

    def on_coil_toggle(self, index: int) -> None:
        """Write the selected checkbox state to one relay output coil."""

        if self.coil_write_busy:
            return
        if self.client is None:
            self.status_var.set("Connect before toggling relay coils")
            self.revert_coil(index)
            return
        desired = bool(self.coil_vars[index].get())
        try:
            self.client.write_coil(index, desired)
            self.status_var.set(f"DO{index + 1} coil {index} -> {'ON' if desired else 'OFF'}")
        except Exception as exc:  # noqa: BLE001 - GUI reports any coil write failure.
            self.status_var.set(f"DO{index + 1} write failed: {exc}")
            self.revert_coil(index)

    def revert_coil(self, index: int) -> None:
        """Undo a checkbox change without recursively writing the coil."""

        self.coil_write_busy = True
        try:
            self.coil_vars[index].set(not bool(self.coil_vars[index].get()))
        finally:
            self.coil_write_busy = False

    def close(self) -> None:
        """Disconnect and close the Tk window."""

        self.disconnect()
        self.root.destroy()


def main() -> int:
    """Create the Tk app and run until the window closes."""

    root = tk.Tk()
    AdminButtonsProbeApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

