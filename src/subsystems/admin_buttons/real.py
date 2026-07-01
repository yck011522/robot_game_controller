"""Real Modbus RTU driver for the HY-IO4400S-4NN admin-button unit."""

from __future__ import annotations

import inspect
from typing import Any, Callable

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

from subsystems.admin_buttons.common import AdminButtonConfig


def create_client(port: str, baudrate: int, timeout_s: float) -> ModbusSerialClient:
    """Create a synchronous Modbus RTU client for the admin-button bus."""

    return ModbusSerialClient(
        port=port,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=timeout_s,
        retries=0,
    )


class RealAdminButtonUnit:
    """Low-level Modbus client for HY-IO4400S-4NN digital inputs and relay coil."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        timeout_s: float,
        config: AdminButtonConfig,
        client_factory: Callable[[str, int, float], Any] = create_client,
    ) -> None:
        self.port = port  # COM port for the HY-IO4400S-4NN RS-485 adapter.
        self.baudrate = int(baudrate)  # Serial speed configured in device_ports_and_addr.yaml.
        self.timeout_s = float(timeout_s)  # Per-Modbus request timeout.
        self.config = config  # Slave id, input mapping, lamp coil, and cooldown.
        self._client = client_factory(self.port, self.baudrate, self.timeout_s)
        self._connected = False

    def connect(self) -> None:
        """Open the serial port or raise if the adapter cannot be reached."""

        self._connected = bool(self._client.connect())
        if not self._connected:
            raise RuntimeError(f"could not open admin button Modbus port {self.port}")

    def read_inputs(self) -> tuple[list[bool], list[str]]:
        """Read all configured digital inputs via Modbus function 02."""

        try:
            response = self._call_modbus(
                self._client.read_discrete_inputs,
                address=self.config.input_start_address,
                count=self.config.input_count,
                slave_address=self.config.slave_address,
            )
        except (ModbusIOException, OSError) as exc:
            return [], [f"read_exception:{exc}"]
        is_error = getattr(response, "isError", None)
        if callable(is_error) and is_error():
            return [], ["read_modbus_error"]
        bits = getattr(response, "bits", None)
        if not isinstance(bits, list) or len(bits) < self.config.input_count:
            return [], ["read_short_response"]
        return [bool(value) for value in bits[: self.config.input_count]], []

    def write_resume_lamp(self, on: bool) -> list[str]:
        """Write relay output 1 via Modbus function 05 single-coil write."""

        try:
            response = self._call_modbus(
                self._client.write_coil,
                address=self.config.resume_lamp_coil_address,
                value=bool(on),
                slave_address=self.config.slave_address,
            )
        except (ModbusIOException, OSError) as exc:
            return [f"lamp_write_exception:{exc}"]
        is_error = getattr(response, "isError", None)
        if callable(is_error) and is_error():
            return ["lamp_write_modbus_error"]
        return []

    def close(self) -> None:
        """Close the serial client."""

        self._client.close()
        self._connected = False

    def _call_modbus(self, fn, *, address: int, slave_address: int, **kwargs):
        """Call a pymodbus method across ``slave``/``unit``/``device_id`` APIs."""

        params = inspect.signature(fn).parameters
        call_kwargs = {"address": address, **kwargs}
        if "slave" in params:
            call_kwargs["slave"] = slave_address
            return fn(**call_kwargs)
        if "unit" in params:
            call_kwargs["unit"] = slave_address
            return fn(**call_kwargs)
        if "device_id" in params:
            call_kwargs["device_id"] = slave_address
            return fn(**call_kwargs)
        return fn(address, *kwargs.values(), slave_address)

