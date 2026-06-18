"""Real Modbus RTU bucket motor driver.

The bucket process owns the RS-485 USB adapter exclusively. Higher-level
code should call this driver with logical labels (``A1`` .. ``B3``); this
module translates those labels to Modbus device addresses and register writes.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

from subsystems.bucket.common import BucketMotorConfig, Direction, MotorStatus


REG_MOTOR_CONTROL = 0x0000
CMD_STOP = 0x00
STATUS_STOPPED = {0x00, 0x80}
STATUS_POSITIVE_LIMIT = 0x10
STATUS_NEGATIVE_LIMIT = 0x90


def create_client(port: str, baudrate: int, timeout_s: float) -> ModbusSerialClient:
    """Create a synchronous Modbus RTU client for the bucket motor bus."""

    return ModbusSerialClient(
        port=port,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=timeout_s,
        retries=0,
    )


def clamp_speed(speed: int) -> int:
    """Clamp speed to the motor controller's supported 1..15 range."""

    return max(0x01, min(0x0F, int(speed)))


def encode_motion_command(direction: Direction, speed: int) -> int:
    """Encode a direction and speed for the motor-control register."""

    clamped_speed = clamp_speed(speed)
    if direction == "positive":
        return clamped_speed
    if direction == "negative":
        return 0x80 | clamped_speed
    raise ValueError(f"unsupported direction {direction!r}")


def decode_motor_status(raw_value: int) -> MotorStatus:
    """Decode the raw motor-control register into a structured status."""

    if raw_value in STATUS_STOPPED:
        return MotorStatus(raw_value, "stopped", None, 0, False, False, "Stopped")
    if raw_value == STATUS_POSITIVE_LIMIT:
        return MotorStatus(raw_value, "limit", "positive", 0, False, True, "Positive limit reached")
    if raw_value == STATUS_NEGATIVE_LIMIT:
        return MotorStatus(raw_value, "limit", "negative", 0, False, True, "Negative limit reached")
    if 0x01 <= raw_value <= 0x0F:
        return MotorStatus(raw_value, "moving", "positive", raw_value, True, False, f"Moving positive at speed {raw_value}")
    if 0x81 <= raw_value <= 0x8F:
        speed = raw_value & 0x0F
        return MotorStatus(raw_value, "moving", "negative", speed, True, False, f"Moving negative at speed {speed}")
    return MotorStatus(raw_value, "unknown", None, 0, False, False, f"Unknown state 0x{raw_value:02X}")


class RealBucketMotorBus:
    """Low-level driver for six Modbus bucket motor controllers."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        timeout_s: float,
        config: BucketMotorConfig,
        client_factory: Callable[[str, int, float], Any] = create_client,
    ) -> None:
        self.port = port  # COM port for the bucket RS-485 adapter.
        self.baudrate = int(baudrate)  # Serial speed configured in device_ports_and_addr.yaml.
        self.timeout_s = float(timeout_s)  # Per-Modbus request timeout.
        self.config = config  # Logical labels, speed, directions, and watchdog tuning.
        self._client = client_factory(self.port, self.baudrate, self.timeout_s)
        self._connected = False

    @property
    def connected(self) -> bool:
        """Return whether the serial client is currently open."""

        return self._connected

    def connect(self) -> None:
        """Open the serial port or raise if the adapter cannot be reached."""

        self._connected = bool(self._client.connect())
        if not self._connected:
            raise RuntimeError(f"could not open bucket motor Modbus port {self.port}")

    def close(self) -> None:
        """Close the serial client."""

        self._client.close()
        self._connected = False

    def move(self, label: str, direction: Direction, speed: int) -> bool:
        """Command one logical bucket motor to move in the selected direction."""

        return self._write_register(label, REG_MOTOR_CONTROL, encode_motion_command(direction, speed))

    def stop(self, label: str) -> bool:
        """Send an immediate stop command to one logical bucket motor."""

        return self._write_register(label, REG_MOTOR_CONTROL, CMD_STOP)

    def read_status(self, label: str) -> MotorStatus | None:
        """Read and decode one logical bucket motor status register."""

        device_address = self._address_for(label)
        try:
            result = self._client.read_holding_registers(
                address=REG_MOTOR_CONTROL,
                count=1,
                device_id=device_address,
            )
        except (ModbusIOException, OSError):
            return None
        is_error = getattr(result, "isError", None)
        if callable(is_error) and is_error():
            return None
        registers = getattr(result, "registers", None)
        if not isinstance(registers, list) or not registers:
            return None
        status = decode_motor_status(int(registers[0]))
        if self.config.inter_request_delay_s > 0.0:
            time.sleep(self.config.inter_request_delay_s)
        return status

    def _write_register(self, label: str, register: int, value: int) -> bool:
        """Write one holding register for a logical bucket label."""

        device_address = self._address_for(label)
        try:
            result = self._client.write_register(
                address=register,
                value=value,
                device_id=device_address,
            )
        except (ModbusIOException, OSError):
            return False
        is_error = getattr(result, "isError", None)
        if callable(is_error) and is_error():
            return False
        if self.config.inter_request_delay_s > 0.0:
            time.sleep(self.config.inter_request_delay_s)
        return True

    def _address_for(self, label: str) -> int:
        """Return the Modbus slave address for a logical bucket label."""

        try:
            return int(self.config.addresses[label])
        except KeyError as exc:
            raise ValueError(f"unknown bucket label {label!r}") from exc
