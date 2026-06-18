"""Real Modbus RTU reader for twelve load cells."""

from __future__ import annotations

import inspect
import struct
from typing import Any, Callable

from pymodbus.client import ModbusSerialClient

from subsystems.weight_sensor.common import WeightSensorConfig


REG_DECIMAL_PLACES = 0x0000
REG_REALTIME_WEIGHT = 0x0001
DEFAULT_DECIMAL_PLACES = 2


def regs_to_i32(hi_u16: int, lo_u16: int) -> int:
    """Convert two big-endian uint16 Modbus registers into one signed int32."""

    combined = (int(hi_u16) << 16) | int(lo_u16)
    return struct.unpack(">i", struct.pack(">I", combined))[0]


def scaled_raw_to_grams(raw_scaled: float, zero_count: float, grams_per_count: float) -> float:
    """Convert scaled sensor counts into grams before tare offset."""

    return (raw_scaled - zero_count) * grams_per_count


def create_client(port: str, baudrate: int, timeout_s: float) -> ModbusSerialClient:
    """Create a synchronous Modbus RTU client for the load-cell bus."""

    return ModbusSerialClient(
        port=port,
        baudrate=baudrate,
        timeout=timeout_s,
        stopbits=1,
        bytesize=8,
        parity="N",
        retries=0,
    )


class RealLoadCellBus:
    """Low-level Modbus client that reads load-cell raw values and decimals."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        timeout_s: float,
        config: WeightSensorConfig,
        client_factory: Callable[[str, int, float], Any] = create_client,
    ) -> None:
        self.port = port  # COM port for the shared load-cell RS-485 adapter.
        self.baudrate = int(baudrate)  # Serial speed from hardware config.
        self.timeout_s = float(timeout_s)  # Per-Modbus request timeout.
        self.config = config  # Slave IDs and count-to-gram conversion settings.
        self.decimals_by_slave: dict[int, int] = {}  # Decimal places reported by each load-cell controller.
        self._client = client_factory(self.port, self.baudrate, self.timeout_s)
        self._connected = False

    @property
    def connected(self) -> bool:
        """Return whether the serial client is currently open."""

        return self._connected

    def connect(self) -> None:
        """Open the serial port and read decimal-place metadata."""

        self._connected = bool(self._client.connect())
        if not self._connected:
            raise RuntimeError(f"could not open weight sensor Modbus port {self.port}")
        self.decimals_by_slave = {}
        for slave_address in self.config.slave_addresses:
            try:
                self.decimals_by_slave[slave_address] = self.read_decimal_places(slave_address)
            except Exception:
                self.decimals_by_slave[slave_address] = DEFAULT_DECIMAL_PLACES

    def close(self) -> None:
        """Close the serial client."""

        self._client.close()
        self._connected = False

    def read_decimal_places(self, slave_address: int) -> int:
        """Read decimal-place scaling from one load-cell controller."""

        response = self._read_holding_registers(REG_DECIMAL_PLACES, 1, slave_address)
        if response.isError():
            raise RuntimeError(f"failed reading decimal places from slave {slave_address}: {response}")
        return int(response.registers[0])

    def read_raw_i32(self, slave_address: int) -> int:
        """Read one real-time signed 32-bit weight value from a controller."""

        response = self._read_holding_registers(REG_REALTIME_WEIGHT, 2, slave_address)
        if response.isError():
            raise RuntimeError(f"failed reading real-time weight from slave {slave_address}: {response}")
        hi, lo = response.registers[0], response.registers[1]
        return regs_to_i32(hi, lo)

    def read_grams_raw(self, slave_address: int) -> tuple[float, int]:
        """Read one sensor and convert it to grams before tare offset."""

        raw_i32 = self.read_raw_i32(slave_address)
        decimals = int(self.decimals_by_slave.get(slave_address, DEFAULT_DECIMAL_PLACES))
        scaled = raw_i32 / (10 ** decimals)
        grams = scaled_raw_to_grams(
            scaled,
            zero_count=self.config.zero_count,
            grams_per_count=self.config.grams_per_count,
        )
        return grams, raw_i32

    def _read_holding_registers(self, address: int, count: int, slave_address: int):
        """Call pymodbus read_holding_registers across API variants."""

        fn = self._client.read_holding_registers
        params = inspect.signature(fn).parameters
        if "slave" in params:
            return fn(address=address, count=count, slave=slave_address)
        if "unit" in params:
            return fn(address=address, count=count, unit=slave_address)
        if "device_id" in params:
            return fn(address=address, count=count, device_id=slave_address)
        return fn(address, count, slave_address)

