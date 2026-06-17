"""Real Modbus RTU safety barrier reader."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from pymodbus.client import ModbusSerialClient

from subsystems.safety_barrier.common import (
    CHANNELS_PER_DEVICE,
    SafetyBarrierConfig,
    SafetyBarrierSnapshot,
    apply_bypass,
)


MODBUS_START_ADDRESS = 0
MODBUS_INPUT_COUNT = 2


@dataclass(frozen=True)
class RealSafetyBarrierTransport:
    """Serial transport and Modbus address settings for the safety barrier."""

    port: str
    baudrate: int
    slave_addresses: tuple[int, ...]
    read_timeout_s: float
    inter_request_delay_s: float


class RealModbusSafetyBarrier:
    """Poll four RS485 Modbus IO units and decode eight normally-closed inputs."""

    def __init__(self, transport: RealSafetyBarrierTransport, config: SafetyBarrierConfig) -> None:
        self._transport = transport
        self._config = config
        expected_channels = len(transport.slave_addresses) * CHANNELS_PER_DEVICE
        if len(config.labels) != expected_channels:
            raise ValueError(
                "safety barrier channel_order length must match "
                f"{len(transport.slave_addresses)} addresses x {CHANNELS_PER_DEVICE} inputs"
            )
        self._client = ModbusSerialClient(
            port=transport.port,
            baudrate=transport.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=transport.read_timeout_s,
            retries=0,
        )
        if not self._client.connect():
            raise RuntimeError(f"could not open safety barrier Modbus port {transport.port}")

    def read(self) -> SafetyBarrierSnapshot:
        """Poll every configured IO unit and return the decoded barrier state."""

        raw_channels: list[bool] = []
        errors: list[str] = []
        for slave_address in self._transport.slave_addresses:
            bits, error = self._read_device_inputs(slave_address)
            if error is not None:
                errors.append(error)
                bits = [False, False]
            raw_channels.extend(bits[:CHANNELS_PER_DEVICE])
            if self._transport.inter_request_delay_s > 0.0:
                time.sleep(self._transport.inter_request_delay_s)
        return apply_bypass(raw_channels, self._config, errors=errors)

    def close(self) -> None:
        """Close the Modbus serial client."""

        self._client.close()

    def _read_device_inputs(self, slave_address: int) -> tuple[list[bool], str | None]:
        """Read the two discrete inputs for one Modbus slave address."""

        try:
            response: Any = self._client.read_discrete_inputs(
                MODBUS_START_ADDRESS,
                count=MODBUS_INPUT_COUNT,
                device_id=slave_address,
            )
        except Exception as exc:  # noqa: BLE001
            return [False, False], f"A{slave_address}:exception:{exc}"

        is_error = getattr(response, "isError", None)
        if callable(is_error) and is_error():
            return [False, False], f"A{slave_address}:modbus_error"

        bits = getattr(response, "bits", None)
        if not isinstance(bits, list) or len(bits) < MODBUS_INPUT_COUNT:
            return [False, False], f"A{slave_address}:short_response"

        # The light barriers are normally closed: HIGH/1 means pass/OK.
        return [bool(bits[0]), bool(bits[1])], None

