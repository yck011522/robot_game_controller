"""Thin simulated load-cell bus for non-hardware profiles."""

from __future__ import annotations

from subsystems.weight_sensor.common import WeightSensorConfig


class SimLoadCellBus:
    """In-memory load-cell bus that reports zero grams for every sensor."""

    def __init__(self, config: WeightSensorConfig) -> None:
        self.config = config  # Slave IDs and conversion settings, kept for interface parity.
        self.decimals_by_slave = {slave: 0 for slave in config.slave_addresses}
        self._connected = False

    @property
    def connected(self) -> bool:
        """Return whether the simulated bus has been started."""

        return self._connected

    def connect(self) -> None:
        """Mark the simulated bus connected."""

        self._connected = True

    def close(self) -> None:
        """Mark the simulated bus closed."""

        self._connected = False

    def read_grams_raw(self, slave_address: int) -> tuple[float, int]:
        """Return a zero reading for one simulated sensor."""

        return 0.0, 0

