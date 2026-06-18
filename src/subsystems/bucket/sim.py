"""Thin no-op bucket controller implementation for non-hardware profiles."""

from __future__ import annotations

from subsystems.bucket.common import BUCKET_LABELS, BucketMotorConfig, Direction, MotorStatus


class SimBucketMotorBus:
    """Small in-memory bucket driver that accepts commands and reports stopped."""

    def __init__(self, config: BucketMotorConfig) -> None:
        self.config = config  # Runtime config kept for the same interface as the real driver.
        self._statuses = {
            label: MotorStatus(0x00, "stopped", None, 0, False, False, "Sim stopped")
            for label in BUCKET_LABELS
        }
        self._connected = False

    @property
    def connected(self) -> bool:
        """Return whether the simulated controller has been started."""

        return self._connected

    def connect(self) -> None:
        """Mark the simulated controller connected."""

        self._connected = True

    def close(self) -> None:
        """Mark the simulated controller closed."""

        self._connected = False

    def move(self, label: str, direction: Direction, speed: int) -> bool:
        """Accept a movement command without driving hardware."""

        self._statuses[label] = MotorStatus(
            0x00,
            "stopped",
            direction,
            0,
            False,
            False,
            f"Sim accepted {direction} at speed {speed}",
        )
        return True

    def stop(self, label: str) -> bool:
        """Accept a stop command without driving hardware."""

        self._statuses[label] = MotorStatus(0x00, "stopped", None, 0, False, False, "Sim stopped")
        return True

    def read_status(self, label: str) -> MotorStatus | None:
        """Return the current simulated status for one logical bucket."""

        return self._statuses.get(label)

