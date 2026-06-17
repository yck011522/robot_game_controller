"""Simulated safety barrier implementations."""

from __future__ import annotations

from subsystems.safety_barrier.common import SafetyBarrierConfig, SafetyBarrierSnapshot, apply_bypass


class SimOpenSafetyBarrier:
    """Safety barrier simulator that always reports all physical channels OK."""

    def __init__(self, config: SafetyBarrierConfig) -> None:
        self._config = config

    def read(self) -> SafetyBarrierSnapshot:
        """Return an all-clear safety barrier sample."""

        raw_channels = [True] * len(self._config.labels)
        return apply_bypass(raw_channels, self._config)

    def close(self) -> None:
        """Release simulator resources."""

        return

