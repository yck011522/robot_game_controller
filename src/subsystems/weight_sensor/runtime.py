"""Weight sensor polling and tare-offset runtime."""

from __future__ import annotations

import time
from collections import deque
from typing import Any, Callable

from subsystems.weight_sensor.common import WeightReading, WeightSensorConfig


DEFAULT_TARGET_HZ = 1000.0  # Run back-to-back full cycles; serial/Modbus timing sets actual rate.
DEFAULT_CLIENT_TIMEOUT_S = 0.05  # Per-Modbus request timeout from the validated experiment script.
DEFAULT_READ_RETRIES = 1  # One retry gives resilience while keeping the cycle rate predictable.
DEFAULT_RETRY_DELAY_S = 0.0005  # Delay before retrying one failed load-cell request.
DEFAULT_INTER_REQUEST_DELAY_S = 0.0  # The validated script ran cleanly with no extra quiet gap.
DEFAULT_TARE_CYCLES = 10  # Startup/reset tare samples per cell (averaged after trimming outliers).
DEFAULT_TARE_OUTLIER_TRIM = 4  # Most-extreme samples discarded per cell before averaging (split low/high).



class WeightSensorRuntime:
    """Poll load cells, maintain tare offsets, and expose JSON-ready snapshots."""

    def __init__(
        self,
        *,
        driver: Any,
        config: WeightSensorConfig,
        now_fn: Callable[[], float] = time.perf_counter,
    ) -> None:
        self.driver = driver  # Concrete real or simulated load-cell bus.
        self.config = config  # Slave IDs and conversion constants.
        self._now = now_fn  # Monotonic clock injection used by tests.
        self._tare_offsets_g = {slave: 0.0 for slave in config.slave_addresses}
        self._latest: dict[int, WeightReading] = {}
        self._errors: dict[int, str | None] = {slave: None for slave in config.slave_addresses}
        self._cycle_times: deque[float] = deque(maxlen=30)
        self._cycle_seq = 0
        self._tare_seq = 0
        self._last_cycle_duration_ms = 0.0
        self._last_tare_reason: str | None = None

    def tare(
        self,
        *,
        samples: int = DEFAULT_TARE_CYCLES,
        outlier_trim: int = DEFAULT_TARE_OUTLIER_TRIM,
        reason: str = "startup",
    ) -> None:
        """Collect several raw samples per cell and use a trimmed mean as offset.

        Rather than a plain average, each cell's raw samples are sorted and the
        most-extreme readings are discarded (``outlier_trim`` total, split as
        ``outlier_trim // 2`` lowest + the remainder highest) before averaging
        the rest. With the defaults (``samples=10``, ``outlier_trim=4``) this
        drops the 2 lowest and 2 highest of 10 samples and averages the middle 6,
        rejecting transient spikes/dropouts that would otherwise bias the zero.

        Trimming is skipped automatically when there are too few good samples to
        leave at least one remaining (e.g. a short ``samples=2`` test cycle),
        falling back to a plain mean. Cells that returned no good reading keep
        their previous offset.

        Args:
            samples: Raw read cycles to collect per cell (>= 1).
            outlier_trim: Total extreme samples to drop per cell before averaging.
            reason: Tag stored in the snapshot for diagnostics (e.g. "startup").
        """

        raw_samples: dict[int, list[float]] = {
            slave: [] for slave in self.config.slave_addresses
        }
        for _ in range(max(1, int(samples))):
            for slave_address in self.config.slave_addresses:
                reading = self._read_one(slave_address, apply_tare=False)
                if reading.ok:
                    raw_samples[slave_address].append(reading.grams_raw)
                self._inter_request_sleep()
        for slave_address in self.config.slave_addresses:
            offset = _trimmed_mean(raw_samples[slave_address], max(0, int(outlier_trim)))
            if offset is not None:
                self._tare_offsets_g[slave_address] = offset
        self._tare_seq += 1
        self._last_tare_reason = reason


    def sample_cycle(self) -> None:
        """Read every configured load cell once and update latest readings."""

        started_s = self._now()
        for slave_address in self.config.slave_addresses:
            self._latest[slave_address] = self._read_one(slave_address, apply_tare=True)
            self._inter_request_sleep()
        finished_s = self._now()
        self._cycle_seq += 1
        self._cycle_times.append(finished_s)
        self._last_cycle_duration_ms = max(0.0, (finished_s - started_s) * 1000.0)

    def snapshot(self) -> dict[str, Any]:
        """Return the latest load-cell telemetry in BUS.md ``telem.weight`` shape."""

        cells_g: dict[str, float] = {}
        raw_i32: dict[str, int | None] = {}
        cell_ok: dict[str, bool] = {}
        errors: dict[str, str] = {}
        for slave_address in self.config.slave_addresses:
            reading = self._latest.get(slave_address)
            key = str(slave_address)
            if reading is None:
                cells_g[key] = 0.0
                raw_i32[key] = None
                cell_ok[key] = False
                errors[key] = "no reading yet"
                continue
            cells_g[key] = reading.grams_tared
            raw_i32[key] = reading.raw_i32 if reading.ok else None
            cell_ok[key] = reading.ok
            if reading.error:
                errors[key] = reading.error
        return {
            "connected": bool(getattr(self.driver, "connected", False)),
            "cycle_seq": self._cycle_seq,
            "tare_seq": self._tare_seq,
            "last_tare_reason": self._last_tare_reason,
            "slave_addresses": list(self.config.slave_addresses),
            "decimal_places": {
                str(slave): int(getattr(self.driver, "decimals_by_slave", {}).get(slave, 0))
                for slave in self.config.slave_addresses
            },
            "tare_offsets_g": {
                str(slave): self._tare_offsets_g.get(slave, 0.0)
                for slave in self.config.slave_addresses
            },
            "cells_g": cells_g,
            "raw_i32": raw_i32,
            "cell_ok": cell_ok,
            "errors": errors,
            "last_cycle_duration_ms": self._last_cycle_duration_ms,
            "observed_cycle_hz": self.observed_cycle_hz(),
        }

    def observed_cycle_hz(self) -> float:
        """Return observed full-cycle read frequency."""

        if len(self._cycle_times) < 2:
            return 0.0
        span_s = self._cycle_times[-1] - self._cycle_times[0]
        if span_s <= 0.0:
            return 0.0
        return (len(self._cycle_times) - 1) / span_s

    def _read_one(self, slave_address: int, *, apply_tare: bool) -> WeightReading:
        """Read one sensor with retry and return raw/tared grams."""

        last_error: Exception | None = None
        for attempt in range(DEFAULT_READ_RETRIES + 1):
            try:
                grams_raw, raw_i32 = self.driver.read_grams_raw(slave_address)
                grams_tared = grams_raw - self._tare_offsets_g.get(slave_address, 0.0) if apply_tare else grams_raw
                self._errors[slave_address] = None
                return WeightReading(slave_address, raw_i32, grams_raw, grams_tared, True)
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                if attempt < DEFAULT_READ_RETRIES and DEFAULT_RETRY_DELAY_S > 0.0:
                    time.sleep(DEFAULT_RETRY_DELAY_S)
        message = str(last_error or "read failed")
        self._errors[slave_address] = message
        previous = self._latest.get(slave_address)
        previous_tared = previous.grams_tared if previous is not None else 0.0
        previous_raw = previous.raw_i32 if previous is not None else 0
        previous_raw_g = previous.grams_raw if previous is not None else 0.0
        return WeightReading(slave_address, previous_raw, previous_raw_g, previous_tared, False, message)

    def _inter_request_sleep(self) -> None:
        """Apply the hardcoded quiet gap between Modbus requests."""

        if DEFAULT_INTER_REQUEST_DELAY_S > 0.0:
            time.sleep(DEFAULT_INTER_REQUEST_DELAY_S)


def _trimmed_mean(values: list[float], trim: int) -> float | None:
    """Return the mean of ``values`` after dropping the most-extreme samples.

    ``trim`` is the total number of samples to discard, split as ``trim // 2``
    from the low end and the remainder from the high end (so an odd trim drops
    one extra high sample). Trimming is skipped when it would leave fewer than
    one sample, in which case the plain mean of all values is returned. Returns
    ``None`` for an empty list so the caller can keep the previous offset.
    """

    if not values:
        return None
    ordered = sorted(values)
    if trim > 0 and (len(ordered) - trim) >= 1:
        low = trim // 2
        high = trim - low
        ordered = ordered[low: len(ordered) - high]
    return sum(ordered) / len(ordered)

