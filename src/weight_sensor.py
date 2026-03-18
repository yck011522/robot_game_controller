"""Weight sensor system for scoring buckets.

Each team has 3 buckets monitored by load cells on an RS-485 bus.
Bucket IDs: 11, 12, 13 (Team 1) and 21, 22, 23 (Team 2).

This module provides:
  - WeightSensorSystem — real hardware reader (RS-485 via USB adapter)
  - SimulatedWeightSensorSystem — drop-in replacement for testing

Both share the same interface:
  - start() / stop() — lifecycle
  - get_weight(bucket_id) → float (grams)
  - get_all_weights() → dict[int, float]
  - is_connected(bucket_id) → bool
  - all_connected → bool
  - connected_count → (connected, total)

The real system runs its own reader thread at ~50 Hz;
the simulated system reads from GameSettings.sim_bucket_weights.

Usage:
    # Real hardware
    system = WeightSensorSystem(bucket_ids=[11, 12, 13, 21, 22, 23])
    system.start()
    weights = system.get_all_weights()  # {11: 150.3, 12: 0.0, ...}
    system.stop()

    # Simulated
    system = SimulatedWeightSensorSystem(
        bucket_ids=[11, 12, 13, 21, 22, 23], settings=settings
    )
    system.start()
"""

import time
import threading
import logging
from typing import Optional

logger = logging.getLogger(__name__)

# Default bucket IDs
TEAM1_BUCKET_IDS = [11, 12, 13]
TEAM2_BUCKET_IDS = [21, 22, 23]
ALL_BUCKET_IDS = TEAM1_BUCKET_IDS + TEAM2_BUCKET_IDS

_SENSOR_READ_HZ = 50


class WeightSensorSystem:
    """Real weight sensor reader — RS-485 via USB adapter.

    Runs a background thread that polls all load cells on the bus
    at ~50 Hz and updates internal registers.

    TODO: Implement RS-485 protocol when hardware is available.
    """

    def __init__(self, bucket_ids: list[int], port: Optional[str] = None):
        self._bucket_ids = list(bucket_ids)
        self._port = port

        self._lock = threading.Lock()
        self._weights: dict[int, float] = {bid: 0.0 for bid in bucket_ids}
        self._connected: dict[int, bool] = {bid: False for bid in bucket_ids}

        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._actual_hz: float = 0.0

    def start(self):
        """Start the RS-485 reader thread."""
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="weight-sensor-reader", daemon=True
        )
        self._reader_thread.start()
        logger.info(
            "WeightSensorSystem started (buckets: %s, port: %s)",
            self._bucket_ids,
            self._port,
        )

    def stop(self):
        """Stop the reader thread."""
        self._stop_event.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        logger.info("WeightSensorSystem stopped")

    def get_weight(self, bucket_id: int) -> float:
        """Get the latest weight reading for a bucket (grams)."""
        with self._lock:
            return self._weights.get(bucket_id, 0.0)

    def get_all_weights(self) -> dict[int, float]:
        """Get all weight readings."""
        with self._lock:
            return dict(self._weights)

    def is_connected(self, bucket_id: int) -> bool:
        with self._lock:
            return self._connected.get(bucket_id, False)

    @property
    def all_connected(self) -> bool:
        with self._lock:
            return all(self._connected.values())

    @property
    def connected_count(self) -> tuple[int, int]:
        with self._lock:
            c = sum(1 for v in self._connected.values() if v)
            return (c, len(self._connected))

    @property
    def actual_hz(self) -> float:
        return self._actual_hz

    def _reader_loop(self):
        """Poll RS-485 bus for weight readings."""
        dt = 1.0 / _SENSOR_READ_HZ
        loop_count = 0
        measure_start = time.perf_counter()

        while not self._stop_event.is_set():
            cycle_start = time.perf_counter()
            loop_count += 1

            # Measure Hz
            elapsed = cycle_start - measure_start
            if elapsed >= 0.5:
                self._actual_hz = loop_count / elapsed
                loop_count = 0
                measure_start = cycle_start

            # TODO: Real RS-485 communication here.
            # For now, no sensors will be detected.

            # Sleep
            remaining = dt - (time.perf_counter() - cycle_start)
            if remaining > 0.0015:
                self._stop_event.wait(remaining - 0.0015)
            while time.perf_counter() - cycle_start < dt:
                if self._stop_event.is_set():
                    return
                time.sleep(0)


class SimulatedWeightSensorSystem:
    """Simulated weight sensors for testing without hardware.

    Drop-in replacement for WeightSensorSystem. Reads bucket weights from
    GameSettings.sim_bucket_weights, which are set by the UI simulator panel.

    Runs its own thread to mimic the real system's asynchronous updates
    and provide accurate Hz measurement.
    """

    def __init__(self, bucket_ids: list[int], settings):
        self._bucket_ids = list(bucket_ids)
        self._settings = settings

        self._lock = threading.Lock()
        self._weights: dict[int, float] = {bid: 0.0 for bid in bucket_ids}

        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._actual_hz: float = 0.0

    def start(self):
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="sim-weight-reader", daemon=True
        )
        self._reader_thread.start()
        logger.info(
            "SimulatedWeightSensorSystem started (buckets: %s)",
            self._bucket_ids,
        )

    def stop(self):
        self._stop_event.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        logger.info("SimulatedWeightSensorSystem stopped")

    def get_weight(self, bucket_id: int) -> float:
        with self._lock:
            return self._weights.get(bucket_id, 0.0)

    def get_all_weights(self) -> dict[int, float]:
        with self._lock:
            return dict(self._weights)

    def is_connected(self, bucket_id: int) -> bool:
        return bucket_id in self._bucket_ids

    @property
    def all_connected(self) -> bool:
        return True

    @property
    def connected_count(self) -> tuple[int, int]:
        return (len(self._bucket_ids), len(self._bucket_ids))

    @property
    def actual_hz(self) -> float:
        return self._actual_hz

    def _reader_loop(self):
        """Periodically read simulated weights from GameSettings."""
        dt = 1.0 / _SENSOR_READ_HZ
        loop_count = 0
        measure_start = time.perf_counter()

        while not self._stop_event.is_set():
            cycle_start = time.perf_counter()
            loop_count += 1

            # Measure Hz
            elapsed = cycle_start - measure_start
            if elapsed >= 0.5:
                self._actual_hz = loop_count / elapsed
                loop_count = 0
                measure_start = cycle_start

            # Read from settings
            sim_weights = self._settings.get("sim_bucket_weights")
            with self._lock:
                for bid in self._bucket_ids:
                    self._weights[bid] = sim_weights.get(bid, 0.0)

            # Sleep
            remaining = dt - (time.perf_counter() - cycle_start)
            if remaining > 0.0015:
                self._stop_event.wait(remaining - 0.0015)
            while time.perf_counter() - cycle_start < dt:
                if self._stop_event.is_set():
                    return
                time.sleep(0)
