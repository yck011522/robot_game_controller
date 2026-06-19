"""Focused tests for load-cell tare and GameController bucket summing."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import load as load_profile  # noqa: E402
from subsystems.weight_sensor.common import BUCKET_CELL_MAP, WeightSensorConfig  # noqa: E402
from subsystems.weight_sensor.runtime import WeightSensorRuntime  # noqa: E402
from apps.game_controller import __main__ as game_controller  # noqa: E402
from apps.game_controller import weight as game_controller_weight  # noqa: E402

game_controller._bucket_values_from_weight = (
    game_controller_weight._bucket_values_from_weight
)


class _FakeLoadCellBus:
    """Queue-backed fake load-cell bus for deterministic tare tests."""

    def __init__(self, values_by_slave: dict[int, list[float]]) -> None:
        self.connected = True  # Runtime snapshot connection flag.
        self.decimals_by_slave = {slave: 0 for slave in values_by_slave}
        self.values_by_slave = {slave: list(values) for slave, values in values_by_slave.items()}

    def read_grams_raw(self, slave_address: int) -> tuple[float, int]:
        """Pop the next gram value for one fake sensor."""

        values = self.values_by_slave[slave_address]
        value = values.pop(0) if values else 0.0
        return float(value), int(round(value))


def test_tare_offsets_are_transparent_in_published_cells() -> None:
    """Runtime tare should subtract startup offsets before publishing cells_g."""

    values = {
        1: [100.0, 100.0, 125.0],
        2: [50.0, 50.0, 80.0],
    }
    runtime = WeightSensorRuntime(
        driver=_FakeLoadCellBus(values),
        config=WeightSensorConfig(slave_addresses=(1, 2), zero_count=0.0, grams_per_count=1.0),
    )

    runtime.tare(samples=2, reason="test")
    runtime.sample_cycle()
    snapshot = runtime.snapshot()

    assert snapshot["tare_offsets_g"] == {"1": 100.0, "2": 50.0}
    assert snapshot["cells_g"] == {"1": 25.0, "2": 30.0}
    assert snapshot["cell_ok"] == {"1": True, "2": True}


def test_game_controller_sums_twelve_cells_into_team_b_buckets() -> None:
    """GC should sum pairs 7/8, 9/10, and 11/12 into B1/B2/B3."""

    weight_state = {
        "enabled": True,
        "bucket_cell_map": BUCKET_CELL_MAP,
        "cells_g": {str(i): float(i) for i in range(1, 13)},
    }

    assert game_controller._bucket_values_from_weight("b", weight_state) == [15.0, 19.0, 23.0]
    assert game_controller._bucket_values_from_weight("a", weight_state) == [3.0, 7.0, 11.0]


def test_dev_bucket_profile_enables_weight_sensor() -> None:
    """The bucket integration profile should bring up the real weight sensor."""

    profile = load_profile(REPO_ROOT / "config" / "profiles" / "dev_bucket_integration.yaml")
    assert profile.subsystem_impl("weight_sensor_io") == "real"


def main() -> int:
    test_tare_offsets_are_transparent_in_published_cells()
    test_game_controller_sums_twelve_cells_into_team_b_buckets()
    test_dev_bucket_profile_enables_weight_sensor()
    print("[test] weight sensor tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
