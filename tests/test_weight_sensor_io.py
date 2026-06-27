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
game_controller._apply_weight_bucket_values = (
    game_controller_weight._apply_weight_bucket_values
)
game_controller._begin_play_weight_tare = (
    game_controller_weight._begin_play_weight_tare
)
game_controller._mark_play_weight_tare_published = (
    game_controller_weight._mark_play_weight_tare_published
)
game_controller._tick_play_weight_tare_verification = (
    game_controller_weight._tick_play_weight_tare_verification
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


def test_play_entry_tare_holds_scores_until_verified_zero_reply() -> None:
    """Play entry should show zero until the play_entry tare verifies zero."""

    team_state = {
        "team": "a",
        "bucket_values": [120, 80, 40],
        "score": 240,
    }
    weight_state = game_controller_weight._initial_weight_state(
        enabled=True,
        min_increment_g=0.0,
    )
    weight_state.update(
        {
            "bucket_cell_map": BUCKET_CELL_MAP,
            "cells_g": {str(i): float(i) for i in range(1, 13)},
            "tare_seq": 4,
            "play_tare_last_request_mono_s": 0.0,
        }
    )

    should_publish = game_controller._begin_play_weight_tare(
        weight_state,
        {"a": team_state},
    )
    assert should_publish is True
    assert team_state["bucket_values"] == [0.0, 0.0, 0.0]
    assert team_state["score"] == 0

    game_controller._apply_weight_bucket_values(team_state, weight_state)
    assert team_state["bucket_values"] == [0.0, 0.0, 0.0]

    weight_state["tare_seq"] = 5
    warning = game_controller._tick_play_weight_tare_verification(
        weight_state,
        now_s=0.2,
        publish_tare=lambda: None,
    )
    assert warning is None
    assert weight_state["play_tare_pending"] is True
    game_controller._apply_weight_bucket_values(team_state, weight_state)
    assert team_state["bucket_values"] == [0.0, 0.0, 0.0]

    weight_state["tare_seq"] = 6
    weight_state["cells_g"] = {str(i): 0.5 for i in range(1, 13)}
    warning = game_controller._tick_play_weight_tare_verification(
        weight_state,
        now_s=0.3,
        publish_tare=lambda: None,
    )
    assert warning is None
    assert weight_state["play_tare_pending"] is False
    game_controller._apply_weight_bucket_values(team_state, weight_state)
    assert team_state["bucket_values"] == [1.0, 1.0, 1.0]


def test_play_entry_tare_retries_then_continues_nonfatal() -> None:
    """A non-zero play tare should retry briefly, then continue with warning."""

    published_count = 0

    def publish_tare() -> None:
        """Count one retry tare command."""

        nonlocal published_count
        published_count += 1

    team_state = {
        "team": "a",
        "bucket_values": [120, 80, 40],
        "score": 240,
    }
    weight_state = game_controller_weight._initial_weight_state(
        enabled=True,
        min_increment_g=0.0,
    )
    weight_state.update(
        {
            "bucket_cell_map": BUCKET_CELL_MAP,
            "cells_g": {str(i): 5.0 for i in range(1, 13)},
            "tare_seq": 10,
            "play_tare_max_retries": 2,
            "play_tare_retry_interval_s": 0.1,
        }
    )

    game_controller._begin_play_weight_tare(
        weight_state,
        {"a": team_state},
    )
    game_controller._mark_play_weight_tare_published(weight_state, now_s=0.0)
    assert team_state["bucket_values"] == [0.0, 0.0, 0.0]

    warning = None
    for now_s, tare_seq in ((0.2, 11), (0.4, 12)):
        weight_state["tare_seq"] = tare_seq
        warning = game_controller._tick_play_weight_tare_verification(
            weight_state,
            now_s=now_s,
            publish_tare=publish_tare,
        )
        assert warning is None

    assert published_count == 2
    assert weight_state["play_tare_pending"] is True

    weight_state["tare_seq"] = 13
    warning = game_controller._tick_play_weight_tare_verification(
        weight_state,
        now_s=0.6,
        publish_tare=publish_tare,
    )
    assert warning is not None
    assert "continuing" in warning
    assert weight_state["play_tare_pending"] is False

    game_controller._apply_weight_bucket_values(team_state, weight_state)
    assert team_state["bucket_values"] == [10.0, 10.0, 10.0]


def test_dev_bucket_profile_enables_weight_sensor() -> None:
    """The bucket integration profile should bring up the real weight sensor."""

    profile = load_profile(REPO_ROOT / "config" / "profiles" / "dev_bucket_integration.yaml")
    assert profile.subsystem_impl("weight_sensor_io") == "real"


def main() -> int:
    test_tare_offsets_are_transparent_in_published_cells()
    test_game_controller_sums_twelve_cells_into_team_b_buckets()
    test_play_entry_tare_holds_scores_until_verified_zero_reply()
    test_play_entry_tare_retries_then_continues_nonfatal()
    test_dev_bucket_profile_enables_weight_sensor()
    print("[test] weight sensor tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
