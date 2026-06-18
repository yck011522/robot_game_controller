"""Focused tests for bucket motor command mapping and watchdog behavior."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import load as load_profile  # noqa: E402
from subsystems.bucket.common import BUCKET_LABELS, BucketMotorConfig, MotorStatus  # noqa: E402
from subsystems.bucket.controller import BucketControllerRuntime  # noqa: E402
from apps.game_controller import __main__ as game_controller  # noqa: E402


class _FakeClock:
    """Controllable monotonic clock for watchdog tests."""

    def __init__(self) -> None:
        self.now_s = 0.0  # Current fake monotonic timestamp in seconds.

    def __call__(self) -> float:
        """Return the current fake monotonic time."""

        return self.now_s

    def advance(self, seconds: float) -> None:
        """Advance the fake clock by a positive number of seconds."""

        self.now_s += float(seconds)


class _FakeBucketDriver:
    """Minimal bucket driver that records commands and exposes statuses."""

    def __init__(self) -> None:
        self.connected = True  # Runtime telemetry mirrors this connection state.
        self.moves: list[tuple[str, str, int]] = []  # Movement writes observed by the test.
        self.stops: list[str] = []  # Stop writes observed by the test.
        self.statuses = {
            label: MotorStatus(0x00, "stopped", None, 0, False, False, "Stopped")
            for label in BUCKET_LABELS
        }

    def move(self, label: str, direction: str, speed: int) -> bool:
        """Record a movement command and mark the fake motor moving."""

        self.moves.append((label, direction, speed))
        raw = speed if direction == "positive" else 0x80 | speed
        self.statuses[label] = MotorStatus(raw, "moving", direction, speed, True, False, "Moving")
        return True

    def stop(self, label: str) -> bool:
        """Record a stop command and mark the fake motor stopped."""

        self.stops.append(label)
        self.statuses[label] = MotorStatus(0x00, "stopped", None, 0, False, False, "Stopped")
        return True

    def read_status(self, label: str) -> MotorStatus | None:
        """Return the fake status for one bucket."""

        return self.statuses[label]


def _config() -> BucketMotorConfig:
    """Return the standard test config with open negative and close positive."""

    return BucketMotorConfig(
        addresses={"A1": 1, "A2": 2, "A3": 3, "B1": 4, "B2": 5, "B3": 6},
        open_direction="negative",
        close_direction="positive",
        speed=8,
        command_timeout_s=10.0,
        status_poll_interval_s=0.1,
        inter_request_delay_s=0.0,
    )


def test_open_and_close_use_physical_direction_mapping() -> None:
    """Open should command negative; close should command positive."""

    clock = _FakeClock()
    driver = _FakeBucketDriver()
    runtime = BucketControllerRuntime(driver=driver, config=_config(), now_fn=clock)

    runtime.handle_command({"action": "open", "team": "b", "bucket_number": 1, "request_id": "open-B1"})
    runtime.handle_command({"action": "close", "bucket_label": "B1", "request_id": "close-B1"})

    assert driver.moves[0] == ("B1", "negative", 8)
    assert driver.moves[1] == ("B1", "positive", 8)


def test_watchdog_stops_motor_after_timeout() -> None:
    """A moving bucket must receive an explicit stop after the watchdog timeout."""

    clock = _FakeClock()
    driver = _FakeBucketDriver()
    runtime = BucketControllerRuntime(driver=driver, config=_config(), now_fn=clock)

    runtime.handle_command({"action": "open", "team": "b", "bucket_number": 2, "request_id": "open-B2"})
    clock.advance(10.1)
    runtime.tick()

    assert "B2" in driver.stops
    result = runtime.snapshot()["buckets"]["B2"]["last_result"]
    assert result["ok"] is True
    assert "watchdog stopped" in result["message"]


def test_limit_status_completes_command_without_watchdog_stop() -> None:
    """A limit status should clear the active command before timeout."""

    clock = _FakeClock()
    driver = _FakeBucketDriver()
    runtime = BucketControllerRuntime(driver=driver, config=_config(), now_fn=clock)

    runtime.handle_command({"action": "open", "team": "b", "bucket_number": 3, "request_id": "open-B3"})
    driver.statuses["B3"] = MotorStatus(0x90, "limit", "negative", 0, False, True, "Negative limit reached")
    runtime.tick()

    assert "B3" not in driver.stops
    bucket = runtime.snapshot()["buckets"]["B3"]
    assert bucket["active_command"] is None
    assert bucket["last_result"]["ok"] is True


def test_conclusion_uses_one_based_bucket_command() -> None:
    """Conclusion should publish B2, not a zero-based bucket index."""

    commands: list[dict] = []
    state = {
        "team": "b",
        "bucket_values": [0, 1, 0],
        "summed_score": 0,
        "score": 1,
        "conclusion_phase": "sum_bucket",
        "conclusion_active_bucket_index": 1,
        "conclusion_phase_started_mono_ns": 0,
        "conclusion_sum_remainder_units": 0.0,
    }

    game_controller._tick_conclusion_team(
        state,
        dt=1.0,
        game_cfg={"sum_score_rate_unit_per_s": 100.0},
        pose_cfg={},
        stage_state={"winner_team": None},
        bucket_command_fn=lambda *args, **kwargs: commands.append({"args": args, **kwargs}),
    )

    assert commands == [
        {
            "args": ("open",),
            "team": "b",
            "bucket_number": 2,
            "reason": "conclusion_bucket_counted",
        }
    ]


def test_dev_bucket_profile_loads() -> None:
    """The dedicated bucket integration profile should validate."""

    profile = load_profile(REPO_ROOT / "config" / "profiles" / "dev_bucket_integration.yaml")
    assert profile.subsystem_impl("bucket_controller") == "real"
    assert profile.tuning["bucket_controller"]["motor_speed"] == 8
    assert profile.tuning["game"]["duration_s"] == 30


def main() -> int:
    test_open_and_close_use_physical_direction_mapping()
    test_watchdog_stops_motor_after_timeout()
    test_limit_status_completes_command_without_watchdog_stop()
    test_conclusion_uses_one_based_bucket_command()
    test_dev_bucket_profile_loads()
    print("[test] bucket controller tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
