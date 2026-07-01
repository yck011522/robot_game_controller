"""Focused tests for physical admin-button polling and GameController handling.

Run:
    python -m pytest tests/test_admin_buttons.py -q
    C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe -m pytest tests/test_admin_buttons.py -q
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import buttons as gc_buttons  # noqa: E402
from apps.game_controller import operator_inputs as gc_operator_inputs  # noqa: E402
from subsystems.admin_buttons.common import (  # noqa: E402
    ESTOP,
    SKIP,
    START_RESUME,
    AdminButtonConfig,
    AdminButtonRuntime,
)


class FakeAdminButtonDriver:
    """Tiny driver fake with queued raw inputs and recorded lamp writes."""

    def __init__(self) -> None:
        self.inputs = [False, False, True, False]  # DI3 high means NC e-stop clear.
        self.lamp_writes: list[bool] = []  # Coil writes requested by AdminButtonRuntime.

    def connect(self) -> None:
        """Open fake resources."""

        return

    def read_inputs(self) -> tuple[list[bool], list[str]]:
        """Return the current fake raw input vector."""

        return list(self.inputs), []

    def write_resume_lamp(self, on: bool) -> list[str]:
        """Record the requested resume lamp state."""

        self.lamp_writes.append(bool(on))
        return []

    def close(self) -> None:
        """Release fake resources."""

        return


CONFIG = AdminButtonConfig(
    station_label="admin",
    slave_address=1,
    input_start_address=0,
    input_count=4,
    resume_input_index=0,
    skip_input_index=1,
    estop_input_index=2,
    resume_lamp_coil_address=0,
    skip_cooldown_s=0.8,
)


def test_runtime_normalizes_polarity_and_lamp_gates_on_estop() -> None:
    """NO start/skip use high=pressed, while NC e-stop uses low=pressed."""

    driver = FakeAdminButtonDriver()
    runtime = AdminButtonRuntime(driver, CONFIG)

    driver.inputs = [True, False, True, False]
    snapshot = runtime.tick(paused=True, now_mono_s=1.0)
    assert snapshot.buttons[START_RESUME].pressed is True
    assert snapshot.buttons[START_RESUME].edge == "rise"
    assert snapshot.buttons[SKIP].pressed is False
    assert snapshot.buttons[ESTOP].pressed is False
    assert snapshot.resume_lamp_on is True
    assert driver.lamp_writes[-1] is True

    driver.inputs = [False, False, False, False]
    snapshot = runtime.tick(paused=True, now_mono_s=1.1)
    assert snapshot.buttons[ESTOP].pressed is True
    assert snapshot.resume_lamp_on is False
    assert driver.lamp_writes[-1] is False


def test_skip_rising_edge_is_rate_limited() -> None:
    """Skip emits only accepted rising edges, with the configured cooldown."""

    driver = FakeAdminButtonDriver()
    runtime = AdminButtonRuntime(driver, CONFIG)
    runtime.tick(paused=False, now_mono_s=0.0)

    driver.inputs = [False, True, True, False]
    first_press = runtime.tick(paused=False, now_mono_s=1.0)
    assert first_press.buttons[SKIP].edge == "rise"
    first_event = first_press.buttons[SKIP].event_id
    assert first_event is not None

    driver.inputs = [False, False, True, False]
    runtime.tick(paused=False, now_mono_s=1.05)
    driver.inputs = [False, True, True, False]
    bounced_press = runtime.tick(paused=False, now_mono_s=1.2)
    assert bounced_press.buttons[SKIP].edge is None
    assert bounced_press.buttons[SKIP].event_id is None

    driver.inputs = [False, False, True, False]
    runtime.tick(paused=False, now_mono_s=1.4)
    driver.inputs = [False, True, True, False]
    later_press = runtime.tick(paused=False, now_mono_s=1.9)
    assert later_press.buttons[SKIP].edge == "rise"
    assert later_press.buttons[SKIP].event_id != first_event


def test_game_controller_blocks_resume_while_physical_estop_is_pressed() -> None:
    """Physical e-stop level blocks both physical and digital resume requests."""

    control_state = {"soft_pause": False, "button_estop_blocked": False}
    button_state = gc_buttons._initial_button_state(enabled=True)
    gc_buttons._update_button_state(
        button_state,
        {
            "stations": {
                "admin": {
                    START_RESUME: {"pressed": False, "edge": None, "event_id": None},
                    SKIP: {"pressed": False, "edge": None, "event_id": None},
                    ESTOP: {"pressed": True, "edge": "rise", "event_id": 1},
                }
            },
            "errors": [],
        },
    )
    gc_buttons._refresh_button_block(control_state, button_state, telem_age_max_s=1.0)
    assert control_state["soft_pause"] is True
    assert control_state["button_estop_blocked"] is True

    reply = gc_operator_inputs._handle_operator_input_request(
        control_state,
        {"stage": "play"},
        {},
        {"action": "play_resume", "source": "ui", "request_id": 1},
        time.perf_counter_ns(),
        producer="test",
        recovery_timeout_s=4.0,
    )
    assert reply["ok"] is False
    assert "physical e-stop" in str(reply["error"])

    gc_buttons._update_button_state(
        button_state,
        {
            "stations": {
                "admin": {
                    START_RESUME: {"pressed": False, "edge": None, "event_id": None},
                    SKIP: {"pressed": False, "edge": None, "event_id": None},
                    ESTOP: {"pressed": False, "edge": "fall", "event_id": 2},
                }
            },
            "errors": [],
        },
    )
    gc_buttons._refresh_button_block(control_state, button_state, telem_age_max_s=1.0)
    assert control_state["button_estop_blocked"] is False
    assert control_state["soft_pause"] is True

    reply = gc_operator_inputs._handle_operator_input_request(
        control_state,
        {"stage": "play"},
        {},
        {"action": "play_resume", "source": "ui", "request_id": 2},
        time.perf_counter_ns(),
        producer="test",
        recovery_timeout_s=4.0,
    )
    assert reply["ok"] is True
    assert control_state["soft_pause"] is False


def test_button_edges_queue_game_controller_operator_requests() -> None:
    """Start/resume and skip rising edges become queued GC operator requests."""

    button_state = gc_buttons._initial_button_state(enabled=True)
    gc_buttons._update_button_state(
        button_state,
        {
            "stations": {
                "admin": {
                    START_RESUME: {"pressed": True, "edge": "rise", "event_id": 10},
                    SKIP: {"pressed": True, "edge": "rise", "event_id": 11},
                    ESTOP: {"pressed": False, "edge": None, "event_id": None},
                }
            },
            "errors": [],
        },
    )
    requests = gc_buttons._pop_button_operator_requests(button_state)
    assert [request["action"] for request in requests] == ["play_resume", "end_game"]
    assert gc_buttons._pop_button_operator_requests(button_state) == []


def main() -> int:
    """Run this focused test module without pytest."""

    test_runtime_normalizes_polarity_and_lamp_gates_on_estop()
    test_skip_rising_edge_is_rate_limited()
    test_game_controller_blocks_resume_while_physical_estop_is_pressed()
    test_button_edges_queue_game_controller_operator_requests()
    print("[test] admin button tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
