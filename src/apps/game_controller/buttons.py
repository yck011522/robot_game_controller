"""Physical admin-button helpers for the game controller runtime."""

from __future__ import annotations

import time
from typing import Any

from subsystems.admin_buttons.common import ESTOP, SKIP, START_RESUME


def _initial_button_state(*, enabled: bool) -> dict[str, Any]:
    """Return GameController's local physical admin-button cache."""

    return {
        "enabled": enabled,
        "stations": {},
        "errors": [],
        "last_recv_mono_s": None,
        "stale": enabled,
        "pending_requests": [],
    }


def _update_button_state(state: dict[str, Any], body: dict[str, Any]) -> None:
    """Store the latest ``telem.buttons`` payload and queue rise-edge requests."""

    stations_raw = body.get("stations")
    stations = stations_raw if isinstance(stations_raw, dict) else {}
    normalized: dict[str, dict[str, dict[str, Any]]] = {}
    pending = state.setdefault("pending_requests", [])
    for station, controls in stations.items():
        if not isinstance(controls, dict):
            continue
        station_key = str(station)
        normalized[station_key] = {}
        for name, signal in controls.items():
            if not isinstance(signal, dict):
                continue
            control_name = str(name)
            pressed = bool(signal.get("pressed", False))
            edge = signal.get("edge") if isinstance(signal.get("edge"), str) else None
            event_id = signal.get("event_id")
            normalized[station_key][control_name] = {
                "pressed": pressed,
                "edge": edge,
                "event_id": event_id,
            }
            if edge == "rise" and control_name in (START_RESUME, SKIP):
                action = "play_resume" if control_name == START_RESUME else "end_game"
                request_id = f"{station_key}:{control_name}:{event_id}"
                pending.append(
                    {
                        "action": action,
                        "source": f"button_controller.{station_key}.{control_name}",
                        "request_id": request_id,
                    }
                )
    state["stations"] = normalized
    errors = body.get("errors")
    state["errors"] = [str(value) for value in errors] if isinstance(errors, list) else []
    state["last_recv_mono_s"] = time.perf_counter()
    state["stale"] = False


def _refresh_button_block(
    control_state: dict[str, Any],
    button_state: dict[str, Any],
    telem_age_max_s: float,
) -> None:
    """Pause while button telemetry is stale, errored, or e-stop is pressed."""

    if not bool(button_state.get("enabled", False)):
        control_state["button_estop_blocked"] = False
        button_state["stale"] = False
        return

    last_recv = button_state.get("last_recv_mono_s")
    stale = (
        not isinstance(last_recv, float)
        or (time.perf_counter() - last_recv) > telem_age_max_s
    )
    button_state["stale"] = stale
    blocked = stale or bool(button_state.get("errors")) or _estop_pressed(button_state)
    control_state["button_estop_blocked"] = blocked
    if blocked:
        control_state["soft_pause"] = True


def _pop_button_operator_requests(button_state: dict[str, Any]) -> list[dict[str, Any]]:
    """Return and clear queued physical-button operator requests."""

    pending = button_state.get("pending_requests")
    if not isinstance(pending, list) or not pending:
        return []
    requests = [dict(item) for item in pending if isinstance(item, dict)]
    pending.clear()
    return requests


def _button_pause_reason(button_state: dict[str, Any]) -> str:
    """Return the pause reason caused by physical admin-button state."""

    if bool(button_state.get("stale", False)):
        return "buttons_stale"
    if bool(button_state.get("errors")):
        return "buttons_error"
    return "estop"


def _state_full_buttons(button_state: dict[str, Any]) -> dict[str, Any]:
    """Build the ``state.full.buttons`` block for UI and logging consumers."""

    stations = button_state.get("stations")
    if not isinstance(stations, dict):
        stations = {}
    return {
        station: {
            name: bool(signal.get("pressed", False))
            for name, signal in controls.items()
            if isinstance(signal, dict)
        }
        for station, controls in stations.items()
        if isinstance(controls, dict)
    }


def _state_full_estop(button_state: dict[str, Any]) -> dict[str, bool]:
    """Build the normalized ``state.full.safety.estop`` payload."""

    return {"pressed": _estop_pressed(button_state)}


def _estop_pressed(button_state: dict[str, Any]) -> bool:
    """Return whether any station currently asserts the physical e-stop."""

    stations = button_state.get("stations")
    if not isinstance(stations, dict):
        return False
    for controls in stations.values():
        if not isinstance(controls, dict):
            continue
        estop = controls.get(ESTOP)
        if isinstance(estop, dict) and bool(estop.get("pressed", False)):
            return True
    return False

