"""Safety barrier helpers for the game controller runtime."""

from __future__ import annotations

import time
from typing import Any


def _initial_safety_state(*, enabled: bool) -> dict[str, Any]:
    """Return the GameController's local safety barrier cache."""

    return {
        "enabled": enabled,
        "ok": True if not enabled else False,
        "channels": [],
        "errors": [],
        "last_recv_mono_s": None,
        "stale": enabled,
    }


def _update_safety_state(state: dict[str, Any], body: dict[str, Any]) -> None:
    """Store the latest `telem.safety` payload and local receipt time."""

    state["ok"] = bool(body.get("ok", False))
    channels = body.get("channels")
    state["channels"] = (
        [bool(value) for value in channels] if isinstance(channels, list) else []
    )
    errors = body.get("errors")
    state["errors"] = (
        [str(value) for value in errors] if isinstance(errors, list) else []
    )
    state["last_recv_mono_s"] = time.perf_counter()
    state["stale"] = False


def _refresh_safety_block(
    control_state: dict[str, Any],
    safety_state: dict[str, Any],
    telem_age_max_s: float,
) -> None:
    """Update safety pause flags from the latest barrier sample and age budget."""

    if not bool(safety_state.get("enabled", False)):
        control_state["safety_blocked"] = False
        safety_state["stale"] = False
        return

    last_recv = safety_state.get("last_recv_mono_s")
    stale = (
        not isinstance(last_recv, float)
        or (time.perf_counter() - last_recv) > telem_age_max_s
    )
    safety_state["stale"] = stale
    blocked = stale or not bool(safety_state.get("ok", False))
    control_state["safety_blocked"] = blocked
    if blocked:
        control_state["soft_pause"] = True
        control_state["safety_pause_latched"] = True


def _safety_pause_reason(safety_state: dict[str, Any]) -> str:
    """Return the pause reason string for the current safety block."""

    if bool(safety_state.get("stale", False)):
        return "barrier_stale"
    return "barrier_open"


def _state_full_safety_barrier(safety_state: dict[str, Any]) -> dict[str, Any]:
    """Build the `state.full.safety.barrier` block for UI and RobotIO consumers."""

    return {
        "enabled": bool(safety_state.get("enabled", False)),
        "ok": bool(safety_state.get("ok", True))
        and not bool(safety_state.get("stale", False)),
        "channels": list(safety_state.get("channels", [])),
        "stale": bool(safety_state.get("stale", False)),
        "errors": list(safety_state.get("errors", [])),
    }