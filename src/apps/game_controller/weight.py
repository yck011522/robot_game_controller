"""Weight-sensor helpers for the game controller runtime."""

from __future__ import annotations

import time
from typing import Any

from core.device_connection import load_serial_settings
from subsystems.weight_sensor.common import BUCKET_CELL_MAP


def _initial_weight_state(*, enabled: bool) -> dict[str, Any]:
    """Return the GameController's local weight-sensor cache."""

    return {
        "enabled": enabled,
        "cells_g": {},
        "cell_ok": {},
        "errors": {},
        "bucket_cell_map": _load_weight_bucket_cell_map(),
        "last_recv_mono_s": None,
        "tare_seq": 0,
        "cycle_seq": 0,
    }


def _update_weight_state(state: dict[str, Any], body: dict[str, Any]) -> None:
    """Store the latest twelve-cell `telem.weight` payload."""

    cells = body.get("cells_g") if isinstance(body, dict) else None
    ok = body.get("cell_ok") if isinstance(body, dict) else None
    errors = body.get("errors") if isinstance(body, dict) else None
    state["cells_g"] = _coerce_number_map(cells)
    state["cell_ok"] = _coerce_bool_map(ok)
    state["errors"] = (
        {str(k): str(v) for k, v in errors.items()} if isinstance(errors, dict) else {}
    )
    state["tare_seq"] = _coerce_int(
        body.get("tare_seq"), int(state.get("tare_seq", 0) or 0)
    )
    state["cycle_seq"] = _coerce_int(
        body.get("cycle_seq"), int(state.get("cycle_seq", 0) or 0)
    )
    state["last_recv_mono_s"] = time.perf_counter()


def _apply_weight_bucket_values(
    team_state: dict[str, Any], weight_state: dict[str, Any]
) -> None:
    """Update one team's bucket values from live load-cell sums during play."""

    if not bool(weight_state.get("enabled", False)):
        return
    values = _bucket_values_from_weight(team_state["team"], weight_state)
    if values is None:
        return
    team_state["bucket_values"] = values


def _bucket_values_from_weight(
    team: str, weight_state: dict[str, Any]
) -> list[float] | None:
    """Return three 1-based bucket sums for a team from the latest 12 cells."""

    cells_g = weight_state.get("cells_g")
    if not isinstance(cells_g, dict) or not cells_g:
        return None
    bucket_cell_map = weight_state.get("bucket_cell_map")
    if not isinstance(bucket_cell_map, dict):
        return None
    values: list[float] = []
    for label in _team_bucket_labels(team):
        cell_ids = bucket_cell_map.get(label, [])
        total_g = 0.0
        for cell_id in cell_ids:
            total_g += max(0.0, float(cells_g.get(str(cell_id), 0.0)))
        values.append(total_g)
    return values[:3]


def _state_full_weight_sensor(weight_state: dict[str, Any]) -> dict[str, Any]:
    """Build the compact `state.full.weight_sensor` block."""

    return {
        "enabled": bool(weight_state.get("enabled", False)),
        "cells_g": dict(weight_state.get("cells_g", {})),
        "cell_ok": dict(weight_state.get("cell_ok", {})),
        "bucket_cell_map": dict(weight_state.get("bucket_cell_map", {})),
        "errors": dict(weight_state.get("errors", {})),
        "tare_seq": int(weight_state.get("tare_seq", 0) or 0),
        "cycle_seq": int(weight_state.get("cycle_seq", 0) or 0),
        "last_recv_mono_s": weight_state.get("last_recv_mono_s"),
    }


def _load_weight_bucket_cell_map() -> dict[str, list[int]]:
    """Load physical load-cell to logical-bucket wiring from device config."""

    settings = load_serial_settings().get("weight_sensor", {})
    raw = settings.get("bucket_cells") if isinstance(settings, dict) else None
    source = raw if isinstance(raw, dict) else BUCKET_CELL_MAP
    out: dict[str, list[int]] = {}
    for label, fallback_cells in BUCKET_CELL_MAP.items():
        cells = (
            source.get(label, fallback_cells)
            if isinstance(source, dict)
            else fallback_cells
        )
        if not isinstance(cells, (list, tuple)):
            cells = fallback_cells
        out[label] = [int(cell) for cell in list(cells)[:2]]
    return out


def _team_bucket_labels(team: str) -> list[str]:
    """Return 1-based logical bucket labels for a team."""

    prefix = "A" if str(team).lower() == "a" else "B"
    return [f"{prefix}1", f"{prefix}2", f"{prefix}3"]


def _coerce_number_map(value: Any) -> dict[str, float]:
    """Coerce a JSON object of numeric values into a string-keyed float map."""

    if not isinstance(value, dict):
        return {}
    out: dict[str, float] = {}
    for key, item in value.items():
        try:
            out[str(key)] = float(item)
        except (TypeError, ValueError):
            out[str(key)] = 0.0
    return out


def _coerce_bool_map(value: Any) -> dict[str, bool]:
    """Coerce a JSON object into a string-keyed bool map."""

    if not isinstance(value, dict):
        return {}
    return {str(key): bool(item) for key, item in value.items()}


def _coerce_int(value: Any, default: int) -> int:
    """Coerce one value to int with a fallback."""

    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)