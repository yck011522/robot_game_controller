"""Weight-sensor helpers for the game controller runtime."""

from __future__ import annotations

import time
from typing import Any, Callable

from core.device_connection import load_serial_settings
from subsystems.weight_sensor.common import BUCKET_CELL_MAP

PLAY_TARE_ZERO_TOLERANCE_G = 2.0
PLAY_TARE_RETRY_INTERVAL_S = 0.1
PLAY_TARE_MAX_RETRIES = 10


def _initial_weight_state(*, enabled: bool, min_increment_g: float = 0.0) -> dict[str, Any]:
    """Return the GameController's local weight-sensor cache.

    Args:
        enabled: Whether a real weight-sensor feed is expected for this profile.
        min_increment_g: Per-bucket weight deadband in grams. A bucket whose
            summed live load-cell reading is below this counts as 0 (rejecting
            empty-bucket drift / noise). 0 disables the deadband. Sourced from
            the ``game.score_min_increment_g`` profile tuning.
    """

    return {
        "enabled": enabled,
        "cells_g": {},
        "cell_ok": {},
        "errors": {},
        "bucket_cell_map": _load_weight_bucket_cell_map(),
        "min_increment_g": max(0.0, float(min_increment_g)),
        "last_recv_mono_s": None,
        "tare_seq": 0,
        "cycle_seq": 0,
        # True between publishing the game-start tare request and verifying
        # near-zero tared telemetry. While true, bucket scores stay at zero.
        "play_tare_pending": False,
        # Tare sequence observed when the game-start command was requested; the
        # next accepted telemetry must have a larger sequence number.
        "play_tare_start_seq": 0,
        # Per-cell absolute gram tolerance for accepting a game-start tare.
        "play_tare_zero_tolerance_g": PLAY_TARE_ZERO_TOLERANCE_G,
        # Seconds to wait after an acknowledged but non-zero tare before retry.
        "play_tare_retry_interval_s": PLAY_TARE_RETRY_INTERVAL_S,
        # Automatic retries after the initial game-start tare command.
        "play_tare_max_retries": PLAY_TARE_MAX_RETRIES,
        # Number of retry commands already sent for the current game start.
        "play_tare_retry_count": 0,
        # Monotonic timestamp when the latest game-start tare command was sent.
        "play_tare_last_request_mono_s": None,
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


def _begin_play_weight_tare(
    weight_state: dict[str, Any],
    teams: dict[str, dict[str, Any]],
) -> bool:
    """Start the game-start tare handshake and blank live bucket values.

    Called by ``game_controller`` when a new game is being prepared. The caller
    owns publishing ``cmd.weight.tare``; this helper only marks local state so
    stale load-cell readings cannot be copied into team bucket values while the
    asynchronous tare command is still in flight.

    Args:
        weight_state: Controller-local cache returned by
            :func:`_initial_weight_state`.
        teams: Active team dictionaries whose bucket values are published in
            ``state.full`` and consumed by the scoreboard.

    Returns:
        True when a real/sim weight-sensor feed is enabled and the caller should
        publish the tare command; False when this profile uses seeded bucket
        values and no weight tare is needed.
    """

    if not bool(weight_state.get("enabled", False)):
        return False
    weight_state["play_tare_pending"] = True
    weight_state["play_tare_start_seq"] = int(weight_state.get("tare_seq", 0) or 0)
    weight_state["play_tare_retry_count"] = 0
    weight_state["play_tare_last_request_mono_s"] = None
    for team_state in teams.values():
        bucket_count = len(team_state.get("bucket_values") or []) or 3
        team_state["bucket_values"] = [0.0] * bucket_count
        team_state["score"] = 0
    return True


def _mark_play_weight_tare_published(
    weight_state: dict[str, Any], *, now_s: float
) -> None:
    """Record when the latest game-start tare command was published.

    Called by ``game_controller`` immediately after it sends ``cmd.weight.tare``
    for a game-start tare. The retry timer starts from this timestamp, not from
    the earlier local bucket blanking step.

    Args:
        weight_state: Controller-local cache returned by
            :func:`_initial_weight_state`.
        now_s: Monotonic seconds from ``time.perf_counter()``.
    """

    if bool(weight_state.get("play_tare_pending", False)):
        weight_state["play_tare_last_request_mono_s"] = float(now_s)


def _tick_play_weight_tare_verification(
    weight_state: dict[str, Any],
    *,
    now_s: float,
    publish_tare: Callable[[], None],
) -> str | None:
    """Verify the game-start tare and retry briefly if readings are non-zero.

    Called once per ``game_controller`` tick after fresh ``telem.weight`` has
    been drained. A tare is accepted only when a newer tare sequence has arrived
    and every configured load cell is within ``play_tare_zero_tolerance_g`` of
    zero. If readings stay outside tolerance, the helper republishes the same
    tare command every
    ``play_tare_retry_interval_s`` until ``play_tare_max_retries`` is reached.

    Args:
        weight_state: Controller-local cache returned by
            :func:`_initial_weight_state`.
        now_s: Monotonic seconds from ``time.perf_counter()``.
        publish_tare: Callback that publishes one ``cmd.weight.tare`` command.

    Returns:
        A console-ready warning message when verification gives up, otherwise
        ``None``.
    """

    if not bool(weight_state.get("enabled", False)):
        return None
    if not bool(weight_state.get("play_tare_pending", False)):
        return None
    if not _play_weight_tare_complete(weight_state):
        return None
    ok, detail = _play_tare_cells_near_zero(weight_state)
    if ok:
        weight_state["play_tare_pending"] = False
        return None
    retry_count = int(weight_state.get("play_tare_retry_count", 0) or 0)
    max_retries = int(weight_state.get("play_tare_max_retries", PLAY_TARE_MAX_RETRIES) or 0)
    last_request_s = weight_state.get("play_tare_last_request_mono_s")
    if last_request_s is not None:
        interval_s = float(
            weight_state.get("play_tare_retry_interval_s", PLAY_TARE_RETRY_INTERVAL_S)
            or 0.0
        )
        if float(now_s) - float(last_request_s) < interval_s:
            return None
    if retry_count >= max_retries:
        weight_state["play_tare_pending"] = False
        return (
            "[game_controller] WARNING play-entry weight tare did not settle "
            f"after {max_retries} retries; continuing. {detail}"
        )
    weight_state["play_tare_retry_count"] = retry_count + 1
    weight_state["play_tare_start_seq"] = int(weight_state.get("tare_seq", 0) or 0)
    weight_state["play_tare_last_request_mono_s"] = float(now_s)
    publish_tare()
    return None


def _apply_weight_bucket_values(
    team_state: dict[str, Any], weight_state: dict[str, Any]
) -> None:
    """Update one team's bucket values from live load-cell sums during play."""

    if not bool(weight_state.get("enabled", False)):
        return
    if bool(weight_state.get("play_tare_pending", False)):
        if not _play_weight_tare_complete(weight_state):
            return
        weight_state["play_tare_pending"] = False
    values = _bucket_values_from_weight(team_state["team"], weight_state)
    if values is None:
        return
    team_state["bucket_values"] = values


def _play_weight_tare_complete(weight_state: dict[str, Any]) -> bool:
    """Return True after telemetry reports a newer tare sequence."""

    tare_seq = int(weight_state.get("tare_seq", 0) or 0)
    start_seq = int(weight_state.get("play_tare_start_seq", 0) or 0)
    return tare_seq > start_seq


def _play_tare_cells_near_zero(weight_state: dict[str, Any]) -> tuple[bool, str]:
    """Return whether every configured load cell is close enough to zero."""

    cells_g = weight_state.get("cells_g")
    if not isinstance(cells_g, dict) or not cells_g:
        return False, "no tared cell readings received"
    tolerance_g = float(
        weight_state.get("play_tare_zero_tolerance_g", PLAY_TARE_ZERO_TOLERANCE_G)
        or 0.0
    )
    cell_ids = _configured_weight_cell_ids(weight_state)
    if not cell_ids:
        cell_ids = sorted(str(key) for key in cells_g.keys())
    worst_cell = None
    worst_abs_g = 0.0
    missing: list[str] = []
    for cell_id in cell_ids:
        if cell_id not in cells_g:
            missing.append(cell_id)
            continue
        value_abs_g = abs(float(cells_g.get(cell_id, 0.0)))
        if value_abs_g > worst_abs_g:
            worst_abs_g = value_abs_g
            worst_cell = cell_id
    if missing:
        return False, f"missing cells={missing[:4]} tolerance_g={tolerance_g:.1f}"
    if worst_abs_g <= tolerance_g:
        return True, f"max_abs_g={worst_abs_g:.1f} tolerance_g={tolerance_g:.1f}"
    return (
        False,
        f"cell={worst_cell} max_abs_g={worst_abs_g:.1f} tolerance_g={tolerance_g:.1f}",
    )


def _configured_weight_cell_ids(weight_state: dict[str, Any]) -> list[str]:
    """Return sorted string load-cell IDs from the configured bucket map."""

    bucket_cell_map = weight_state.get("bucket_cell_map")
    if not isinstance(bucket_cell_map, dict):
        return []
    cell_ids: set[str] = set()
    for cells in bucket_cell_map.values():
        if not isinstance(cells, (list, tuple)):
            continue
        for cell_id in cells:
            cell_ids.add(str(cell_id))
    return sorted(
        cell_ids,
        key=lambda item: (0, int(item)) if item.isdigit() else (1, item),
    )


def _bucket_values_from_weight(
    team: str, weight_state: dict[str, Any]
) -> list[float] | None:
    """Return three 1-based bucket sums for a team from the latest 12 cells.

    Each bucket sum is gated by the ``min_increment_g`` deadband stored in
    ``weight_state``: a bucket whose summed reading is below the threshold is
    reported as 0.0 so empty-bucket drift never increments the score. A 0 (or
    missing) threshold disables the gate.
    """

    cells_g = weight_state.get("cells_g")
    if not isinstance(cells_g, dict) or not cells_g:
        return None
    bucket_cell_map = weight_state.get("bucket_cell_map")
    if not isinstance(bucket_cell_map, dict):
        return None
    try:
        min_increment_g = max(0.0, float(weight_state.get("min_increment_g", 0.0)))
    except (TypeError, ValueError):
        min_increment_g = 0.0
    values: list[float] = []
    for label in _team_bucket_labels(team):
        cell_ids = bucket_cell_map.get(label, [])
        total_g = 0.0
        for cell_id in cell_ids:
            total_g += max(0.0, float(cells_g.get(str(cell_id), 0.0)))
        if total_g < min_increment_g:
            total_g = 0.0
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
