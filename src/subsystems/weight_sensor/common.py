"""Shared weight sensor constants and helpers."""

from __future__ import annotations

from dataclasses import dataclass


LOAD_CELL_IDS = tuple(range(1, 13))
BUCKET_CELL_MAP = {
    "A1": (1, 2),
    "A2": (3, 4),
    "A3": (5, 6),
    "B1": (7, 8),
    "B2": (9, 10),
    "B3": (11, 12),
}


@dataclass(frozen=True, slots=True)
class WeightSensorConfig:
    """Runtime identity and conversion settings for the load-cell bus."""

    slave_addresses: tuple[int, ...]
    zero_count: float
    grams_per_count: float


@dataclass(frozen=True, slots=True)
class WeightReading:
    """One raw and converted load-cell reading."""

    slave_address: int
    raw_i32: int
    grams_raw: float
    grams_tared: float
    ok: bool
    error: str | None = None

