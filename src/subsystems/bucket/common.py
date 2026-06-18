"""Shared bucket controller types and logical bucket-address helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


Direction = Literal["positive", "negative"]
BucketAction = Literal["open", "close", "stop", "open_all", "close_all", "stop_all"]

BUCKET_LABELS = ("A1", "A2", "A3", "B1", "B2", "B3")
TEAM_PREFIX = {"a": "A", "b": "B"}


@dataclass(frozen=True, slots=True)
class MotorStatus:
    """Decoded status for one motor controller register read."""

    raw: int
    state: str
    direction: Direction | None
    speed: int
    is_moving: bool
    at_limit: bool
    description: str


@dataclass(frozen=True, slots=True)
class BucketMotorConfig:
    """Runtime settings that control the six bucket motors."""

    addresses: dict[str, int]
    open_direction: Direction
    close_direction: Direction
    speed: int
    command_timeout_s: float
    status_poll_interval_s: float
    inter_request_delay_s: float


@dataclass(frozen=True, slots=True)
class ActiveBucketCommand:
    """Watchdog state for one in-flight motor command."""

    action: str
    direction: Direction
    request_id: str | int | None
    started_mono_s: float
    timeout_s: float


@dataclass(frozen=True, slots=True)
class BucketCommandResult:
    """Result of accepting, rejecting, or watchdog-stopping a command."""

    ok: bool
    label: str
    action: str
    request_id: str | int | None
    message: str


def bucket_label(team: str, bucket_number: int) -> str:
    """Return the stable human-facing bucket label such as ``B2``."""

    normalized_team = str(team).lower()
    if normalized_team not in TEAM_PREFIX:
        raise ValueError("team must be 'a' or 'b'")
    if bucket_number not in (1, 2, 3):
        raise ValueError("bucket_number must be 1, 2, or 3")
    return f"{TEAM_PREFIX[normalized_team]}{bucket_number}"


def normalize_bucket_label(value: object) -> str:
    """Normalize a caller-provided bucket label and validate it exists."""

    label = str(value).strip().upper()
    if label not in BUCKET_LABELS:
        raise ValueError(f"bucket_label must be one of {list(BUCKET_LABELS)}")
    return label

