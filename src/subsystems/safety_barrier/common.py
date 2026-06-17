"""Shared safety barrier channel mapping and bypass helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


CHANNELS_PER_DEVICE = 2


@dataclass(frozen=True)
class SafetyBarrierConfig:
    """Resolved safety barrier configuration from installation and profile YAML."""

    labels: tuple[str, ...]
    bypass_by_label: dict[str, bool]


@dataclass(frozen=True)
class SafetyBarrierSnapshot:
    """One safety barrier sample after applying the profile bypass policy."""

    ok: bool
    channels: list[bool]
    effective_channels: list[bool]
    labels: list[str]
    bypass_channels: dict[str, bool]
    errors: list[str]


def resolve_safety_barrier_config(
    *,
    channel_order: list[Any],
    bypass_channels: dict[str, Any] | None,
) -> SafetyBarrierConfig:
    """Validate channel labels and normalize profile bypass settings."""

    labels = tuple(str(label).strip() for label in channel_order if str(label).strip())
    if not labels:
        raise ValueError("serial_settings.safety_barrier.channel_order must not be empty")
    bypass_source = bypass_channels or {}
    bypass_by_label = {
        label: bool(bypass_source.get(label, False))
        for label in labels
    }
    return SafetyBarrierConfig(labels=labels, bypass_by_label=bypass_by_label)


def apply_bypass(
    raw_channels: list[bool],
    config: SafetyBarrierConfig,
    *,
    errors: list[str] | None = None,
) -> SafetyBarrierSnapshot:
    """Return raw channels plus the final bypass-aware safety decision."""

    labels = list(config.labels)
    bounded_channels = list(raw_channels[: len(labels)])
    while len(bounded_channels) < len(labels):
        bounded_channels.append(False)

    error_list = list(errors or [])
    effective_channels: list[bool] = []
    for label, raw_ok in zip(labels, bounded_channels):
        # A bypassed barrier is allowed to be physically false while the
        # final ok flag remains true.
        effective_channels.append(bool(raw_ok) or bool(config.bypass_by_label.get(label, False)))

    return SafetyBarrierSnapshot(
        ok=all(effective_channels) and not error_list,
        channels=[bool(value) for value in bounded_channels],
        effective_channels=effective_channels,
        labels=labels,
        bypass_channels=dict(config.bypass_by_label),
        errors=error_list,
    )

