"""Profile loader + validator.

For P1 this only enforces the rules needed by `bus_smoke.yaml`. Later
phases extend `validate()` with the rest of [CONFIG.md §5](../../docs/architecture/CONFIG.md#5-validation).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


VALID_TEAMS = ("a", "b")

PER_TEAM_SUBSYSTEMS = ("haptic_io", "robot_io", "jogging_planner")

GLOBAL_SUBSYSTEMS = (
    "weight_sensor_io",
    "light_column_1_3", "light_column_4_5", "light_column_6_8",
    "display_broadcaster", "scoreboard_broadcaster",
    "bucket_controller", "button_controller",
    "safety_barrier_controller",
    "event_recorder", "gamemaster_ui", "bus_broker",
)

POOLED_SUBSYSTEMS = ("collision_workers",)


class ConfigError(ValueError):
    """Raised on profile validation failure. Message lists every problem."""


@dataclass(frozen=True)
class Profile:
    """Validated profile, exposed as plain attribute access.

    `raw` is the original parsed dict (used by impls that need fields the
    dataclass does not yet model — keeps additive schema changes from
    needing a code change here).
    """
    name: str
    description: str
    active_teams: tuple[str, ...]
    subsystems: dict[str, Any]
    tuning: dict[str, Any] = field(default_factory=dict)
    hardware: dict[str, Any] = field(default_factory=dict)
    recorder: dict[str, Any] = field(default_factory=dict)
    path: Path | None = None
    raw: dict[str, Any] = field(default_factory=dict)

    def is_enabled(self, subsystem: str, team: str | None = None) -> bool:
        """Return True iff this subsystem is configured to spawn."""
        node = self.subsystems.get(subsystem)
        if subsystem in PER_TEAM_SUBSYSTEMS:
            if team is None:
                raise ValueError(f"{subsystem} is per-team; pass team='a' or 'b'")
            return node is not None and node.get(team) is not None
        if subsystem in POOLED_SUBSYSTEMS:
            return isinstance(node, dict) and int(node.get("count", 0)) > 0
        return node is not None and node != "null"


def load(path: str | Path) -> Profile:
    """Load + validate a profile YAML. Raises ConfigError on bad data."""
    p = Path(path).resolve()
    if not p.exists():
        raise ConfigError(f"profile not found: {p}")
    data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    errors: list[str] = []
    _validate(data, errors)
    if errors:
        raise ConfigError("invalid profile {}:\n  - {}".format(p, "\n  - ".join(errors)))

    return Profile(
        name=data.get("profile_name", p.stem),
        description=data.get("description", ""),
        active_teams=tuple(data.get("active_teams") or ()),
        subsystems=dict(data.get("subsystems") or {}),
        tuning=dict(data.get("tuning") or {}),
        hardware=dict(data.get("hardware") or {}),
        recorder=dict(data.get("recorder") or {}),
        path=p,
        raw=data,
    )


def _validate(data: dict[str, Any], errors: list[str]) -> None:
    # active_teams: subset of [a, b], may be empty (bus_smoke uses []).
    teams = data.get("active_teams")
    if teams is None:
        errors.append("missing 'active_teams'")
        teams = []
    elif not isinstance(teams, list):
        errors.append("'active_teams' must be a list")
        teams = []
    else:
        for t in teams:
            if t not in VALID_TEAMS:
                errors.append(f"active_teams contains invalid team {t!r}; must be in {list(VALID_TEAMS)}")

    subs = data.get("subsystems")
    if not isinstance(subs, dict):
        errors.append("'subsystems' must be a mapping")
        return

    # Per-team subsystems: teams not in active_teams must be null;
    # teams in active_teams must be non-null. (CONFIG.md §5.2)
    for name in PER_TEAM_SUBSYSTEMS:
        node = subs.get(name)
        if not isinstance(node, dict):
            errors.append(f"subsystems.{name} must be a mapping {{a: ..., b: ...}}")
            continue
        for t in VALID_TEAMS:
            val = node.get(t)
            if t in teams and val is None:
                errors.append(f"subsystems.{name}.{t} must be set when team {t!r} is active")
            if t not in teams and val is not None:
                errors.append(f"subsystems.{name}.{t} must be null when team {t!r} is not active")

    # Collision workers: count >= 0.
    cw = subs.get("collision_workers")
    if cw is not None:
        if not isinstance(cw, dict) or "count" not in cw:
            errors.append("subsystems.collision_workers must be {count: N}")
        else:
            try:
                if int(cw["count"]) < 0:
                    errors.append("subsystems.collision_workers.count must be >= 0")
            except (TypeError, ValueError):
                errors.append("subsystems.collision_workers.count must be an integer")

    # bus_broker is always required at runtime (CONFIG.md §3.2).
    if subs.get("bus_broker") != "real":
        errors.append("subsystems.bus_broker must be 'real' (no other impl exists)")
