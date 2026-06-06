"""Profile loader + validator.

For P1 this only enforces the rules needed by `bus_smoke.yaml`. Later
phases extend `validate()` with the rest of [CONFIG.md §5](../../docs/architecture/CONFIG.md#5-validation).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
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
    # TODO(buttons): physical admin buttons are deferred until the later
    # hardware bring-up phase; keep the config slot reserved.
    "safety_barrier_controller",
    # TODO(safety): safety barrier hardware is deferred until the later
    # hardware bring-up phase; keep the config slot reserved.
    "event_recorder", "gamemaster_ui", "bus_broker", "collision_broker",
)

POOLED_SUBSYSTEMS = ("collision_workers",)

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_CONFIG_PATH = REPO_ROOT / "config" / "runtime.yaml"


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
        if subsystem in PER_TEAM_SUBSYSTEMS:
            return self.subsystem_impl(subsystem, team=team) is not None
        if subsystem in POOLED_SUBSYSTEMS:
            node = self.subsystems.get(subsystem)
            return isinstance(node, dict) and int(node.get("count", 0)) > 0
        return self.subsystem_impl(subsystem) is not None

    def subsystem_impl(self, subsystem: str, team: str | None = None) -> str | None:
        node = self.subsystems.get(subsystem)
        if subsystem in PER_TEAM_SUBSYSTEMS:
            if team is None:
                raise ValueError(f"{subsystem} is per-team; pass team='a' or 'b'")
            if not isinstance(node, dict):
                return None
            return _normalize_impl_value(node.get(team))
        if isinstance(node, dict):
            return _normalize_impl_value(node.get("impl"))
        return _normalize_impl_value(node)

    def subsystem_float(self, subsystem: str, key: str, default: float | None = None) -> float | None:
        return default_runtime_setting(subsystem, key, default)


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
    if _global_impl(subs.get("bus_broker")) != "real":
        errors.append("subsystems.bus_broker must be 'real' (no other impl exists)")

    hardware = data.get("hardware") or {}
    tuning = data.get("tuning") or {}
    robot_tune = tuning.get("robot") if isinstance(tuning, dict) else None
    robot_hw = hardware.get("robot") if isinstance(hardware, dict) else None

    needs_robot_limits = False
    for t in teams:
        robot_impl = subs.get("robot_io", {}).get(t) if isinstance(subs.get("robot_io"), dict) else None
        planner_impl = subs.get("jogging_planner", {}).get(t) if isinstance(subs.get("jogging_planner"), dict) else None
        if robot_impl is not None or planner_impl is not None:
            needs_robot_limits = True
            break

    if needs_robot_limits:
        if not isinstance(robot_tune, dict):
            errors.append("tuning.robot.q_limits_min_deg and tuning.robot.q_limits_max_deg are required when robot_io or jogging_planner is enabled")
        else:
            _validate_robot_limit_array(robot_tune.get("q_limits_min_deg"), "tuning.robot.q_limits_min_deg", errors)
            _validate_robot_limit_array(robot_tune.get("q_limits_max_deg"), "tuning.robot.q_limits_max_deg", errors)

    for t in VALID_TEAMS:
        robot_impl = subs.get("robot_io", {}).get(t) if isinstance(subs.get("robot_io"), dict) else None
        if robot_impl != "real_rtde":
            continue
        if not isinstance(robot_hw, dict):
            errors.append(f"hardware.robot.{t}.host is required when subsystems.robot_io.{t} is 'real_rtde'")
            continue
        team_hw = robot_hw.get(t)
        if not isinstance(team_hw, dict):
            errors.append(f"hardware.robot.{t}.host is required when subsystems.robot_io.{t} is 'real_rtde'")
            continue
        host = team_hw.get("host")
        if not isinstance(host, str) or not host.strip():
            errors.append(f"hardware.robot.{t}.host must be a non-empty string when subsystems.robot_io.{t} is 'real_rtde'")
        port = team_hw.get("port")
        if port is not None:
            try:
                if int(port) <= 0:
                    errors.append(f"hardware.robot.{t}.port must be > 0 when provided")
            except (TypeError, ValueError):
                errors.append(f"hardware.robot.{t}.port must be an integer when provided")


def _validate_robot_limit_array(value: Any, field: str, errors: list[str]) -> None:
    if not isinstance(value, list):
        errors.append(f"{field} must be a list of 6 degree values")
        return
    if len(value) < 6:
        errors.append(f"{field} must provide 6 values")
        return
    for idx, item in enumerate(value[:6]):
        try:
            float(item)
        except (TypeError, ValueError):
            errors.append(f"{field}[{idx}] must be numeric")


def default_runtime_setting(subsystem: str, key: str, default: float | None = None) -> float | None:
    node = _runtime_subsystems().get(subsystem, {})
    raw = node.get(key, default)
    if raw is None:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


@lru_cache(maxsize=1)
def _runtime_subsystems() -> dict[str, dict[str, Any]]:
    if not RUNTIME_CONFIG_PATH.exists():
        return {}
    data = yaml.safe_load(RUNTIME_CONFIG_PATH.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return {}
    subsystems = data.get("subsystems", data)
    if not isinstance(subsystems, dict):
        return {}
    return {
        str(name): dict(node)
        for name, node in subsystems.items()
        if isinstance(name, str) and isinstance(node, dict)
    }


def _normalize_impl_value(value: Any) -> str | None:
    if value is None or value == "null":
        return None
    if isinstance(value, str):
        stripped = value.strip()
        return stripped or None
    return None


def _global_impl(node: Any) -> str | None:
    if isinstance(node, dict):
        return _normalize_impl_value(node.get("impl"))
    return _normalize_impl_value(node)
