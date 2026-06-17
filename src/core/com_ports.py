"""Installation-local COM port mapping helpers.

The launcher profile describes what should run. This module reads the
separate hardware-discovery output that says which Windows COM ports belong
to each serial device on this machine.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COM_PORTS_PATH = REPO_ROOT / "config" / "com_ports.yaml"


@dataclass(frozen=True)
class SerialPortResolution:
    """Resolved serial ports plus whether discovery should be skipped."""

    ports: tuple[str, ...]
    configured: bool
    source: str


@dataclass(frozen=True)
class SerialSettingResolution:
    """Resolved serial setting value plus the config location that supplied it."""

    value: Any
    source: str


def resolve_serial_ports(
    profile: Any,
    key: str,
    *,
    path: str | Path | None = None,
) -> SerialPortResolution:
    """Resolve serial ports for a logical device key.

    `config/com_ports.yaml` is authoritative when it contains the key, even
    when the value is an empty list. Profile-local `hardware.serial_ports`
    remains supported as a compatibility fallback; an empty profile value is
    treated as omitted so old development profiles can still auto-discover.
    """

    data = load_com_ports(path)
    if key in data:
        return SerialPortResolution(
            ports=_normalize_port_list(data.get(key)),
            configured=True,
            source=str(_path_from_arg(path)),
        )

    profile_ports = _profile_serial_ports(profile)
    if key in profile_ports:
        ports = _normalize_port_list(profile_ports.get(key))
        if ports:
            return SerialPortResolution(
                ports=ports,
                configured=True,
                source="profile.hardware.serial_ports",
            )

    return SerialPortResolution(ports=(), configured=False, source="")


@lru_cache(maxsize=4)
def _load_com_ports_cached(path_text: str) -> dict[str, Any]:
    path = Path(path_text)
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return {}
    serial_ports = data.get("serial_ports", data)
    if not isinstance(serial_ports, dict):
        return {}
    return {str(k): v for k, v in serial_ports.items()}


def load_com_ports(path: str | Path | None = None) -> dict[str, Any]:
    """Load raw logical-device -> COM-port entries."""

    return dict(_load_com_ports_cached(str(_path_from_arg(path).resolve())))


def load_serial_settings(path: str | Path | None = None) -> dict[str, dict[str, Any]]:
    """Load raw serial settings grouped by protocol or logical device family."""

    data = _load_raw_config(path)
    serial_settings = data.get("serial_settings")
    if not isinstance(serial_settings, dict):
        return {}
    settings_by_key: dict[str, dict[str, Any]] = {}
    for key, value in serial_settings.items():
        if isinstance(value, dict):
            settings_by_key[str(key)] = dict(value)
    return settings_by_key


def require_serial_setting(
    key: str,
    setting: str,
    *,
    path: str | Path | None = None,
) -> SerialSettingResolution:
    """Return one required serial setting from `config/com_ports.yaml`.

    Raises ValueError when the setting is absent so runtime serial code cannot
    silently fall back to a second source of truth.
    """

    settings_by_key = load_serial_settings(path)
    settings = settings_by_key.get(key)
    if settings is None or setting not in settings:
        source = _path_from_arg(path)
        raise ValueError(
            f"missing serial_settings.{key}.{setting} in {source}; "
            "serial connection settings must be configured there"
        )
    return SerialSettingResolution(
        value=settings[setting],
        source=f"{_path_from_arg(path)}:serial_settings.{key}.{setting}",
    )


def require_serial_int(
    key: str,
    setting: str,
    *,
    min_value: int | None = None,
    path: str | Path | None = None,
) -> int:
    """Return a required integer serial setting with basic range validation."""

    resolved = require_serial_setting(key, setting, path=path)
    try:
        value = int(resolved.value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{resolved.source} must be an integer") from exc
    if min_value is not None and value < min_value:
        raise ValueError(f"{resolved.source} must be >= {min_value}")
    return value


def require_serial_float(
    key: str,
    setting: str,
    *,
    min_value: float | None = None,
    path: str | Path | None = None,
) -> float:
    """Return a required float serial setting with basic range validation."""

    resolved = require_serial_setting(key, setting, path=path)
    try:
        value = float(resolved.value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{resolved.source} must be a number") from exc
    if min_value is not None and value < min_value:
        raise ValueError(f"{resolved.source} must be >= {min_value}")
    return value


def require_serial_baudrate(
    key: str,
    *,
    path: str | Path | None = None,
) -> int:
    """Return the required baudrate for a configured serial protocol key."""

    return require_serial_int(key, "baudrate", min_value=1, path=path)


def clear_cache() -> None:
    """Clear the YAML cache; useful for tests and future discovery writers."""

    _load_com_ports_cached.cache_clear()
    _load_raw_config_cached.cache_clear()


def _path_from_arg(path: str | Path | None) -> Path:
    return Path(path) if path is not None else DEFAULT_COM_PORTS_PATH


@lru_cache(maxsize=4)
def _load_raw_config_cached(path_text: str) -> dict[str, Any]:
    path = Path(path_text)
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return data if isinstance(data, dict) else {}


def _load_raw_config(path: str | Path | None = None) -> dict[str, Any]:
    return dict(_load_raw_config_cached(str(_path_from_arg(path).resolve())))


def _profile_serial_ports(profile: Any) -> dict[str, Any]:
    hardware = getattr(profile, "hardware", {})
    if not isinstance(hardware, dict):
        return {}
    serial_ports = hardware.get("serial_ports")
    if not isinstance(serial_ports, dict):
        return {}
    return serial_ports


def _normalize_port_list(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        stripped = value.strip()
        return (stripped,) if stripped else ()
    if not isinstance(value, list):
        return ()
    ports: list[str] = []
    for item in value:
        if not isinstance(item, str):
            continue
        stripped = item.strip()
        if stripped:
            ports.append(stripped)
    return tuple(ports)
