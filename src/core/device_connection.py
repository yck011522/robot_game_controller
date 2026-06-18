"""Installation-local hardware connection helpers.

Profiles describe behavior for a run. This module reads
`config/device_ports_and_addr.yaml`, which is the single source of truth for
physical COM ports, serial protocol settings, and robot network endpoints.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DEVICE_CONNECTION_PATH = REPO_ROOT / "config" / "device_ports_and_addr.yaml"


@dataclass(frozen=True)
class SerialPortResolution:
    """Resolved serial ports and the config field that supplied them."""

    ports: tuple[str, ...]
    source: str


@dataclass(frozen=True)
class SerialSettingResolution:
    """Resolved serial setting value plus the config field that supplied it."""

    value: Any
    source: str


@dataclass(frozen=True)
class RobotEndpoint:
    """Network endpoint for one real robot."""

    host: str
    port: int
    source: str


def load_device_connection(path: str | Path | None = None) -> dict[str, Any]:
    """Load the raw device connection YAML as a mapping.

    Raises ValueError when the file is absent or malformed so real hardware
    processes cannot continue with implicit defaults.
    """

    return dict(_load_raw_config_cached(str(_path_from_arg(path).resolve())))


def load_serial_ports(path: str | Path | None = None) -> dict[str, Any]:
    """Load the raw `serial_ports` mapping from the device connection file."""

    data = load_device_connection(path)
    serial_ports = data.get("serial_ports")
    if not isinstance(serial_ports, dict):
        raise ValueError(f"missing serial_ports mapping in {_path_from_arg(path)}")
    return {str(key): value for key, value in serial_ports.items()}


def resolve_serial_ports(
    key: str,
    *,
    path: str | Path | None = None,
) -> SerialPortResolution:
    """Resolve the configured COM ports for one logical serial device.

    A present empty list is valid and means "run disconnected; do not scan".
    A missing key is an error because hardware connections are not profile
    fallbacks or runtime guesses.
    """

    serial_ports = load_serial_ports(path)
    if key not in serial_ports:
        raise ValueError(f"missing serial_ports.{key} in {_path_from_arg(path)}")
    return SerialPortResolution(
        ports=_normalize_port_list(serial_ports[key], f"serial_ports.{key}", path=path),
        source=f"{_path_from_arg(path)}:serial_ports.{key}",
    )


def load_serial_settings(path: str | Path | None = None) -> dict[str, dict[str, Any]]:
    """Load serial settings grouped by protocol or logical device family."""

    data = load_device_connection(path)
    serial_settings = data.get("serial_settings", {})
    if not isinstance(serial_settings, dict):
        raise ValueError(f"missing serial_settings mapping in {_path_from_arg(path)}")
    settings_by_key: dict[str, dict[str, Any]] = {}
    for key, value in serial_settings.items():
        if not isinstance(value, dict):
            raise ValueError(f"serial_settings.{key} must be a mapping in {_path_from_arg(path)}")
        settings_by_key[str(key)] = dict(value)
    return settings_by_key


def require_serial_setting(
    key: str,
    setting: str,
    *,
    path: str | Path | None = None,
) -> SerialSettingResolution:
    """Return one required serial setting from the device connection file."""

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
    """Return a required integer serial setting with optional range validation."""

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
    """Return a required float serial setting with optional range validation."""

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


def require_serial_int_list(
    key: str,
    setting: str,
    *,
    min_length: int = 1,
    path: str | Path | None = None,
) -> tuple[int, ...]:
    """Return a required list of integer serial settings."""

    resolved = require_serial_setting(key, setting, path=path)
    if not isinstance(resolved.value, list) or len(resolved.value) < min_length:
        raise ValueError(f"{resolved.source} must be a list with at least {min_length} value(s)")
    values: list[int] = []
    for index, item in enumerate(resolved.value):
        try:
            values.append(int(item))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{resolved.source}[{index}] must be an integer") from exc
    return tuple(values)


def require_robot_endpoint(
    team: str,
    *,
    path: str | Path | None = None,
) -> RobotEndpoint:
    """Return the configured network endpoint for one real robot team."""

    data = load_device_connection(path)
    robots = data.get("robot")
    if not isinstance(robots, dict):
        raise ValueError(f"missing robot mapping in {_path_from_arg(path)}")
    endpoint = robots.get(team)
    if not isinstance(endpoint, dict):
        raise ValueError(f"missing robot.{team} in {_path_from_arg(path)}")
    source = f"{_path_from_arg(path)}:robot.{team}"
    host = endpoint.get("host")
    if not isinstance(host, str) or not host.strip():
        raise ValueError(f"{source}.host must be a non-empty string")
    port = endpoint.get("port")
    try:
        port_int = int(port)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{source}.port must be an integer") from exc
    if port_int <= 0:
        raise ValueError(f"{source}.port must be > 0")
    return RobotEndpoint(host=host.strip(), port=port_int, source=source)


def clear_cache() -> None:
    """Clear the YAML cache; useful for tests and discovery writers."""

    _load_raw_config_cached.cache_clear()


def _path_from_arg(path: str | Path | None) -> Path:
    """Return the explicit path or the project default device config path."""

    return Path(path) if path is not None else DEFAULT_DEVICE_CONNECTION_PATH


@lru_cache(maxsize=4)
def _load_raw_config_cached(path_text: str) -> dict[str, Any]:
    """Read and cache one device connection YAML file."""

    path = Path(path_text)
    if not path.exists():
        raise ValueError(f"device connection config not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"device connection config must be a mapping: {path}")
    return data


def _normalize_port_list(value: Any, field: str, *, path: str | Path | None = None) -> tuple[str, ...]:
    """Normalize one serial port setting to a tuple of non-empty strings."""

    source = f"{_path_from_arg(path)}:{field}"
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            raise ValueError(f"{source} must not be an empty string")
        return (stripped,)
    if isinstance(value, list):
        ports: list[str] = []
        for index, item in enumerate(value):
            if not isinstance(item, str) or not item.strip():
                raise ValueError(f"{source}[{index}] must be a non-empty string")
            ports.append(item.strip())
        return tuple(ports)
    raise ValueError(f"{source} must be a COM port string or a list of COM port strings")
