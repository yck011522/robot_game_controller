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


def clear_cache() -> None:
    """Clear the YAML cache; useful for tests and future discovery writers."""

    _load_com_ports_cached.cache_clear()


def _path_from_arg(path: str | Path | None) -> Path:
    return Path(path) if path is not None else DEFAULT_COM_PORTS_PATH


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
