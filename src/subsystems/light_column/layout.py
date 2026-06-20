"""Load and validate physical/semantic LED strip mappings from device config.

This is the single source of truth that turns the installation file
``config/device_ports_and_addr.yaml`` into the runtime objects the light-column
controller needs: which COM ports exist, which controller addresses live on
each port, and the team / tutorial-player / indicator strip groupings.

Strip IDs are ``address * 10 + channel`` (address 1-8, channel 1-2), e.g. strip
``42`` is controller address 4, channel 2.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from core.device_connection import load_device_connection, resolve_serial_ports


@dataclass(frozen=True)
class LightColumnLayout:
    """Validated arena strip groups plus fixed controller-to-port routing."""

    leds_per_strip: int
    render_hz: float
    wire_color_order: str
    serial_ports: tuple[str, ...]
    controller_addresses_by_port: dict[str, tuple[int, ...]]
    team_strips: dict[str, tuple[int, ...]]
    tutorial_player_strips: dict[str, dict[str, int]]
    team_indicator_strips: dict[str, tuple[int, ...]]

    @property
    def all_strips(self) -> tuple[int, ...]:
        """Return every team-owned strip exactly once in numeric order."""

        return tuple(sorted(set(self.team_strips["a"] + self.team_strips["b"])))


def load_light_column_layout() -> LightColumnLayout:
    """Load the installation mapping from ``device_ports_and_addr.yaml``."""

    root = load_device_connection()
    node = root.get("light_columns")
    if not isinstance(node, dict):
        raise ValueError("missing light_columns mapping in device configuration")
    bus_node = _mapping(node.get("controller_buses"), "light_columns.controller_buses")
    ports: list[str] = []
    addresses_by_port: dict[str, tuple[int, ...]] = {}
    for port_key, raw_addresses in bus_node.items():
        resolved = resolve_serial_ports(str(port_key))
        if len(resolved.ports) != 1:
            raise ValueError(f"{resolved.source} must resolve to exactly one COM port")
        port = resolved.ports[0]
        addresses = _integer_tuple(
            raw_addresses, f"light_columns.controller_buses.{port_key}"
        )
        ports.append(port)
        addresses_by_port[port] = addresses

    semantics = _mapping(
        node.get("semantic_groups"), "light_columns.semantic_groups"
    )
    team_node = _mapping(semantics.get("teams"), "semantic_groups.teams")
    tutorial_node = _mapping(
        semantics.get("tutorial_players"), "semantic_groups.tutorial_players"
    )
    indicator_node = _mapping(
        semantics.get("team_indicators"), "semantic_groups.team_indicators"
    )
    teams = {
        team: _strip_tuple(team_node.get(team), f"semantic_groups.teams.{team}")
        for team in ("a", "b")
    }
    tutorial = {
        team: {
            str(player): strip_id_from_ref(strip_ref)
            for player, strip_ref in _mapping(
                tutorial_node.get(team), f"semantic_groups.tutorial_players.{team}"
            ).items()
        }
        for team in ("a", "b")
    }
    indicators = {
        team: _strip_tuple(
            indicator_node.get(team), f"semantic_groups.team_indicators.{team}"
        )
        for team in ("a", "b")
    }
    all_team_strips = teams["a"] + teams["b"]
    if len(all_team_strips) != 16 or len(set(all_team_strips)) != 16:
        raise ValueError("team strip groups must partition all 16 strips exactly once")
    if str(node.get("wire_color_order", "")).upper() != "RGB":
        raise ValueError("light_columns.wire_color_order must be RGB")
    return LightColumnLayout(
        leds_per_strip=max(1, int(node.get("leds_per_strip", 28))),
        render_hz=max(1.0, float(node.get("render_hz", 40.0))),
        wire_color_order="RGB",
        serial_ports=tuple(ports),
        controller_addresses_by_port=addresses_by_port,
        team_strips=teams,
        tutorial_player_strips=tutorial,
        team_indicator_strips=indicators,
    )


def strip_id_from_ref(value: Any) -> int:
    """Convert a readable ``controller.channel`` reference into strip ID XY."""

    text = str(value).strip()
    parts = text.split(".")
    if len(parts) != 2:
        raise ValueError(f"invalid LED strip reference {value!r}; expected address.channel")
    address, channel = (int(part) for part in parts)
    if address not in range(1, 9) or channel not in (1, 2):
        raise ValueError(f"LED strip reference out of range: {value!r}")
    return address * 10 + channel


def _mapping(value: Any, field: str) -> dict[Any, Any]:
    """Require and return one mapping-valued configuration field."""

    if not isinstance(value, dict):
        raise ValueError(f"{field} must be a mapping")
    return value


def _integer_tuple(value: Any, field: str) -> tuple[int, ...]:
    """Require a nonempty list of unique integer controller addresses."""

    if not isinstance(value, list) or not value:
        raise ValueError(f"{field} must be a nonempty list")
    result = tuple(int(item) for item in value)
    if len(result) != len(set(result)) or any(item not in range(1, 9) for item in result):
        raise ValueError(f"{field} contains duplicate or out-of-range addresses")
    return result


def _strip_tuple(value: Any, field: str) -> tuple[int, ...]:
    """Require a list of readable strip references and convert each one."""

    if not isinstance(value, list):
        raise ValueError(f"{field} must be a list")
    return tuple(strip_id_from_ref(item) for item in value)
