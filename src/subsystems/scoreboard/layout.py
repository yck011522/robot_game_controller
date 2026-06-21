"""Load and validate the per-bucket scoreboard panel mapping from device config.

This module turns the installation file ``config/device_ports_and_addr.yaml``
into the runtime objects the scoreboard controller needs: which single COM port
the daisy-chained LED panels live on, and which 1-based panel index each bucket's
text should be routed to.

A "display index" here is the ``<N>`` in the panel command syntax
``/display/<N>/...`` (see ``transport.py``). Bucket labels are ``A1``-``A3`` for
team A and ``B1``-``B3`` for team B, matching the game-controller bucket order
(``buckets[0]`` == bucket 1, etc.).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from core.device_connection import load_device_connection, resolve_serial_ports


# Canonical bucket labels expected in the mapping: three buckets per team, in
# bucket-number order. Used to validate the device file and to translate a
# (team, bucket_index) pair into the label that keys ``bucket_displays``.
_REQUIRED_BUCKET_LABELS = ("A1", "A2", "A3", "B1", "B2", "B3")

# Logical key under ``serial_ports`` in the device file that names the COM port
# the scoreboard panels are wired to.
_SCOREBOARD_PORT_KEY = "score_board"


@dataclass(frozen=True)
class ScoreboardLayout:
    """Validated scoreboard wiring: one COM port plus bucket->panel routing.

    Attributes:
        port: The single resolved COM port the panels are daisy-chained on
            (e.g. ``"COM40"``).
        bucket_displays: Map of bucket label (``"A1"`` ...) to the 1-based panel
            index used in ``/display/<N>/...`` commands.
    """

    port: str
    bucket_displays: dict[str, int]

    def display_for(self, team: str, bucket_index: int) -> int | None:
        """Return the panel index for ``team``'s ``bucket_index`` (0-based).

        Args:
            team: Team letter, ``"a"`` or ``"b"``.
            bucket_index: Zero-based bucket index (0, 1, 2 -> buckets 1, 2, 3).

        Returns:
            The 1-based panel index, or ``None`` if the (team, bucket) pair has
            no configured panel (which should not happen for a valid file).
        """

        label = f"{team.upper()}{bucket_index + 1}"
        return self.bucket_displays.get(label)

    @property
    def all_displays(self) -> tuple[int, ...]:
        """Return every configured panel index exactly once, in numeric order."""

        return tuple(sorted(set(self.bucket_displays.values())))


def load_scoreboard_layout() -> ScoreboardLayout:
    """Load the scoreboard wiring from ``device_ports_and_addr.yaml``.

    Raises:
        ValueError: If the ``scoreboard`` section, the ``bucket_displays`` map,
            or the resolved COM port is missing or malformed. Real hardware
            processes must fail loudly rather than run with implicit defaults.
    """

    root = load_device_connection()
    node = root.get("scoreboard")
    if not isinstance(node, dict):
        raise ValueError("missing scoreboard mapping in device configuration")

    raw_displays = node.get("bucket_displays")
    if not isinstance(raw_displays, dict):
        raise ValueError("missing scoreboard.bucket_displays mapping")

    bucket_displays: dict[str, int] = {}
    for label in _REQUIRED_BUCKET_LABELS:
        if label not in raw_displays:
            raise ValueError(f"scoreboard.bucket_displays missing entry for {label!r}")
        bucket_displays[label] = _coerce_display_index(
            raw_displays[label], f"scoreboard.bucket_displays.{label}"
        )

    # Panel indices must be unique so two buckets never fight over one panel.
    indices = list(bucket_displays.values())
    if len(set(indices)) != len(indices):
        raise ValueError("scoreboard.bucket_displays panel indices must be unique")

    resolved = resolve_serial_ports(_SCOREBOARD_PORT_KEY)
    if len(resolved.ports) != 1:
        raise ValueError(f"{resolved.source} must resolve to exactly one COM port")

    return ScoreboardLayout(port=resolved.ports[0], bucket_displays=bucket_displays)


def _coerce_display_index(value: Any, field: str) -> int:
    """Coerce one ``bucket_displays`` value into a positive 1-based panel index."""

    try:
        index = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be an integer panel index") from exc
    if index < 1:
        raise ValueError(f"{field} must be a 1-based panel index (>= 1)")
    return index
