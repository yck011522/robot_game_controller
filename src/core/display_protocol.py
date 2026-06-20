"""Wire format for the UDP game-state fan-out to the player-display Pis.

A single UDP datagram carries the whole ``state.full`` body plus a small
header so a receiver can detect staleness and packet reordering::

    {"v": 1, "seq": <int>, "ts_wall_ns": <int>, "state": {...state.full...}}

Design notes
------------
* The payload is compact UTF-8 JSON (no whitespace) to stay as small as
  possible. The fat state is a few KB, so a datagram may be split into a
  handful of IP fragments on a 1500-byte-MTU LAN. UDP delivery is
  best-effort: receivers keep only the most recent valid datagram and a
  dropped fragment simply means the previous frame is reused.
* ``seq`` is a monotonically increasing counter owned by the broadcaster.
  A receiver can ignore any datagram whose ``seq`` is not newer than the
  last one it accepted to guard against reordering.
* ``ts_wall_ns`` is the broadcaster's wall clock at send time (``time_ns``)
  so a display can show / act on staleness if the feed stops.

This module is the single source of truth shared by ``state_broadcaster``
(sender) and ``display_viewer`` / the on-Pi client (receiver).
"""

from __future__ import annotations

import json
import time
from typing import Any

# Bump whenever the datagram envelope (not the inner state.full schema)
# changes in a backward-incompatible way. Receivers drop mismatched
# versions instead of mis-parsing them.
PROTOCOL_VERSION = 1


def encode_datagram(
    state: dict[str, Any],
    seq: int,
    *,
    ts_wall_ns: int | None = None,
) -> bytes:
    """Serialize one ``state.full`` body into a UDP datagram payload.

    Parameters
    ----------
    state:
        The ``state.full`` body to broadcast verbatim.
    seq:
        Monotonic send counter from the broadcaster (newer == larger).
    ts_wall_ns:
        Wall-clock send time in nanoseconds; defaults to ``time.time_ns()``.

    Returns
    -------
    bytes
        Compact UTF-8 JSON ready to hand to ``socket.sendto``.
    """

    if ts_wall_ns is None:
        ts_wall_ns = time.time_ns()
    envelope = {
        "v": PROTOCOL_VERSION,
        "seq": int(seq),
        "ts_wall_ns": int(ts_wall_ns),
        "state": state,
    }
    # separators removes the default ", " / ": " padding to shrink the wire size.
    return json.dumps(envelope, separators=(",", ":")).encode("utf-8")


def decode_datagram(raw: bytes) -> dict[str, Any] | None:
    """Parse a received datagram, returning ``None`` on any malformed input.

    A return value of ``None`` means "ignore this packet" (truncated,
    non-JSON, or wrong protocol version) so the receiver keeps its last
    good frame instead of crashing on corrupt or partial data.
    """

    try:
        message = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(message, dict):
        return None
    if message.get("v") != PROTOCOL_VERSION:
        return None
    if not isinstance(message.get("state"), dict):
        return None
    return message
