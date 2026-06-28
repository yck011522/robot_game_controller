# Display Broadcast Protocol

This document defines the UDP game-state feed consumed by the Raspberry Pi
player displays. It describes the protocol implemented by
`src/core/display_protocol.py` and `src/apps/state_broadcaster/__main__.py`.

> **Do not implement against `docs/NETWORK_PROTOCOL.md`.** That file describes
> the retired legacy publisher. The envelope, port, state fields, and player
> addressing in this document are the current contract.

## 1. Transport and endpoint

| Property | Current value | Notes |
|---|---:|---|
| Transport | IPv4 UDP | Connectionless and best-effort; no acknowledgement or retry. |
| Destination | `192.168.0.255` | Directed broadcast for `192.168.0.0/24`. Can be changed to unicast or loopback. |
| Receiver bind | `0.0.0.0` | Bind on all Pi interfaces. Do not bind to the broadcast address. |
| UDP port | `49200` | Every display receiver listens on this port. |
| Normal state rate | approximately 60 Hz | One datagram per new `state.full`; unchanged states are not repeated. |
| Receiver buffer | at least 65,536 bytes | The reference receiver uses `1 << 16`. |
| Encoding | compact UTF-8 JSON | One complete JSON document per UDP datagram. No delimiter or newline. |

The endpoint and Pi mapping are installation settings in
`config/device_ports_and_addr.yaml` under `display_broadcast`. Broadcast traffic
normally remains inside its IP subnet. The Pi firewall must allow inbound UDP
port `49200` from the controller PC.

A state is currently several kilobytes and can exceed the usual 1,500-byte
Ethernet MTU. IP may therefore fragment one UDP datagram. If any fragment is
lost, the entire datagram is lost. A receiver must keep rendering its last good
state rather than blanking the screen after one missed frame.

## 2. Datagram envelope

Each datagram has this exact top-level shape:

```json
{
  "v": 1,
  "seq": 1842,
  "ts_wall_ns": 1782050123456789000,
  "state": {}
}
```

| Field | Type | Meaning |
|---|---|---|
| `v` | integer | Envelope protocol version. The current and only accepted value is `1`. |
| `seq` | integer | Broadcaster-owned datagram counter. It increases once per transmitted state and normally starts at `1` when the broadcaster starts. |
| `ts_wall_ns` | integer | Broadcaster wall-clock send time: nanoseconds since the Unix epoch. Useful for diagnostics, but not for local timeout measurement. |
| `state` | object | The complete `state.full` body described below. |

The envelope `seq` is different from `state.seq`. The outer value orders UDP
datagrams, including replayed frames. The inner value orders states produced by
the game controller. They will often differ and a receiver must not assume they
are equal.

New optional state fields may be added without changing `v`. A receiver should
ignore fields it does not understand. A backward-incompatible change to the
outer envelope requires a new version; a receiver should reject an unsupported
version rather than guess its meaning.

## 3. State shape

The broadcaster places the game controller's `state.full` object into `state`
verbatim. In a normal two-team session it has the following shape. `null` is a
valid value for fields marked nullable, and diagnostic fields should always be
read defensively.

```text
state
|-- ts_mono_ns: integer
|-- ts_wall_ns: integer
|-- producer: string
|-- seq: integer
|-- stage: string
|-- active_stage: string
|-- paused: boolean
|-- pause_reason: string | null
|-- soft_estop: boolean
|-- safety
|   `-- barrier
|       |-- enabled, ok, stale: boolean
|       |-- channels: array
|       `-- errors: array
|-- weight_sensor
|   |-- enabled: boolean
|   |-- cells_g, cell_ok, bucket_cell_map, errors: object
|   |-- tare_seq, cycle_seq: integer
|   `-- last_recv_mono_s: number | null
|-- countdown_s: integer
|-- game_duration_s: number
|-- sum_score_rate_unit_per_s: number
|-- stage_entered_mono_ns: integer
|-- tutorial_entered_wall_ns: integer | null
`-- teams
    |-- a: team object
    `-- b: team object
```

The bus-envelope fields inside `state` have these constraints:

- `state.ts_mono_ns` and `stage_entered_mono_ns` use the controller process's
  monotonic clock. They are meaningful only for differences from the same
  controller process and must not be compared with a Pi's monotonic clock.
- `state.ts_wall_ns` is the controller's wall-clock creation time. The outer
  `ts_wall_ns` is the later broadcaster send time.
- `state.seq` normally begins at `0` and increases per controller state.
- `producer` identifies the game-controller process.

### Stage and pause fields

`active_stage` is the authoritative lifecycle stage and is one of:

| Value | Display meaning |
|---|---|
| `daydreaming` | Attract/screensaver mode. |
| `idle` | Waiting for a player to begin. |
| `tutorial` | Interactive tutorial. |
| `play` | Timed game in progress. |
| `reset` | Game ended; robot rewind/return is in progress. |
| `conclusion` | Bucket counting and final-score presentation. |

When the game is paused, `stage` becomes `"paused"` while `active_stage`
continues to report the preserved lifecycle stage. Therefore, select the page
using `active_stage` and draw pause/E-stop UI as an overlay using `paused`.
`pause_reason` is nullable and intended for operator detail; known values
include `soft_estop`, `recovery`, `barrier_open`, `barrier_stale`,
`barrier_ack_required`, and team-prefixed robot fault reasons. Treat unknown
non-empty values as valid future reasons.

`countdown_s` is the non-negative whole number of seconds remaining in
`tutorial`, `play`, or a timer-driven `reset`. It is `0` in untimed stages and
also during a rewind-driven reset. It stops changing while paused. Do not use
`0` alone to infer that a timed stage has completed; use `active_stage`.

### Team object

The keys under `teams` are lowercase team IDs. A profile can run only one
team, so test for a key before using it. Each present team has this shape:

```text
team
|-- robot
|   |-- q_target_rad: array[number] (6 values)
|   |-- q_rad: array[number] (6 values)
|   `-- status: object
|-- haptic
|   |-- dial_pos_rad, dial_deg, dial_robot_deg, dial_vel_rad_s: array[number] (6 values)
|   |-- connected: array[boolean] (6 values)
|   |-- board_loop_hz: array[number] (6 values)
|   |-- tutorial_progress_pct: array[number] (6 values, nominally 0..100)
|   |-- bounds_min_rad, bounds_max_rad: array[number] (6 values)
|   `-- play_sync: object
|-- collision
|   |-- in_collision: boolean
|   |-- first_hit: object | null
|   |-- path_scalar, prox_scalar, final_scalar: number
|   |-- prox_zones: array (6 per-axis zone objects)
|   |-- prox_probe_offsets_deg: array[number]
|   |-- prox_hits: array
|   `-- prox_age_ticks: array[integer]
|-- planner: object
|-- rewind
|   |-- enabled: boolean
|   |-- status: string
|   |-- recorded_point_count, point_count, current_index: integer
|   |-- progress: number (nominally 0..1)
|   |-- initial_q_rad: array[number] | null
|   |-- max_error_deg: number | null
|   `-- shortcut: object
|-- score: number
|-- summed_score: number
|-- bucket_labels: array[string]
|-- buckets: array[number]
`-- conclusion
    |-- phase: string
    |-- active_bucket_index: integer | null
    |-- target_pose_name: string | null
    |-- target_pose_deg: array[number] | null
    |-- bucket_open_triggered: boolean
    `-- done: boolean
```

For player `aN` or `bN`, select team `a` or `b` and zero-based array index
`N - 1`. For example, player `b4` uses `teams.b.robot.q_rad[3]`,
`teams.b.haptic.dial_deg[3]`, and
`teams.b.haptic.tutorial_progress_pct[3]`. Angles ending in `_rad` are radians;
`dial_deg`, `dial_robot_deg`, `_deg`, and `_pct` fields already carry the units
in their names. `dial_deg` is the raw dial-space angle. `dial_robot_deg` is
that same measured dial position mapped through the configured per-axis
`gear_ratio`, so it reflects robot-space direction flips and scaling.

Display-oriented interpretations:

- `score` is the team's live gameplay score.
- `buckets` and `bucket_labels` are parallel arrays, normally three entries.
- During conclusion, `summed_score` is the total accumulated so far and
  `active_bucket_index` selects the bucket currently being counted.
- `conclusion.done` indicates that final team totals can be shown.
- Collision scalars are nominally `0.0..1.0`; `final_scalar` is the effective
  speed fraction, where `1.0` means full speed.
- `robot.status`, `planner`, `rewind.shortcut`, safety detail, and weight-sensor
  detail are diagnostics. A player UI should not require every diagnostic key
  in order to render its essential screen.

An optional `batch_validation` object may appear at the top of `state` in
automated validation profiles. It is not part of normal display behavior.

### Proximity collision zones

`teams.<team>.collision.prox_zones` is the recommended source for drawing each
joint's nearby collision picture. It is an array of six per-axis objects, one
per robot joint, indexed the same way as the other per-joint arrays (index
`N - 1` for player `aN`/`bN`). The game controller probes a small window of
joint angles just above and below each joint's current position, then collapses
the result into three display bands expressed in **absolute joint degrees**, so
a receiver never has to add the current angle itself.

Each axis object has this shape:

| Field | Type | Meaning |
|---|---|---|
| `valid` | boolean | `false` means there is no fresh collision test for this joint (stale or missing). Draw the whole lane as the neutral "untested" background and ignore the remaining fields. |
| `free_min_deg` | number \| null | Lower edge of the green (collision-free) band, in absolute joint degrees. |
| `free_max_deg` | number \| null | Upper edge of the green band, in absolute joint degrees. |
| `blocked_above_till_deg` | number \| null | Outer (upper) edge of the red band above the green band. `null` means no collision was found above, so there is no red band on that side. |
| `blocked_below_till_deg` | number \| null | Outer (lower) edge of the red band below the green band. `null` means no collision was found below. |

The tested window spans only a few degrees around the current joint angle, so
the red bands intentionally stop at `blocked_above_till_deg` and
`blocked_below_till_deg` rather than continuing to the edge of the lane.
Everything beyond those edges was never collision-checked and must be shown as
the neutral background, not as free space. When a side has no red band, the
green band already reaches the outer edge of the tested window, and the area
beyond it is likewise untested background.

These values share the same space and units as the current joint angle, which is
`teams.<team>.robot.q_rad[N-1]` converted to degrees, so the bands and the
current-position marker can be placed on one common angular scale.

To render one joint lane:

1. Fill the lane with the neutral "untested" background color.
2. If `valid` is `false`, stop here.
3. Draw a green band from `free_min_deg` to `free_max_deg`.
4. If `blocked_above_till_deg` is not `null`, draw a red band from
   `free_max_deg` to `blocked_above_till_deg`.
5. If `blocked_below_till_deg` is not `null`, draw a red band from
   `blocked_below_till_deg` to `free_min_deg`.

This yields the natural cases: a fully free window (green only), a collision
above or below (green plus one red band), or collisions on both sides
(red-green-red). The current joint angle always falls inside the green band and
may be drawn as a position marker on top of the bands.

The raw `prox_probe_offsets_deg`, `prox_hits`, and `prox_age_ticks` fields remain
available for diagnostics, but `prox_zones` already encodes the same information
in display-ready form and is the preferred input for visualization.

## 4. Pi and player assignment

The current installation mapping is:

| Pi hostname | Pi address | Panels/players | Array indices |
|---|---|---|---|
| `rpi5-11` | `192.168.0.11` | `a1`, `a2` | Team A indices 0, 1 |
| `rpi5-12` | `192.168.0.12` | `a3`, `a4` | Team A indices 2, 3 |
| `rpi5-13` | `192.168.0.13` | `a5`, `a6` | Team A indices 4, 5 |
| `rpi5-14` | `192.168.0.14` | `b1`, `b2` | Team B indices 0, 1 |
| `rpi5-15` | `192.168.0.15` | `b3`, `b4` | Team B indices 2, 3 |
| `rpi5-16` | `192.168.0.16` | `b5`, `b6` | Team B indices 4, 5 |

The YAML configuration is authoritative if this table and the installed
wiring ever differ. The hostname lookup is case-insensitive. Each Pi is
expected to drive the two listed player screens; how those logical panels map
to the Pi's physical display connectors is local to the Pi application.

## 5. Receiver requirements

A robust receiver should:

1. Bind an IPv4 UDP socket to `0.0.0.0:49200` and allocate a buffer large
   enough for the entire datagram.
2. Decode UTF-8 and JSON inside exception handling. Ignore malformed,
   truncated, non-object, or unsupported-version packets.
3. Require `v == 1`, integer `seq` and `ts_wall_ns`, and an object `state`.
4. Drain queued datagrams each render cycle and retain only the greatest new
   outer `seq`, so rendering never falls behind the network.
5. Ignore duplicate or lower sequence numbers during an active stream.
6. Track packet freshness using the Pi's monotonic clock at successful receipt.
   Do not depend on synchronized PC/Pi wall clocks.
7. Continue drawing the last good state through brief loss. Show a disconnected
   or signal-lost overlay after about 1 second without a valid new datagram.
8. After a stale interval, allow a lower sequence number to establish a new
   stream. The live broadcaster has no session ID and resets `seq` when its
   process restarts; rejecting all lower values forever would prevent recovery.
9. Treat absent teams, short arrays, `null`, and unknown fields/stages as valid
   degraded input. Never let one unusual value terminate the display process.

The UDP feed is intended for a trusted installation LAN. It is neither
authenticated nor encrypted; do not expose the receiver port to an untrusted
network.

## 6. Minimal standard-library receiver

This example demonstrates protocol handling independently of the display
framework. The render loop can copy the most recent `state` under the lock.

```python
"""Receive Display Broadcast Protocol v1 on a Raspberry Pi.

Run:
    python display_receiver.py
    python display_receiver.py --bind 0.0.0.0 --port 49200
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from typing import Any

PROTOCOL_VERSION = 1  # Outer UDP envelope version accepted by this client.
RX_BUFFER_BYTES = 1 << 16  # Must hold one complete UDP datagram.
SIGNAL_TIMEOUT_S = 1.0  # Delay before the UI reports signal loss.


class StateReceiver:
    """Receive UDP frames in a worker thread and retain the latest valid state."""

    def __init__(self, bind: str, port: int) -> None:
        """Bind the receiver to one local address and UDP port."""

        self.state: dict[str, Any] | None = None  # Last accepted state.full body.
        self.last_seq: int | None = None  # Outer sequence used for reorder rejection.
        self.last_rx_mono: float | None = None  # Local receipt time for staleness.
        self.lock = threading.Lock()  # Protects the three public state variables.
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((bind, port))

    def run(self) -> None:
        """Receive forever; call this method from a daemon worker thread."""

        while True:
            raw, _sender = self.socket.recvfrom(RX_BUFFER_BYTES)
            message = self._decode(raw)
            if message is None:
                continue
            now = time.monotonic()  # Pi-local clock; unaffected by wall-clock changes.
            seq = message["seq"]  # Validated outer datagram sequence.
            with self.lock:
                stale = (
                    self.last_rx_mono is None
                    or now - self.last_rx_mono > SIGNAL_TIMEOUT_S
                )
                if not stale and self.last_seq is not None and seq <= self.last_seq:
                    continue
                self.state = message["state"]
                self.last_seq = seq
                self.last_rx_mono = now

    @staticmethod
    def _decode(raw: bytes) -> dict[str, Any] | None:
        """Return a validated v1 envelope, or None when a packet is unusable."""

        try:
            message = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        if not isinstance(message, dict) or message.get("v") != PROTOCOL_VERSION:
            return None
        if type(message.get("seq")) is not int:  # Reject booleans as integers.
            return None
        if type(message.get("ts_wall_ns")) is not int:
            return None
        if not isinstance(message.get("state"), dict):
            return None
        return message

    def signal_lost(self) -> bool:
        """Return whether no valid new frame has arrived within the timeout."""

        with self.lock:
            return (
                self.last_rx_mono is None
                or time.monotonic() - self.last_rx_mono > SIGNAL_TIMEOUT_S
            )


def main() -> None:
    """Parse network settings and run the UDP receiver until interrupted."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")  # Local receive interface.
    parser.add_argument("--port", type=int, default=49200)  # Shared display port.
    args = parser.parse_args()
    receiver = StateReceiver(args.bind, args.port)
    threading.Thread(target=receiver.run, daemon=True).start()
    while True:
        time.sleep(1.0)
        print("SIGNAL LOST" if receiver.signal_lost() else "receiving")


if __name__ == "__main__":
    main()
```

## 7. Local integration testing

No Pi or robot hardware is needed for a loopback test. Use the repository's
validated Python interpreter and set `PYTHONPATH` so the applications can find
the source tree:

```powershell
$env:PYTHONPATH = "src"

# Terminal 1: receiver/reference UI for Pi rpi5-11.
& C:\Users\yck01\miniconda3\envs\game\python.exe -m apps.display_viewer `
    --host rpi5-11 --bind 127.0.0.1 --port 49200

# Terminal 2: run the game through a profile whose display broadcaster is enabled.
# Override the destination so packets remain on this PC.
& C:\Users\yck01\miniconda3\envs\game\python.exe -m apps.launcher `
    --profile config\profiles\two_teams.yaml
```

The launcher currently obtains the broadcaster destination from
`config/device_ports_and_addr.yaml`; for a local end-to-end run, temporarily
use its documented `dest: "127.0.0.1"` alternative or launch the broadcaster
standalone with `--dest 127.0.0.1`.

Protocol and UDP loopback regression tests can be run without installing new
libraries:

```powershell
$env:PYTHONPATH = "src"
& C:\Users\yck01\miniconda3\envs\game\python.exe -m unittest `
    tests.test_display_broadcast -v
```

For UI development from a captured game, `apps.state_replayer` can send a
`.jsonl.gz` display recording to `127.0.0.1`, one Pi's unicast address, or the
subnet broadcast address while preserving the recorded stage timing.

## 8. Source-of-truth files

- Wire encoder/decoder: `src/core/display_protocol.py`
- UDP sender: `src/apps/state_broadcaster/__main__.py`
- Reference receiver/UI: `src/apps/display_viewer/__main__.py`
- State schema construction: `src/apps/game_controller/published_states.py`
- Endpoint and Pi mapping: `config/device_ports_and_addr.yaml`
- Regression tests: `tests/test_display_broadcast.py`

