# Network Protocol — Game State Broadcast

This document describes the UDP broadcast mechanism used to publish game state
from the Game Master Controller PC to Raspberry Pi display nodes.  Copy this
file into the display node repository as the definitive protocol reference.

---

## Architecture Overview

```
┌─────────────────────┐        UDP Broadcast (one-way)        ┌─────────────────┐
│  Game Controller PC │ ───────────────────────────────────►  │  RPi Display #1 │
│  192.168.1.1        │  dest 255.255.255.255:9000            │  192.168.1.10   │
│  (StatePublisher)   │                                       ├─────────────────┤
└─────────────────────┘                                       │  RPi Display #2 │
         │                                                    │  192.168.1.11   │
         │  Gigabit Ethernet                                  ├─────────────────┤
         │  POE+ Unmanaged Switch                             │       ...       │
         └──────────────────────────────────────────────────► │  RPi Display #6 │
                                                              │  192.168.1.15   │
                                                              └─────────────────┘
```

- **Transport**: UDP (connectionless, no acknowledgement)
- **Direction**: one-way, controller → displays only
- **Addressing**: subnet broadcast — no per-device addressing required
- **Frequency**: 50 Hz default (configurable in `GameSettings.publish_hz`)
- **Payload**: JSON, UTF-8, compact (`separators=(",",":")`)
- **Typical payload size**: ~600 bytes

### Why UDP Broadcast

| Property | Benefit |
|----------|---------|
| Connectionless | RPis can join or reboot at any time and immediately start receiving |
| Broadcast | One send reaches all devices; no per-device routing |
| Fire-and-forget | Missed packets are silently skipped; next packet arrives within 20 ms |
| No broker | Zero infrastructure beyond a network switch |

---

## Network Setup

### Physical

- All devices connected to the same unmanaged Gigabit POE+ switch
- No router between devices (direct LAN)
- RPis can be powered via POE if the switch supports POE+

### IP Addressing

| Device | Recommended IP | Notes |
|--------|---------------|-------|
| Game Controller PC | `192.168.1.1` | Static, set in Windows adapter settings |
| RPi #1 | `192.168.1.10` | Static or DHCP — does not matter for broadcast |
| RPi #2 | `192.168.1.11` | |
| … | … | |
| RPi #6 | `192.168.1.15` | |

- **Subnet mask**: `255.255.255.0`
- **Broadcast address**: `192.168.1.255` (or `255.255.255.255` for limited broadcast)
- The RPis do **not** need a known IP for receiving broadcasts; only the subnet
  membership matters

#### Setting a static IP on Raspberry Pi OS (bookworm)

```bash
# /etc/dhcpcd.conf
interface eth0
static ip_address=192.168.1.10/24
```

Or using NetworkManager:
```bash
nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.10/24 ipv4.method manual
nmcli con up "Wired connection 1"
```

---

## Packet Format

Each packet is a single UDP datagram containing a UTF-8 JSON object.

### Top-level keys

| Key | Type | Description |
|-----|------|-------------|
| `v` | int | Protocol version. Current: `1` |
| `ts` | float | Unix timestamp of the packet (seconds since epoch) |
| `stage` | string | Current game stage: `"Idle"`, `"Tutorial"`, `"GameOn"`, `"Conclusion"`, `"Reset"` |
| `countdown_s` | int | Seconds remaining in the current stage |
| `estop` | bool | Emergency stop active |
| `joints` | object | Per-joint state, keyed by motor ID string |
| `scores` | object | Team scores and high score |
| `buckets` | object | Per-bucket raw weight (grams), keyed by bucket ID string |
| `multipliers` | object | Score multiplier per bucket (static config) |
| `health` | object | System health metrics (optional — may be ignored by display) |

### `joints` object

Keys are motor ID strings `"11"` through `"16"` (Team 1 joints).

| Sub-key | Type | Description |
|---------|------|-------------|
| `dial_deg` | float | Rate-limited clamped dial position (degrees) |
| `robot_deg` | float | Actual robot joint position (degrees) |

*Future fields (will be added without a version bump when available):*
- `velocity_dps` — joint angular velocity (°/s)
- `torque_nm` — joint torque (N·m)

### `scores` object

| Sub-key | Type | Description |
|---------|------|-------------|
| `team1` | float | Current Team 1 score |
| `team2` | float | Current Team 2 score |
| `high` | float | All-time high score (this session) |
| `high_holder` | string | `"Team 1"` or `"Team 2"` |

### `buckets` object

Keys are bucket ID strings. Bucket IDs: `"11"`, `"12"`, `"13"` (Team 1),
`"21"`, `"22"`, `"23"` (Team 2). Values are weights in grams (float).

### `health` object

| Sub-key | Type | Description |
|---------|------|-------------|
| `game_loop_hz` | float | Game loop frequency |
| `robot_physics_hz` | float | Robot physics thread frequency |
| `haptic_connected` | string | e.g. `"6/6"` |
| `weight_sensors` | string | e.g. `"6/6"` |
| `publisher_hz` | float | Actual UDP publish frequency |

### Example packet (formatted for readability)

```json
{
  "v": 1,
  "ts": 1741651200.123,
  "stage": "GameOn",
  "countdown_s": 87,
  "estop": false,
  "joints": {
    "11": {"dial_deg": 12.5, "robot_deg": 11.8},
    "12": {"dial_deg": -45.0, "robot_deg": -43.2},
    "13": {"dial_deg": 0.0, "robot_deg": 0.0},
    "14": {"dial_deg": 30.0, "robot_deg": 29.1},
    "15": {"dial_deg": -10.0, "robot_deg": -9.8},
    "16": {"dial_deg": 5.0, "robot_deg": 4.9}
  },
  "scores": {
    "team1": 1250.0,
    "team2": 980.0,
    "high": 1250.0,
    "high_holder": "Team 1"
  },
  "buckets": {
    "11": 200.0, "12": 150.0, "13": 100.0,
    "21": 180.0, "22": 120.0, "23": 80.0
  },
  "multipliers": {
    "11": 1.0, "12": 2.0, "13": 3.0,
    "21": 1.0, "22": 2.0, "23": 3.0
  },
  "health": {
    "game_loop_hz": 99.8,
    "robot_physics_hz": 199.6,
    "haptic_connected": "6/6",
    "weight_sensors": "6/6",
    "publisher_hz": 50.1
  }
}
```

---

## Receiver Implementation (Raspberry Pi)

### Minimal Python receiver

```python
import socket
import json
import threading
import time

PORT = 9000

# Shared state dict — updated by receiver thread, read by display thread
state = {}
state_lock = threading.Lock()
last_packet_time = 0.0


def receiver_thread():
    global last_packet_time
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", PORT))
    sock.settimeout(0.5)

    while True:
        try:
            data, _ = sock.recvfrom(4096)
            packet = json.loads(data)
            with state_lock:
                state.update(packet)
                last_packet_time = time.time()
        except socket.timeout:
            pass  # No packet this cycle — display will show stale data
        except json.JSONDecodeError:
            pass  # Malformed packet — ignore


threading.Thread(target=receiver_thread, daemon=True).start()
```

### Detecting stale / lost signal

```python
STALE_THRESHOLD_S = 0.5  # show "NO SIGNAL" after 500 ms without a packet

def is_signal_fresh() -> bool:
    return (time.time() - last_packet_time) < STALE_THRESHOLD_S
```

### Pygame display loop skeleton

```python
import pygame

pygame.init()
screen = pygame.display.set_mode((1080, 1920))  # vertical 1080p display
clock = pygame.Clock()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            raise SystemExit

    with state_lock:
        snap = dict(state)  # shallow copy for thread safety

    screen.fill((0, 0, 0))

    if not is_signal_fresh():
        # Draw "NO SIGNAL" overlay
        pass
    else:
        stage = snap.get("stage", "Idle")
        countdown = snap.get("countdown_s", 0)
        joints = snap.get("joints", {})
        scores = snap.get("scores", {})
        # ... draw your UI here ...

    pygame.display.flip()
    clock.tick(60)  # 60 FPS display, independent of 50 Hz receive rate
```

### Per-player display assignment

Each RPi controls two screens (two players).  Use the RPi's own IP address
(or a config file) to determine which two motor IDs and bucket IDs to display:

| RPi IP | Controls screens for | Motor IDs | Bucket IDs |
|--------|---------------------|-----------|------------|
| `192.168.1.10` | Player 11, Player 12 | 11, 12 | 11, 21 |
| `192.168.1.11` | Player 13, Player 14 | 13, 14 | 12, 22 |
| `192.168.1.12` | Player 15, Player 16 | 15, 16 | 13, 23 |
| `192.168.1.13` | Player 21, Player 22 | 11, 12 | — |
| … | … | … | … |

*(Adjust assignment to match your physical layout.)*

---

## Protocol Versioning

The `v` field in every packet is the protocol version integer.

- **Version 1** (current): all fields described above
- A receiver that sees `v > 1` should warn the operator but can continue
  reading known fields — new fields are additive and backward-compatible
- Removing or renaming existing fields requires a version bump

To change the version, update `PROTOCOL_VERSION` in `src/state_publisher.py`.

---

## Configuration

All parameters are in `GameSettings` and can be changed at runtime:

| Setting | Default | Description |
|---------|---------|-------------|
| `broadcast_addr` | `"255.255.255.255"` | UDP broadcast destination |
| `broadcast_port` | `9000` | UDP port |
| `publish_hz` | `50.0` | Publish frequency |

To use a subnet-specific broadcast instead of the limited broadcast:

```python
settings.set("broadcast_addr", "192.168.1.255")
```

The `StatePublisher.publish_hz` property can also be set at runtime without
restarting the publisher thread:

```python
publisher.publish_hz = 100.0
```
