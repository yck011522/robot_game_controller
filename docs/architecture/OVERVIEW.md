# Architecture Overview

This is the short version. For details, follow the links into the
companion docs.

Status: **CONFIRMED.** Last reviewed: 2026-06-04.

---

## 1. The system in one paragraph

A two-team arcade robot game runs 24/7 on a single Windows PC (the
"Game Controller PC") plus two LAN peers (Vision PC, Audio PC) and
some RPi display nodes. Each team has 6 haptic dials, one UR10e arm,
and 3 weighing buckets; players turn dials to jog the arm and drop
objects into the buckets for points. The Game Controller PC runs many
small processes that talk to each other over a single ZeroMQ bus.
Hardware-native protocols (USB serial, RTDE, RS-485) terminate at one
"I/O" process each; everyone else sees the world only through bus
topics.

---

## 2. The four ideas everything else follows from

1. **One process per timing concern.** The 50 Hz game loop, the
   100 Hz RTDE loop, the LED column at ~40 Hz, the haptic boards at
   200 Hz — each is a separate OS process with its own loop. Crashes
   are isolated; a hung LED column cannot stall the robot.
2. **One bus, one wire format.** Everything on the Game Controller PC
   talks via a single ZeroMQ XSUB/XPUB broker, JSON bodies, two-frame
   multipart messages. Two off-bus channels exist for specific
   reasons: collision REQ/REP needs load balancing, and UI → GC
   commands need synchronous acks. Details: [BUS.md](BUS.md).
3. **State is owned by the GameController.** GC publishes one fat
   `state.full` snapshot at 50 Hz that any consumer can read. Other
   processes either feed GC raw observations (`telem.*`) or take
   commands from it (`cmd.*`). There is no shared memory, no
   blackboard process.
4. **Profiles pick reality vs simulation, per subsystem.** A single
   YAML file says which processes to spawn, which teams are active,
   and which subsystems use Real vs Sim impls. Same code, different
   profile = same game on a desk with no hardware, on a half-built
   bench, or in the deployed cabinet. Details: [CONFIG.md](CONFIG.md).

---

## 3. Map at a glance

For the canonical diagram, the per-team / shared / global breakdown,
and the edge characterization (rate, latency budget, ZMQ pattern),
see [SYSTEM_MAP.md](SYSTEM_MAP.md). The short version:

- **Per-team (×N active teams):** `HapticIO`, `RobotIO`,
  `JoggingPlanner`.
- **Shared between both teams:** `WeightSensorIO`,
  `ScoreboardBroadcaster`, `BucketController`, `DisplayBroadcaster`,
  `CollisionWorker` pool (16, shared because both robot cells are
  identical).
- **Global (no team concept):** `LightColumnController` (×3 RS-485
  groups), `ButtonController`, `SafetyBarrierController`.
- **Compute / UI / infra:** `GameController`, `GamemasterUI`,
  `EventRecorder`, `Launcher/Supervisor`, `BusBroker`,
  `CollisionBroker`.
- **External LAN peers:** Vision PC and Audio PC. They write per-game
  files locally; the recorder pulls them over HTTP at game end. Only
  a 1 Hz heartbeat each rides the realtime bus.

---

## 4. Lifecycle

A "game" is one pass through the state machine:

```
Idle  →  Tutorial  →  Play  →  Conclusion  →  Reset  →  (back to Idle)
```

- **Idle**: robot + haptic dials slowly play back an animation. Auto-
  cycle arcade mode lives here.
- **Tutorial**: a player engages, robot resets to start, instructions
  show on the displays. Exits when timer expires or all engaged
  players have scrolled to the bottom.
- **Play**: the actual game. Players jog the arm with dials, drop
  objects, score from the load cells.
- **Conclusion**: robot points at each bucket in turn, sums up the
  score, then returns to start.
- **Reset**: short cooldown / cleanup, then back to Idle.

The first transition out of Idle (Idle → Tutorial) is also the
moment GC stamps `game_id` and `tutorial_entered_wall_ns` on
`state.full`; both external PCs and the recorder use those as the
alignment origin for the run.

---

## 5. Time

Two clocks on every message:

- `ts_wall_ns` — UTC nanoseconds since 1970, NTP-synced across all
  machines. The only clock safe to compare across processes or
  machines. Used by the replay tool and `index.jsonl`.
- `ts_mono_ns` — per-process monotonic ns. Origin is arbitrary. Use
  only for the producer's own jitter stats and for a same-machine
  single-hop latency estimate.

Full rules: [BUS.md §4](BUS.md#4-time).

---

## 6. Recording (deferred)

The plan is one folder per game written by `EventRecorder`, plus a
running `recordings/index.jsonl`. No video, no per-process INFO logs.
The full design is in [LOGGING.md](LOGGING.md), but **logging
implementation is deferred** until the system is otherwise demoable
(see [MIGRATION_PLAN.md §1](../MIGRATION_PLAN.md)). Until then,
processes write nothing to disk and the EventRecorder slot in the
profile is a stub that only emits a heartbeat.

---

## 7. Where to go next

Reading order for a new contributor:

1. This file.
2. [SYSTEM_MAP.md](SYSTEM_MAP.md) — what processes exist and how
   they're connected.
3. [BUS.md](BUS.md) — exact wire-level contract.
4. [CONFIG.md](CONFIG.md) — profile schema and Real-vs-Sim selection.
5. [SUPERVISOR.md](SUPERVISOR.md) — startup, heartbeat, respawn.
6. [LOGGING.md](LOGGING.md) — recording design (DRAFT, deferred).
7. [MIGRATION_PLAN.md](../MIGRATION_PLAN.md) — phased path from the
   current single-process code to the target system.
