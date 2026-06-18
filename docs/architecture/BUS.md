# Bus

Concrete ZMQ wire-level spec for the Game Controller PC.

Status: **CONFIRMED — subject to revision after first integration tests.**
Last reviewed: 2026-06-02.

See [SYSTEM_MAP.md](SYSTEM_MAP.md) for the process catalog and edge
characterization. This document fixes ports, sockets, topics, and
payload shapes.

---

## 1. Topology — one shared bus via XPUB/XSUB proxy

We run one tiny **broker process** that hosts an `XSUB ↔ XPUB` proxy.
Every publisher `connect()`s its PUB to the broker's XSUB; every
subscriber `connect()`s its SUB to the broker's XPUB. This means:

- **One pair of well-known endpoints** that every process knows. New
  processes plug in without anyone else changing config.
- **No bind/connect ordering issues.** Producers and consumers both
  connect; the broker is the only binder.
- **EventRecorder is trivial:** one SUB with `setsockopt(SUBSCRIBE, b"")`.
- **bus_tap is trivial:** same pattern, one connect, prints everything.
- Broker is ~10 lines (`zmq.proxy(xsub, xpub)`), runs in its own process,
  and dies cleanly with the supervisor.

Two specialized channels stay off the main bus because they have
different semantics:

- **Collision REQ/REP** uses its own ROUTER/DEALER broker (load balancing).
- **UI → GC commands** use a direct REQ/REP socket (synchronous ack).

```
   publishers ──PUB──> tcp://127.0.0.1:5550 (XSUB) ─┐
                                                    │  zmq.proxy
   subscribers <─SUB── tcp://127.0.0.1:5551 (XPUB) ─┘   (BusBroker)

   JoggingPlanner(s) ──REQ──> :5560 (ROUTER) ──DEALER──> :5561 <── CollisionWorker × 16

   GamemasterUI ──REQ──> :5570 (REP) ── GameController
```

---

## 2. Endpoint table

| # | Endpoint | Socket | Bound by | Purpose |
|---|----------|--------|----------|---------|
| 1 | `tcp://127.0.0.1:5550` | XSUB | `BusBroker` | All publishers connect here |
| 2 | `tcp://127.0.0.1:5551` | XPUB | `BusBroker` | All subscribers connect here |
| 3 | `tcp://127.0.0.1:5560` | ROUTER | `CollisionBroker` | JoggingPlanner REQ clients connect here |
| 4 | `tcp://127.0.0.1:5561` | DEALER | `CollisionBroker` | CollisionWorker REP workers connect here |
| 5 | `tcp://127.0.0.1:5570` | REP | `GameController` | GamemasterUI REQ client connects here |
| 6 | `tcp://0.0.0.0:5552` | PUB (external) | `Vision PC`, `Audio PC` | External PCs connect their PUB to the bus broker on the LAN-facing XSUB (heartbeats only, see §11) |
| 7 | `http://<vision_pc>:8080` | HTTP server | external Vision PC | `EventRecorder` GETs per-game files from here after game end (see §11) |
| 8 | `http://<audio_pc>:8080` | HTTP server | external Audio PC | `EventRecorder` GETs per-game files from here after game end (see §11) |
| 9 | 5590–5599 | — | reserved | future expansion |

Ports below 5550 and above 5599 are free for ad-hoc tools.

> All addresses move to `ipc://` later if TCP overhead shows up in
> profiling. The transport string is one config entry; the rest is
> unchanged.

---

## 3. Wire format

Every message on the main bus is a **two-frame ZMQ multipart**:

```
frame 0  topic string         e.g. b"telem.haptic.a"
frame 1  JSON body (UTF-8)    e.g. b'{"ts_mono_ns":..., "dials":[...]}'
```

- Topic is plain ASCII bytes. SUB filtering is a byte-prefix match, so
  topic naming follows §5.
- Body is JSON-encoded with `json.dumps(..., separators=(",",":"))`.
  Revisit msgpack/orjson only if profiling shows a hotspot.
- Every body is a JSON **object** (never a bare array/string/number) so
  fields can be added without a breaking change.

Helper functions live in `src/core/bus.py`:

```python
def publish(sock, topic: str, body: dict) -> None: ...
def recv(sock) -> tuple[str, dict]: ...
```

### 3.1 Standard envelope fields

Every body includes at minimum:

| Field | Type | Required | Meaning |
|-------|------|----------|---------|
| `ts_wall_ns` | int | yes on `state.full` and `heartbeat.*`; recommended elsewhere | Wall-clock ns since Unix epoch (UTC), from `time.time_ns()`. NTP-synced across all machines. This is the **alignment clock** — the only timestamp safe to compare across processes / machines and the one the replay tool uses to merge state with external skeleton / audio files. |
| `ts_mono_ns` | int | yes | Per-process monotonic ns from `time.monotonic_ns()`. Origin is arbitrary and process-local. Use only for this producer's own jitter / loop-rate stats and for single-hop latency estimates on the same machine. Never compare across processes. |
| `producer` | string | yes | Process name (e.g. `"haptic_io.a"`, `"game_controller"`). |
| `seq` | int | optional | Per-publisher monotonic counter. Only added on topics that need drop detection (see §4.3). |

Topic-specific fields live alongside these.

---

## 4. Time

Two clocks are carried on every message; they answer different
questions and must not be mixed.

### 4.1 Wall clock (`ts_wall_ns`) — alignment

- Source: `time.time_ns()`.
- Units: integer nanoseconds since Unix epoch 1970-01-01 UTC.
- Same meaning on every machine, assuming NTP is configured (Controller
  PC, Vision PC, Audio PC all sync to the same source; LAN NTP gives
  ~1 ms accuracy, which is well below our 20 ms tick).
- This is the **only** clock the replay tool uses to align `state.full`
  with the skeleton and audio files pulled from external PCs (see
  §11) and with the per-game ledger in `recordings/index.jsonl`.
- Why integer ns and not an ISO-8601 string: 8 bytes vs ~30, fits a
  C-long, sorts trivially, no timezone parsing. Human-readable
  timestamps are computed at display time from this field.

### 4.2 Monotonic clock (`ts_mono_ns`) — local jitter only

- Source: `time.monotonic_ns()`.
- Units: integer nanoseconds since an arbitrary origin chosen by the
  OS at process start.
- **Origin is different in every process and on every machine.** Do
  not subtract `ts_mono_ns` produced by one process from
  `time.monotonic_ns()` measured in another, except as a rough
  same-machine latency estimate.
- Used for: producer's own loop-rate / jitter stats (reported in
  `heartbeat.*`), single-hop latency dashboards on the same machine.
- Never written into long-lived logs as an absolute time.

### 4.3 Game-relative time

The GameController carries two reference timestamps on **every**
`state.full` snapshot (they are stable between stage transitions, so
there is no separate stage-change topic):

| Field | Set when | Clock | Meaning |
|-------|----------|-------|---------|
| `tutorial_entered_mono_ns` | Idle → Tutorial | GC monotonic | Origin for "seconds since Tutorial", computed inside GC as `(now_mono_ns - tutorial_entered_mono_ns) / 1e9`. Useful as a label inside GC and on the dashboard; do not export. `null` until the first Tutorial entry of the run. |
| `tutorial_entered_wall_ns` | Idle → Tutorial | wall (UTC ns) | Wall-clock origin for the run. External PCs start their per-game capture at this moment, so this is the timestamp the replay tool uses to align skeleton / audio files with `state.full`. Also the row key written into `recordings/index.jsonl`. `null` until the first Tutorial entry of the run. |

Subscribers that care about stage edges detect them themselves by
comparing `state.full.stage` against the previous snapshot.

### 4.4 Sequence numbers (drop detection)

Add `seq` only where drops are interesting:

- `state.full` — yes (UI / recorder can detect missed snapshots).
- `req.collision_check` / `rep.collision_result` — yes (request id for
  out-of-order replies when using DEALER both sides).
- Everything else — skip until a need appears.

---

## 5. Topic catalog

Topics follow the conventions in [SYSTEM_MAP.md §8](SYSTEM_MAP.md). The
`<team>` placeholder is literally `a` or `b`. Square brackets mean the
suffix is present only on per-team variants.

### 5.1 State (GC is the only publisher)

There is exactly one state topic. Subscribers needing finer-grained
internals read from the `telem.*` and `cmd.*` topics below.

| Topic | Rate | Body summary |
|-------|------|--------------|
| `state.full` | 50 Hz | Authoritative game snapshot (see §6.1). Includes stage, scores, bucket weights, per-team robot/haptic summaries, safety/buttons, and the `tutorial_entered_mono_ns` / `tutorial_entered_wall_ns` reference timestamps. |

The GC ↔ JoggingPlanner pipeline does **not** produce `state.*` topics.
JP subscribes to `telem.haptic.<team>` directly and publishes its
planned output as `cmd.robot.target.<team>` (§5.3). When JP runs
in-process inside GC, neither of those topics goes on the bus.

### 5.2 Telemetry (I/O processes → GC)

| Topic | Producer | Rate |
|-------|----------|------|
| `telem.haptic.<team>` | `HapticIO[.<team>]` | 50 Hz |
| `telem.robot.actual.<team>` | `RobotIO[.<team>]` | 100 Hz |
| `telem.weight` | `WeightSensorIO` | as fast as the 12-cell Modbus cycle allows |
| `telem.bucket` | `BucketController` | 5 Hz (state) |
| `telem.buttons` | `ButtonController` | 50 Hz |
| `telem.safety` | `SafetyBarrierController` | 50 Hz |

### 5.3 Commands (GC → I/O; UI → GC is REQ/REP, see §7)

| Topic | Producer | Consumer | Rate / pattern |
|-------|----------|----------|----------------|
| `cmd.haptic.<team>` | `GameController` | `HapticIO[.<team>]` | 50 Hz, CONFLATE |
| `cmd.robot.target.<team>` | `JoggingPlanner[.<team>]` (or GC if JP is in-process) | `RobotIO[.<team>]` | 100 Hz |
| `cmd.bucket` | `GameController` | `BucketController` | on demand |
| `cmd.weight.tare` | `GameController` | `WeightSensorIO` | startup/reset |

### 5.4 Request / reply (collision check)

Off the main bus. See §8.

### 5.5 Heartbeats

| Topic | Producer | Rate |
|-------|----------|------|
| `heartbeat.<proc>` | every long-lived process | 1 Hz |

`<proc>` is the value of the `PROC_NAME` env var the supervisor injects.
Per-team processes use names like `haptic_io.a`, `robot_io.b`.

---

## 6. Payload schemas

JSON shapes for the topics that have non-obvious bodies. Fields not
listed are reserved for future use.

### 6.1 `state.full`

Authoritative game snapshot, published by GameController at 50 Hz. This
is the **fat snapshot** every UI / LED / broadcaster consumer reads. It
is intentionally redundant with the finer-grained `telem.*` topics —
`state.full` is the authoritative view; `telem.*` is the raw trace.
Subscribers that care about stage edges detect them by comparing
`stage` against the previous snapshot.

```jsonc
{
  // ---- envelope ----------------------------------------------------
  // Wall clock, integer nanoseconds since Unix epoch 1970-01-01 UTC.
  // Source: time.time_ns() on the GameController PC. NTP-synced
  // across all machines (Controller PC, Vision PC, Audio PC). This is
  // the ONLY clock that aligns across processes and machines, so it
  // is the clock the replay tool uses to merge state.full with the
  // skeleton / audio files pulled from external PCs.
  "ts_wall_ns": 1717400000000000000,

  // Per-process monotonic clock, integer nanoseconds. Source:
  // time.monotonic_ns(). Origin is arbitrary and DIFFERENT in every
  // process and on every machine. Use ONLY for:
  //   - this producer's own jitter / loop-rate measurement, or
  //   - a single-hop latency estimate by a direct consumer on the
  //     same machine (consumer's monotonic_ns - this value).
  // Never compare ts_mono_ns across processes; never log-align with it.
  "ts_mono_ns": 1234567890,

  "producer": "game_controller",  // canonical process name (see CONFIG.md §3)
  "seq": 12345,                   // monotonic per-publisher counter; gaps = dropped snapshots

  // ---- stage -------------------------------------------------------
  "stage": "paused",              // "idle" | "tutorial" | "play" | "paused" | "conclusion" | "reset"
  "paused": true,                  // convenience mirror for UI / display consumers
  "pause_reason": "b:protective_stop", // null when not paused; examples: admin_pause, estop, barrier_open, b:protective_stop
  "stage_t_s": 7.42,              // seconds since current stage entered (monotonic, GC-local)

  // Reference timestamps the replay tool and dashboards use to anchor
  // game-relative time. Both are set together when GC first enters
  // Tutorial (this is the moment external PCs start their per-game
  // capture, so it is the right origin for skeleton / audio alignment).
  // `tutorial_entered_mono_ns` is GC-local monotonic (only useful
  // inside GC and for offsetting other GC-local monotonic readings).
  // `tutorial_entered_wall_ns` is wall ns UTC and is the timestamp
  // written into recordings/index.jsonl.
  "tutorial_entered_mono_ns": 999000000000,       // null until first Tutorial entry of the run
  "tutorial_entered_wall_ns": 1717400000000000000, // null until first Tutorial entry of the run

  // Stable identifier for the current game, set when GC first enters
  // Tutorial. Format: "YYYY-MM-DD_HH-MM-SS_run-NNN" (local time of GC
  // PC; the suffix disambiguates same-second runs after a restart).
  // External PCs use this string to name their per-game capture
  // folder; recorder uses it to name the per-game folder under
  // recordings/games/.
  "game_id": "2026-06-02_19-12-03_run-042",   // null until Tutorial entry

  // ---- game state --------------------------------------------------
  "active_teams": ["a"],          // subset of ["a", "b"]; matches CONFIG.md active_teams
  "score": {"a": 142, "b": 0},    // integer points per team

  // Weight per bucket, grams. Keys are bucket IDs from the RS-485
  // load-cell network (see SYSTEM_MAP.md §3 WeightSensorIO).
  // 1x = team A buckets, 2x = team B buckets.
  "buckets": {
    "11": 32.1, "12": 50.7, "13": 60.0,
    "21":  0.0, "22":  0.0, "23":  0.0
  },

  // Per-team robot summary. `null` for an inactive team. All joint
  // values are radians (positions) or radians/second (velocities), in
  // URDF joint order (6 elements for UR10e). Actual values come from
  // the RTDE receive interface (actual_q, actual_qd); planned values
  // are what JoggingPlanner most recently sent to RobotIO this tick.
  "robots": {
    "a": {
      "connected":  true,                 // RTDE link up
      "q_target":   [0, 0, 0, 0, 0, 0],   // raw target from haptic stack (rad)
      "q_planned":  [0, 0, 0, 0, 0, 0],   // JoggingPlanner position output sent to robot (rad)
      "qd_planned": [0, 0, 0, 0, 0, 0],   // JoggingPlanner velocity output (rad/s); 0 when planner emits position-only commands
      "q_actual":   [0, 0, 0, 0, 0, 0],   // measured joint position from RTDE actual_q (rad)
      "qd_actual":  [0, 0, 0, 0, 0, 0]    // measured joint velocity from RTDE actual_qd (rad/s)
    },
    "b": null
  },

  // Per-team haptic summary. Arrays have one entry per dial (6).
  // bounds_min / bounds_max are the artificial software limits that
  // GameController is currently telling the haptic dial firmware to
  // enforce (via cmd.haptic.<team>). They can change at runtime
  // (e.g. Tutorial widens them, potential collision narrows them),
  // so they live in state.full rather than in the static YAML config.
  "haptic": {
    "a": {
      "connected":  [true, true, true, true, true, true],   // ESP32 board link up
      "board_loop_hz": [200, 200, 198, 200, 200, 200],      // per-board measured loop rate, Hz
      "dial_pos":   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],         // dial angle, rad
      "dial_vel":   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],         // dial angular velocity, rad/s
      "bounds_min": [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14], // active soft lower bound per dial, rad
      "bounds_max": [ 3.14,  3.14,  3.14,  3.14,  3.14,  3.14]  // active soft upper bound per dial, rad
    }
  },

  // Safety inputs.
  //   barrier.channels  -- 8 raw light-barrier beams in the persistent
  //                        channel order configured in device_ports_and_addr.yaml.
  //                        true = beam unbroken / normally-closed input HIGH.
  //   barrier.ok        -- final decision checked by GC and RobotIO.
  //                        Bypass config is applied before this reaches
  //                        state.full; stale/error state forces ok false.
  //   estop.pressed     — logical e-stop assertion status as reported
  //                       by ButtonController. The field is normalized
  //                       to logical semantics; downstream consumers do
  //                       not see raw normally-closed wiring polarity.
  "safety": {
    "barrier": {
      "ok": true,
      "channels": [true, true, true, true, true, true, true, true],
      "stale": false,
      "errors": []
    },
    "estop": {"pressed": false}
  },

  // Two global admin button stations, positioned at opposite corners
  // of the setup. Both stations carry the same controls and any press
  // on either station is treated as the same command. Each station has
  // two momentary normally-closed buttons (start_resume, reset) and
  // one normally-closed latching e-stop mushroom. ButtonController
  // debounces and inverts the raw contact polarity so the bus always
  // exposes logical states here.
  "buttons": {
    "left":  {"start_resume": false, "reset": false, "estop": false},
    "right": {"start_resume": false, "reset": false, "estop": false}
  }
}
```

### 6.2 `telem.haptic.<team>`

Latest-wins (CONFLATE on the consumer side). HapticIO publishes at the
boards' native rate (50–200 Hz); GameController only needs the most
recent sample per tick.

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "haptic_io.a",
  "team": "a",
  "dial_pos_rad": [.., .., .., .., .., ..],
  "dial_vel_rad_s": [.., .., .., .., .., ..],
  "board_connected": [true, true, true, true, true, true],
  "board_loop_hz": [200, 200, 198, 200, 200, 200]
}
```

### 6.3 `cmd.haptic.<team>`

Latest-wins (CONFLATE).

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "game_controller",
  "team": "a",
  "tracking_target_rad": [.., .., .., .., .., ..],
  // High-rate updates only carry the tracking target plus current bounds.
  // OOB kick enable/amplitude remain infrequent parameter writes on the
  // haptic board, not part of the per-tick runtime stream.
  // Active soft bounds the dial firmware should enforce this tick.
  // Mirrors state.full.haptic.<team>.bounds_min/max so the dial
  // doesn't need to subscribe to state.full. Per-dial, radians.
  "bounds_min_rad": [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14],
  "bounds_max_rad": [ 3.14,  3.14,  3.14,  3.14,  3.14,  3.14]
}
```

### 6.4 `cmd.haptic.reseat.<team>`

Sparse, explicit command. Used when GameController wants HapticIO to issue
the firmware `R` command and digitally reseat the dial positions to a known
pose. This is separate from the high-rate `cmd.haptic.<team>` stream.

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "game_controller",
  "team": "a",
  "current_pos_rad": [.., .., .., .., .., ..]
}
```

### 6.5 `telem.robot.actual.<team>`

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "robot_io.a",
  "seq": 5512345,
  "team": "a",
  "q_rad":   [.., .., .., .., .., ..],
  "qd_rad_s":[.., .., .., .., .., ..],
  "rtde_ok": false,
  "robot_status": {
    "rtde_ok": false,
    "receive_ok": true,
    "control_ok": false,
    "fault_active": true,
    "fault_reason": "protective_stop",
    "protective_stopped": true,
    "emergency_stopped": false,
    "safety_stopped": false,
    "program_running": false,
    "robot_mode": 7,
    "robot_status": 1,
    "safety_mode": 3,
    "safety_status_bits": 3076,
    "last_send_error": "RTDE control script is not running!"
  }
}
```

### 6.5 `cmd.robot.target.<team>`

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "jogging_planner.a",
  "team": "a",
  "q_target_rad": [.., .., .., .., .., ..]
}
```
### 6.6 `telem.weight`

```jsonc
{
  "ts_mono_ns": ...,
  "ts_wall_ns": ...,
  "producer": "weight_sensor_io",
  "seq": 123,
  "connected": true,
  "cycle_seq": 45,
  "tare_seq": 2,
  "last_tare_reason": "conclusion_reset",
  "slave_addresses": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
  "decimal_places": {"1": 0, "2": 0, "...": 0},
  "tare_offsets_g": {"1": 770.0, "2": 4556.0, "...": 3314.0},
  "cells_g": {"1": 0.0, "2": 0.0, "...": 0.0},
  "raw_i32": {"1": 770, "2": 4556, "...": 3314},
  "cell_ok": {"1": true, "2": true, "...": true},
  "errors": {},
  "last_cycle_duration_ms": 124.0,
  "observed_cycle_hz": 8.0
}
```

### 6.6.1 `cmd.weight.tare`

Sparse command telling `weight_sensor_io` to collect its configured tare
sample window and apply those offsets internally. Downstream consumers
continue to receive already-tared `cells_g` values.

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "game_controller",
  "request_id": "weight-tare-3",
  "reason": "conclusion_reset"
}
```

### 6.7 `telem.buttons`

Two global admin button stations ("left" and "right" corners), both
read by the single `button_controller` process. Either station's press
is treated as the same command. The physical contacts may be
normally-closed, but this topic is already normalized to logical
`pressed` semantics, so `pressed: true` always means the operator is
asserting that control.

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "button_controller",
  "stations": {
    "left": {
      "start_resume": {"pressed": false, "edge": null}, // "edge" ∈ null | "rise" | "fall"
      "reset":        {"pressed": false, "edge": null},
      "estop":        {"pressed": false, "edge": null}
    },
    "right": {
      "start_resume": {"pressed": false, "edge": null},
      "reset":        {"pressed": false, "edge": null},
      "estop":        {"pressed": false, "edge": null}
    }
  }
}
```

Logical meaning:

- `start_resume` is the single acknowledge path. In `idle` it starts a run (`idle -> tutorial`). In `paused` it resumes only if every blocking condition is clear: barrier OK, e-stop physically unlatched, and any recoverable robot fault either already cleared or clearable by the robot recovery hook.
- `reset` aborts the current run and enters `reset`. It clears round-scoped game state but does not unlatch a physical e-stop or silently clear a safety stop.
- `estop` is level-triggered, not edge-triggered. While asserted, GameController keeps the game in `paused` and inhibits all robot motion. Releasing the latching mushroom only removes the interlock; the game still waits for a later `start_resume` edge before motion resumes.

### 6.8 `telem.safety`

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "safety_barrier_controller",
  // 8 raw light-barrier beams. true = beam unbroken / normally-closed
  // input HIGH. ok is the final bypass-aware decision; consumers check ok.
  "ok": true,
  "channels": [true, true, true, true, true, true, true, true],
  "effective_channels": [true, true, true, true, true, true, true, true],
  "channel_labels": ["SBarr11", "SBarr12", "SBarr21", "SBarr22",
                     "SBarr31", "SBarr32", "SBarr41", "SBarr42"],
  "bypass_channels": {
    "SBarr11": false, "SBarr12": false, "SBarr21": false, "SBarr22": false,
    "SBarr31": false, "SBarr32": false, "SBarr41": false, "SBarr42": false
  },
  "errors": []
}
```

### 6.8.1 `cmd.bucket` / `telem.bucket`

`cmd.bucket` is sparse and accepted by the single shared
`bucket_controller` process. Single-bucket commands use 1-based logical
labels (`A1`..`A3`, `B1`..`B3`) or `team` + `bucket_number`.

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "game_controller",
  "action": "open",              // open | close | stop | open_all | close_all | stop_all
  "team": "b",                   // optional for *_all commands
  "bucket_number": 2,            // 1, 2, or 3; never zero-based
  "bucket_label": "B2",          // optional explicit label
  "request_id": "bucket-12",
  "reason": "conclusion_bucket_counted"
}
```

`telem.bucket` reports controller status, watchdog state, and observed
status-scan timing for RS-485 saturation tuning.

```jsonc
{
  "ts_mono_ns": ...,
  "ts_wall_ns": ...,
  "producer": "bucket_controller",
  "seq": 123,
  "connected": true,
  "active_count": 1,
  "status_poll_interval_s": 0.5,
  "last_scan_duration_ms": 12.4,
  "observed_status_scan_hz": 2.0,
  "buckets": {
    "B2": {
      "address": 5,
      "status": {"raw": 144, "state": "limit", "direction": "negative",
                 "speed": 0, "is_moving": false, "at_limit": true,
                 "description": "Negative limit reached"},
      "active_command": null,
      "last_result": {"ok": true, "label": "B2", "action": "open",
                      "request_id": "bucket-12",
                      "message": "open completed: Negative limit reached"},
      "last_error": null
    }
  }
}
```

### 6.9 `heartbeat.<proc>`

```jsonc
{
  "ts_mono_ns": ...,
  "ts_wall_ns": ...,
  "producer": "haptic_io.a",
  "pid": 12345,
  "loop_hz": 199.4,
  "loop_jitter_ms_p95": 1.2,
  "queue_depth": 0
}
```

---

## 7. Admin/UI → GC commands (REQ/REP at `:5570`)

The pygame UI sends commands that need an acknowledgment. These commands
should mirror the same logical admin actions as the physical button
stations so there is only one control vocabulary in the system. One REQ
on the UI side, one REP in GC's main loop.

Current P4 scope: this path is live for the observer-dashboard controls
only. The broader maintenance verb set remains future work.

Request envelope:

```jsonc
{
  "ts_mono_ns": ...,
  "ts_wall_ns": ...,
  "producer": "gamemaster_ui",
  "request_id": 42,               // echoed by GC; used for retry dedupe
  "action": "soft_estop",        // current action set listed below
  "source": "keyboard"           // e.g. keyboard | mouse
}
```

Reply envelope:

```jsonc
{
  "ts_mono_ns": ...,
  "ts_wall_ns": ...,
  "producer": "game_controller",
  "ok": true,
  "error": null,                  // string when ok=false
  "request_id": 42,
  "source": "keyboard",
  "result": {
    "action": "soft_estop",
    "soft_estop": true,
    "active_stage": "play",
    "last_action": "soft_estop"
  }
}
```

Current action set:

| Action | Effect | Notes |
|--------|--------|-------|
| `play_resume` | Clear the current software pause. | Current dashboard label is "PLAY / RESUME". This is the temporary UI-side acknowledge path in the P4 runtime. |
| `soft_estop` | Assert the software pause / soft e-stop. | This is a software-only pause, not a substitute for a hardwired safety loop. |
| `end_game` | Force the game into conclusion. | Intended for operator control during the observer-dashboard flow. |

Planned but not yet implemented on this socket: `set_stage`,
`adjust_score`, `reload_config`, and any richer maintenance/admin
verbs.

REQ sockets get stuck after a timeout, so the UI must rebuild its REQ
socket on timeout — same gotcha as the collision check (see SYSTEM_MAP
§5). The current dashboard implementation does exactly that.

### 7.1 Acknowledge / resume path

All recoverable pauses use the same path conceptually, but only part of
it is implemented today:

1. A pause cause arrives: `telem.buttons.*.estop`, a barrier breach, the UI `soft_estop` command, or a recoverable robot fault such as `robot_status.fault_reason == "protective_stop"`.
2. GameController enters `state.full.stage = "paused"`, sets `paused = true`, records `pause_reason`, freezes new motion planning, and keeps robot targets pinned to actual pose.
3. The operator clears the physical cause: unlatches the e-stop mushroom, clears the barrier, or inspects and clears the robot protective stop.
4. The operator presses `start_resume` on either admin station. In the current dashboard runtime, the UI-side counterpart is `play_resume`.
5. GameController validates that every resume guard is clear. If a robot recovery hook is configured, this is where it may run the Dashboard `unlock protective stop` / `power on` / `brake release` sequence.
6. Only after that explicit acknowledge edge does GameController return to the prior runnable stage.

This keeps the safety model simple: clearing an interlock never restarts the game by itself; an explicit acknowledge is always required.

---

## 8. Collision REQ/REP (ROUTER ↔ DEALER at `:5560`/`:5561`)

Off the main bus. See [SYSTEM_MAP.md §5](SYSTEM_MAP.md#5-collision-worker-fan-out-and-respawn)
for the broker pattern and respawn behavior.

**Bundled requests.** Each request carries a list of independent joint
configurations to be checked. Configurations in the same bundle are
**not** treated as a trajectory — the worker does not check the path
between consecutive entries, it just evaluates each `q` in isolation
and returns one result per entry in the same order. Bundling exists
because prior measurements showed amortizing the pybullet step + IPC
overhead across N checks is significantly cheaper than N round-trips;
this is re-validated as part of the throughput benchmark scheduled in
NEXT_STEPS.md §5 P8 (see notes there for the bundle-size sweep).

Request (`req.collision_check`, sent on the JP's REQ to `:5560`):

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "jogging_planner.a",
  "request_id": 84231,            // echoed in the reply
  "team": "a",                    // informational; both teams share the pool
  "check_self": true,             // self-collision check (applies to every config)
  "check_world": true,            // world-collision check (applies to every config)
  "configs_rad": [                // 1..N independent joint configurations to evaluate
    [.., .., .., .., .., ..],
    [.., .., .., .., .., ..],
    [.., .., .., .., .., ..]
  ]
}
```

A single-check request is just a bundle of size 1. JoggingPlanner may
choose its bundle size dynamically based on the benchmark results;
workers do not care.

Reply (`rep.collision_result`):

```jsonc
{
  "ts_mono_ns": ...,
  "producer": "collision_worker_07",
  "request_id": 84231,
  "ok": true,                     // false on worker-side failure (e.g. malformed q); see "error"
  "error": null,                  // string when ok=false
  // One result per entry in the request's configs_rad, in the same order.
  "results": [
    {"collision": false, "first_hit": null},
    {"collision": true,  "first_hit": {"link_a": "wrist_3", "link_b": "table"}},
    {"collision": false, "first_hit": null}
  ],
  "compute_ms": 4.2               // total wall time spent inside the worker for this bundle
}
```

---

## 9. Conventions for new topics

When adding a topic later:

1. Pick a name that matches §5's prefixes (`state.` / `telem.` / `cmd.` /
   `req.` / `rep.` / `heartbeat.`).
2. Include the standard envelope fields from §3.1.
3. Per-team topics end in `.a` or `.b`. Global topics have no suffix.
4. If the consumer is a slow one (LED / column / scoreboard), publisher
   stays at native rate and consumer sets `ZMQ_CONFLATE=1` on its SUB.
5. Default to "no `seq`". Add `seq` only when drop detection matters.
6. Document the JSON shape in this file in §6. Do **not** rely on
   readers inferring it from code.

---

## 10. Open items

- Whether to migrate body encoding to msgpack/orjson. Defer until P10
  unless profiling demands it sooner.
- Whether to promote per-scope state topics (`state.score`,
  `state.joints.*`) alongside `state.full` if UI redraws become the
  bottleneck. Defer.

---

## 11. External PCs (Vision / Audio) — out-of-band file transfer

Skeleton tracking and player prosody are **analyzed offline after the
game**, so streaming them over ZMQ during the run is wasted bandwidth.
Instead:

1. **During the run** each external PC writes its data to a local
   per-game folder, named by the `game_id` that GC publishes in every
   `state.full` snapshot. Layout on the external PC is mirror of what
   ends up in the recorder:
   ```
   <vision_pc>/captures/<game_id>/skeleton/red.jsonl
   <vision_pc>/captures/<game_id>/skeleton/blue.jsonl
   <audio_pc>/captures/<game_id>/audio/red_1.jsonl … blue_6.jsonl
   ```
2. **At game end** (Conclusion → Reset transition) GC calls a small
   HTTP `POST /game_ended` on each external PC with
   `{"game_id": "...", "ended_wall_ns": ...}`. The external PC
   acknowledges and closes its files.
3. **Recorder pulls the files** by GET-ing
   `http://<external_pc>:8080/captures/<game_id>/` (directory listing)
   then each file. Files land in the recorder's per-game folder under
   `skeleton/` and `audio/` (see `LOGGING.md`).
4. **Liveness** is the only thing on the realtime bus: each external
   PC PUBs `heartbeat.vision_pc` / `heartbeat.audio_pc` at 1 Hz so the
   dashboard can show an online/offline badge. Heartbeat body is the
   standard schema (§6.9) plus an optional `current_game_id` field so
   the dashboard can warn if the external PC is recording the wrong
   game.

The broker exposes a second XSUB endpoint on `0.0.0.0:5552` for the
LAN-facing PUBs (port 5550 stays bound to localhost only). The XPUB
side (5551) is unchanged — subscribers don't care which side a
message came from.

Failure modes:

- **External PC offline at game end** — GC's `POST /game_ended` fails.
  GC logs the failure and the recorder schedules a retry every minute
  for the next hour. Files arrive late; `index.jsonl` is updated when
  they do.
- **External PC rebooted mid-game** — missing heartbeat in the
  dashboard tells the gamemaster. The captured file for that game will
  be partial or missing; recorder records what it gets.
- **Disk full on external PC** — same as above; external PC is
  responsible for its own disk hygiene.

The HTTP server on each external PC is intentionally tiny
(`http.server` from stdlib is enough). No auth: this is a closed LAN.
