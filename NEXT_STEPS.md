# Next Steps — Architecture Refactor

**Purpose:** Temporary working file to (1) capture feature inventory decisions and
(2) hand off context to a future Copilot session if this conversation ends.

**Status:** Awaiting user markup of the feature inventory in §2.

**Created:** 2026-06-02

---

## 1. How to use this file

For each feature item in §2:
- Append one of: `KEEP`, `REMOVE`, `RENAME → <new name>`, `DEFER`.
- Optionally add a short note after a `—`.
- Delete items that are clearly not applicable (or mark `REMOVE`).
- Add new items at the bottom of any section, or in §2.K "Additions".

When done, tell Copilot: *"NEXT_STEPS.md is ready, proceed."*

---

## 2. Feature inventory (mark up below)

### A. Game lifecycle & flow
1. Five-stage state machine: Idle → Tutorial → GameOn → Conclusion → Reset — `KEEP/REMOVE/RENAME`
2. Auto-cycle arcade mode (24/7 loop) — `KEEP/REMOVE/RENAME`
3. Manual stage override from gamemaster UI — `KEEP/REMOVE/RENAME`
4. Tutorial readiness threshold (min players engaged before advancing) — `KEEP/REMOVE/RENAME`
5. Software emergency stop (supplements hardwired e-stop) — `KEEP/REMOVE/RENAME`
6. Named profiles (Easy / Hard / Demo / Maintenance) — `KEEP/REMOVE/RENAME`
7. Profile scheduling (rotate profiles automatically across games) — `KEEP/REMOVE/RENAME`

### B. Player / control input (haptic)
8. 6 haptic dials over USB serial, ESP32 + FOC motors — `KEEP/REMOVE/RENAME`
9. Per-dial gear ratio (currently global → make per-joint?) — `KEEP/REMOVE/RENAME`
10. Per-joint min/max angle bounds — `KEEP/REMOVE/RENAME`
11. Rate limiter dial-side (`dial_max_velocity_dps`) — `KEEP/REMOVE/RENAME`
12. Rate limiter robot-side (`robot_max_velocity_dps`) — `KEEP/REMOVE/RENAME`
13. OOB kick (enable, amplitude, pulse interval) — `KEEP/REMOVE/RENAME`
14. PD tracking force (`tracking_kp`, `tracking_kd`, `tracking_max_torque`) — `KEEP/REMOVE/RENAME`
15. Hard-stop stiffness (`bounds_kp`) — `KEEP/REMOVE/RENAME`
16. Simulated haptic mode (no hardware) — `KEEP/REMOVE/RENAME`

### C. Robot
17. Simulated robot interface (existing) — `KEEP/REMOVE/RENAME`
18. Real UR10e via RTDE (`incoming_code/rtde_core.py`) — `KEEP/REMOVE/RENAME`
19. JoggingPlanner pipeline (unit → gearing → clamp → rate limit) — `KEEP/REMOVE/RENAME`
20. Collision-aware motion planning (depends on D) — `KEEP/REMOVE/RENAME`
21. Per-robot connection status in UI — `KEEP/REMOVE/RENAME`
22. Latency dashboard (haptic input → robot motion) — `KEEP/REMOVE/RENAME`
23. Two-team robots? (system map implied one UR10e; clarify) — `KEEP/REMOVE/RENAME`

### D. Collision detection
24. pybullet collision check on every planned trajectory — `KEEP/REMOVE/RENAME`
25. Multiple collision workers with load-balancing + respawn — `KEEP/REMOVE/RENAME`
26. Self-collision check — `KEEP/REMOVE/RENAME`
27. World-collision check (table, bucket walls, other team's arm) — `KEEP/REMOVE/RENAME`
28. Collision avoidance toggle in UI — `KEEP/REMOVE/RENAME`

### E. Scoring / weight sensors
29. 6 buckets on RS-485 load cells (IDs 11/12/13 + 21/22/23) — `KEEP/REMOVE/RENAME`
30. Live per-team score from weights — `KEEP/REMOVE/RENAME`
31. Simulated weight sensors — `KEEP/REMOVE/RENAME`
32. Manual score adjustment (sensor fallback) — `KEEP/REMOVE/RENAME`
33. High score display (current run) — `KEEP/REMOVE/RENAME`
34. High score history (derived from `index.jsonl`, no separate DB) — `KEEP/REMOVE/RENAME`

### F. Visual output
35. LED strips per joint — `KEEP/REMOVE/RENAME`
36. LED column (separate device per `LED_COLUMN.md`) — `KEEP/REMOVE/RENAME`
37. Animation library (existing classes) — `KEEP/REMOVE/RENAME`
38. RPi display nodes over UDP broadcast (existing protocol unchanged) — `KEEP/REMOVE/RENAME`
39. Pygame realtime dashboard on the gamemaster PC — `KEEP/REMOVE/RENAME`

### G. Single pygame app — gamemaster + dashboard
40. Joint visualizer (dial / commanded / clamped / rate-limited / actual) — `KEEP/REMOVE/RENAME`
41. Game state + countdown — `KEEP/REMOVE/RENAME`
42. Frequency dashboard (game loop Hz, robot Hz, FOC Hz per motor, serial writer Hz) — `KEEP/REMOVE/RENAME`
43. Per-board connection status (ESP32 / robot / weight sensors / LED) — `KEEP/REMOVE/RENAME`
44. Live-editable parameters (force, gear, bounds, limits) — `KEEP/REMOVE/RENAME`
45. Profile save / load — `KEEP/REMOVE/RENAME`
46. Session log / analytics view (read from `index.jsonl`) — `KEEP/REMOVE/RENAME`
47. Keyboard/mouse lockout in deployment mode (display only, no input) — `KEEP/REMOVE/RENAME`

### H. External telemetry
48. Vision PC → skeleton tracking (red + blue files) — `KEEP/REMOVE/RENAME`
49. Audio PC → prosody (12 player files: red_1..6, blue_1..6) — `KEEP/REMOVE/RENAME`

### I. Infrastructure
50. Config file with named profiles (YAML) — `KEEP/REMOVE/RENAME`
51. Single launcher / supervisor with heartbeat + respawn — `KEEP/REMOVE/RENAME`
52. EventRecorder → per-game folders + `index.jsonl` — `KEEP/REMOVE/RENAME`
53. Replay tool (`tools/replay.py`) — bus playback honoring timestamps — `KEEP/REMOVE/RENAME`
54. Bus tap tool (`tools/bus_tap.py`) — live print of selected topics — `KEEP/REMOVE/RENAME`
55. Per-process crash files (only on exceptions, not 24/7 logs) — `KEEP/REMOVE/RENAME`
56. Hardware-absent dev mode (Real + Sim impl behind same interface, picked by profile) — `KEEP/REMOVE/RENAME`
57. Integration test harness driving the bus — `KEEP/REMOVE/RENAME`

### J. Things in current code to verify
58. `src/enumerate_usb.py`, `src/port_registry.py` — keep as part of HapticIO? Or shared utility? — `KEEP/REMOVE/RENAME`
59. `src/test_led_animation_rate.py`, `src/test_led_comm.py`, `src/test_probe_94.py` — promote to real pytest tests, demote to `tools/`, or delete? — `KEEP/REMOVE/RENAME`
60. `incoming_code/bullet_collision_keyboard_explorer.py` — keep as a `tools/` exploration script after extracting the collision logic into CollisionWorker? — `KEEP/REMOVE/RENAME`

### K. Additions (user-added items)

<!-- Add new feature requests here -->

---

## 3. Decisions already made (do not re-litigate)

These were settled across the planning conversation on 2026-06-01 and
2026-06-02. Resume from here.

### IPC
- **ZeroMQ** is the single inter-process bus on the Game Controller PC.
- Hardware-native protocols (USB serial, RTDE, RS-485) stop at the I/O
  process boundary. Everyone else sees them only via ZMQ topics.
- Payload format: **JSON** for now; revisit MessagePack/orjson only if
  profiling shows a hotspot.
- Patterns used:
  - `PUB/SUB` for state fan-out and telemetry. Use `CONFLATE=1` on slow
    subscribers (LED, UDP broadcaster).
  - `REQ → ROUTER/DEALER → REP` for collision-check workload, load-balanced
    across N pybullet workers.
  - `REQ/REP` for UI → GameController commands (with acks).
  - `SUB-all` for EventRecorder.
- Endpoints: `tcp://127.0.0.1:<port>` locally; same library extends to
  LAN for external Vision/Audio PCs.

### Process model
- One process per timing concern. Threads only inside a process for
  I/O-bound helpers.
- Authoritative state owner: **GameController** (chosen over a separate
  blackboard process to keep the system smaller).
- Process list (target state): see [docs/architecture/SYSTEM_MAP.md](docs/architecture/SYSTEM_MAP.md) §3.
- JoggingPlanner starts **in-process** inside GameController; can be
  promoted to its own process later without changing other processes.

### Frontend
- **Single pygame app** that is both the gamemaster control panel and
  the realtime dashboard. Tk will be retired.
- Same physical PC as the GameController; deployment hides keyboard/mouse
  from the public.
- RPi display nodes remain pure consumers over the existing UDP broadcast
  (protocol unchanged).

### Hardware toggling
- **Named profiles** backed by YAML configs (e.g. `dev`, `bench`, `show`).
- Each subsystem has a `Real` and a `Sim` implementation behind the same
  interface; the profile picks which to instantiate.
- Profile also tells the launcher which processes to start.

### Launcher / Supervisor
- Single launcher spawns all enabled processes per the active profile.
- Each process PUBs `heartbeat.<proc>` at 1 Hz.
- Supervisor monitors heartbeats and respawns crashed processes per
  per-process policy (`always` | `at_game_start` | `never`).
- CollisionWorkers: `always`, optionally `at_game_start` as well.
- GameController: `never` (a GC crash ends the run).
- Respawn implementation can be deferred; the slot exists from day one.

### Logging
- **Drop traditional per-process INFO logs.** 24/7 operation makes them
  noise.
- **EventRecorder** is the only logging system. It writes one folder per
  game, plus a running `recordings/index.jsonl` of completed games with
  metadata + final scores.
- Per-game folder layout:
  ```
  recordings/games/<ts>_run-<id>/
    meta.json         (profile, version, durations, final scores, crash flag)
    bus.jsonl         (every ZMQ message, ts_ns + topic + payload)
    haptic_raw.jsonl  (optional raw serial)
    robot_rtde.jsonl  (raw RTDE samples)
    skeleton/red.jsonl
    skeleton/blue.jsonl
    audio/red_1.jsonl … red_6.jsonl
    audio/blue_1.jsonl … blue_6.jsonl
  ```
  **No video** is recorded (privacy).
- Crash debugging: each process may dump a small
  `recordings/crashes/<proc>_<ts>.txt` on uncaught exceptions only.
  Engineered in later, not first pass.

### Testing strategy
- A. Unit tests per subsystem using Sim impls.
- B. Integration tests on the bus (inject events, assert state).
- C. Replay tests from recorded `bus.jsonl`.
- D. Opt-in hardware smoke tests.

### Source tree shape
- **Hybrid C**: top-level `src/subsystems/{robot,led,haptic,...}` +
  shared `src/core/` (bus, state schema, config, supervisor, recorder) +
  `src/apps/` (one folder per runnable process / frontend).

### Docs cleanup posture
- Move all existing root `.md` files into `docs/`, rewrite the stale
  ones in the same pass, add new architecture docs.

### Environment
- Python env name: **`game`** (existing conda/venv on this machine).
- Deployment OS: **Windows only**.
- Latency priorities: end-to-end haptic→robot loop; LED/column FPS.

---

## 4. Reference files (read these to rebuild context)

Order of reading for a fresh session:

1. **[docs/architecture/SYSTEM_MAP.md](docs/architecture/SYSTEM_MAP.md)** — the
   process catalog, mermaid diagram, edge table, pybullet respawn pattern,
   bus topology, open items. **The authoritative reference for the target
   architecture.**
2. **NEXT_STEPS.md** (this file) — decisions and feature inventory.
3. Existing root markdown files (slated for cleanup, but still describe
   current behavior):
   - [README.md](README.md)
   - [game_mechanics.md](game_mechanics.md)
   - [GAMEMASTER_UI_FEATURES.md](GAMEMASTER_UI_FEATURES.md)
   - [NETWORK_PROTOCOL.md](NETWORK_PROTOCOL.md) — keep this protocol; RPi
     displays are unchanged.
   - [PROTOCOL.md](PROTOCOL.md)
   - [LED_COLUMN.md](LED_COLUMN.md)
   - [LED_QUICKSTART.md](LED_QUICKSTART.md)
   - [TESTING_PLAN.md](TESTING_PLAN.md)
4. Current source (to map onto the target architecture):
   - [src/main.py](src/main.py) — current single-process launcher (threads).
   - [src/game_controller.py](src/game_controller.py) — current GC + game loop.
   - [src/game_settings.py](src/game_settings.py) — current shared state object.
   - [src/state_publisher.py](src/state_publisher.py) — current UDP broadcaster.
   - [src/gamemaster_ui.py](src/gamemaster_ui.py) — current Tk UI to be replaced.
   - [src/jogging_controller.py](src/jogging_controller.py)
   - [src/haptic_serial.py](src/haptic_serial.py)
   - [src/robot_interface.py](src/robot_interface.py)
   - [src/weight_sensor.py](src/weight_sensor.py)
   - [src/led_animation_controller.py](src/led_animation_controller.py)
   - [src/led_animations.py](src/led_animations.py)
   - [src/led_serial.py](src/led_serial.py)
   - [src/led_controller.py](src/led_controller.py)
   - [src/enumerate_usb.py](src/enumerate_usb.py)
   - [src/port_registry.py](src/port_registry.py)
5. Code to integrate (currently outside `src/`):
   - [incoming_code/rtde_core.py](incoming_code/rtde_core.py) — UR10e
     RTDE bridge → becomes `RobotIO` process.
   - [incoming_code/bullet_collision_keyboard_explorer.py](incoming_code/bullet_collision_keyboard_explorer.py)
     — pybullet sandbox → extract collision logic into `CollisionWorker`.
   - [incoming_code/ur10e_robot/](incoming_code/ur10e_robot/) — URDF +
     meshes for the collision worker.
   - [incoming_code/Robot Control Code Implementation Guideline.md](incoming_code/Robot%20Control%20Code%20Implementation%20Guideline.md)

---

## 5. Next steps (resume here)

In order:

### Step 1 — User finishes marking up §2
The user edits this file in place. Copilot waits for the signal to read
it back.

### Step 2 — Produce architecture docs
Once §2 is marked up, generate:

- `docs/architecture/OVERVIEW.md` — narrative architecture overview
  referencing `SYSTEM_MAP.md`. Audience: a new contributor.
- `docs/architecture/BUS.md` — exact ZMQ endpoints, topic names, payload
  schemas (JSON shapes per topic), conventions, port assignments.
- `docs/architecture/LOGGING.md` — EventRecorder design, per-game folder
  spec, `index.jsonl` schema, replay tool spec, no-video policy,
  skeleton/audio file naming.
- `docs/architecture/CONFIG.md` — profile YAML schema, how `Real` vs
  `Sim` is selected, which processes a profile enables.
- `docs/architecture/SUPERVISOR.md` — heartbeat protocol, respawn
  policies per process, shutdown sequence.
- `docs/architecture/subsystems/<name>.md` per kept subsystem (from §2):
  responsibilities, public interface, bus topics in/out, internal threads,
  Real vs Sim impl, testing notes. One file each for at least:
  GameController, HapticIO, RobotIO, WeightSensorIO, LEDController,
  JoggingPlanner, CollisionWorker, GamemasterUI, EventRecorder,
  UDPBroadcaster, Launcher.

### Step 3 — Produce `docs/MIGRATION_PLAN.md`
Phased plan from the current single-process threaded code to the target
architecture. Each phase must be independently runnable and reviewable.
Suggested phases (to be refined):

- **P0 — Repo reshape.** Move root `.md` files into `docs/`, archive
  stale, create empty `src/{core,subsystems,apps}/` skeleton. No
  behavior change.
- **P1 — Introduce ZMQ bus alongside current code.** Add `core/bus.py`,
  publish current GameSettings snapshot on a `state.full` topic, keep
  everything else identical. UI still reads `GameSettings` directly. New
  tool `tools/bus_tap.py` prints traffic.
- **P2 — Split EventRecorder out as its own process.** Subscribe to
  `state.full`. Per-game folder layout in place. `index.jsonl` written.
  No process management yet.
- **P3 — Split UDPBroadcaster out as its own process.** Becomes a pure
  SUB → UDP bridge. RPi protocol unchanged.
- **P4 — Split LEDController out.** First slow subscriber. Validates
  CONFLATE pattern.
- **P5 — Introduce Launcher with profiles** (no respawn yet). YAML
  profile selects which processes start. Real vs Sim impls picked by
  profile.
- **P6 — Split HapticIO out.** First two-way I/O subsystem on the bus.
- **P7 — Split RobotIO out** using `incoming_code/rtde_core.py`. Real
  UR10e becomes available; SimRobot stays for dev.
- **P8 — Introduce CollisionWorker(s) and JoggingPlanner-as-process** (if
  promoted). REQ → ROUTER/DEALER → REP wired up.
- **P9 — Replace Tk gamemaster with pygame app.** Reads state from bus,
  sends commands via REQ/REP.
- **P10 — Add Supervisor heartbeats + respawn.** Per-process policies.
- **P11 — Integrate external Vision/Audio PCs** into EventRecorder.
- **P12 — Crash files + retry-aware collision-check client.**

Each phase ends with a working system. The user should be able to stop
at any phase and still have a usable game.

### Step 4 — Docs cleanup (parallel-able with later phases)
Rewrite stale root `.md` to match the new architecture as part of P0/P1.

---

## 6. Other things worth remembering

- The user prefers **determinism and traceability**. Every design choice
  should be justifiable in terms of "can I reproduce this run, can I
  inspect this message?"
- The user prefers **as many separate processes/threads as practical**,
  each with its own timing cycle, for debugging clarity.
- Do not over-engineer in early phases. Slots for later features
  (respawn, replay, structured logs) should exist, but implementation
  can be deferred until smoke tests reveal real needs.
- Type hints: no strict policy; clarity over rigor.
- The `docs/` folder is currently empty (apart from
  `docs/architecture/SYSTEM_MAP.md` that was just created).
- Conversation thread that produced this file: planning conversation
  started 2026-06-01, system map written 2026-06-02. If resuming in a
  new session, the user does not need a recap — just read the files
  listed in §4 in order.
