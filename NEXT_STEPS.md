# Next Steps — Architecture Refactor

**Purpose:** Temporary working file to (1) capture feature inventory decisions and
(2) hand off context to a future Copilot session if this conversation ends.

**Status:** Step 1 done; three architecture docs (SYSTEM_MAP, BUS, CONFIG) confirmed.
Resuming at Step 2 — remaining architecture docs, then Step 3 (MIGRATION_PLAN).

**Created:** 2026-06-02
**Last updated:** 2026-06-04

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
1. Five-stage state machine: Idle → Tutorial → GameOn → Conclusion → Reset — `change to Idle (which the robot and the heptic knob will slowly playback an animation) -> triggered by user -> Tutorial (reset robot to initial position / heptic knob in operation, display showing instructions to play) -> triggerd by times up or all user scrolled to bottom. ->  Play state -> Conclusion (score count / reset robot to look at each weighing bucket / sum up score / reset robot to initial position) -> change back to idle mode`
2. Auto-cycle arcade mode (24/7 loop) — `KEEP but the mode change is triggered by user action like i mentioned before`
3. Manual stage override from gamemaster UI — `KEEP (likely by a few physical button read by rs485 io module, to stop game immediately and jump to reset)`
4. Tutorial readiness threshold (min players engaged before advancing) — `KEEP but change behaviour to "wait till times up or untill all player scrolled to bottom"`
5. Software emergency stop (supplements hardwired e-stop) — `KEEP`
6. Named profiles (Easy / Hard / Demo / Maintenance) — `REMOVE`
7. Profile scheduling (rotate profiles automatically across games) — `REMOVE`

### B. Player / control input (haptic)
8. 6 haptic dials over USB serial, ESP32 + FOC motors — `KEEP but 6 dials x 2 teams`
9. Per-dial gear ratio (currently global → make per-joint?) — `KEEP move that to main system yaml config file`
10. Per-joint min/max angle bounds — `KEEP`
11. Rate limiter dial-side (`dial_max_velocity_dps`) — `REMOVE`
12. Rate limiter robot-side (`robot_max_velocity_dps`) — `RENAME to max velocity and max acceleration that is present in the incoming code robot simulation`
13. OOB kick (enable, amplitude, pulse interval) — `KEEP, move that to system config file, this is a config for the haptic feedback knob anyways and wont be changed often`
14. PD tracking force (`tracking_kp`, `tracking_kd`, `tracking_max_torque`) — `KEEP, also move to system config`
15. Hard-stop stiffness (`bounds_kp`) — `KEEP, move to system config`
16. Simulated haptic mode (no hardware) — `REMOVE, user the keyboard input method similar to those in the incoming code keyboard navigator, this is used for testing the game (with other hardware) even without the haptic feedback hardware`


### C. Robot
17. Simulated robot interface (existing) — `REMOVE, use the Pybullet simulator with UI that is in the incoming code.`
18. Real UR10e via RTDE (`incoming_code/rtde_core.py`) — `KEEP, this will become two robots for the two teams `
19. JoggingPlanner pipeline (unit → gearing → clamp → rate limit) — `KEEP`
20. Collision-aware motion planning (depends on D) — `KEEP`
21. Per-robot connection status in UI — `KEEP`
22. Latency dashboard (haptic input → robot motion) — `KEEP, can probably split into a few steps for more details`
23. Two-team robots? (system map implied one UR10e; clarify) — `yes, two robots, team A and team B. `

### D. Collision detection
24. pybullet collision check on every planned trajectory — `KEEP`
25. Multiple collision workers with load-balancing + respawn — `KEEP`
26. Self-collision check — `KEEP`
27. World-collision check (table, bucket walls, other team's arm) — `KEEP, but note that two teams will never touch each other, two teams also have the same robot cell. so the pybullet checker instances can be shared between two teams`
28. Collision avoidance toggle in UI — `collision check (2 types) toggle in system config, should be on by default`

### E. Scoring / weight sensors
29. 6 buckets on RS-485 load cells (IDs 11/12/13 + 21/22/23) — `KEEP`
30. Live per-team score from weights — `KEEP, these will be LED display panels , not the same as the led columns`
31. Simulated weight sensors — `KEEP, as part of the new approach to simulate any missing hardware`
32. Manual score adjustment (sensor fallback) — `KEEP, as part of the new approach to simulate any missing hardware`
33. High score display (current run) — `REMOVE, simply show which team has higher score during end of game`
34. High score history (derived from `index.jsonl`, no separate DB) — `use the main game record ledger to record this info for easy parsing`

### F. Visual output
35. LED strips per joint — `REMOVE`
36. LED column (separate device per `LED_COLUMN.md`) — `KEEP`
37. Animation library (existing classes) — `KEEP`
38. RPi display nodes over UDP broadcast (existing protocol unchanged) — `KEEP`
39. Pygame realtime dashboard on the gamemaster PC — `KEEP`

### G. Single pygame app — gamemaster + dashboard
40. Joint target / current position visualizer (dial position / actual position / velocity as arrow) — `KEEP the version found in the incoming code keyboard explorer`
41. Game state + countdown — `KEEP`
42. Frequency dashboard (game loop Hz, robot Hz, FOC Hz per motor, serial writer Hz) — `KEEP, we need this per every thread or subprocesses that has their own internal timer`
43. Per-board connection status (ESP32 / robot / weight sensors / LED) — `KEEP, make them tiny color boxes with names`
44. Live-editable parameters (force, gear, bounds, limits) — `REMOVE, make them adjustable from yaml`
45. Profile save / load — `REMOVE/
46. Session log / analytics view (read from `index.jsonl`) — `REMOVE`
47. Keyboard/mouse lockout in deployment mode (display only, no input) — `KEEP mouse and keyboard also in deploy. `

### H. External telemetry
48. Vision PC with skeleton tracking (red + blue files) -> main computer — `KEEP`
49. Audio PC with prosody (12 player files: red_1..6, blue_1..6)  -> main computer— `KEEP`

### I. Infrastructure
50. Config file with named profiles (YAML) — `KEEP`
51. Single launcher / supervisor with heartbeat + respawn — `KEEP`
52. EventRecorder → per-game folders + `index.jsonl` — `KEEP`
53. Replay tool (`tools/replay.py`) — bus playback honoring timestamps — `KEEP`
54. Bus tap tool (`tools/bus_tap.py`) — live print of selected topics — `KEEP, make sure whatever UI we use will be fast enough for 50Hz data.`
55. Per-process crash files (only on exceptions, not 24/7 logs) — `keep`
56. Hardware-absent dev mode (Real + Sim impl behind same interface, picked by profile) — `KEEP`
57. Integration test harness driving the bus — `KEEP`

### J. Things in current code to verify
58. `src/enumerate_usb.py`, `src/port_registry.py` — keep as part of HapticIO? Or shared utility? — `KEEP, not shared`
59. `src/test_led_animation_rate.py`, `src/test_led_comm.py`, `src/test_probe_94.py` — promote to real pytest tests, demote to `tools/`, or delete? — `KEEP, move to archive`
60. `incoming_code/bullet_collision_keyboard_explorer.py` — keep as a `tools/` exploration script after extracting the collision logic into CollisionWorker? — `REMOVE`

### K. Additions (user-added items)
Migrate all components to support two teams (team A and B, each has player 1 to 6), each with 6 haptic input devices, 6 multipurpose displays (over 3 Rpis), 1 robot, 3 weighing bucket for scoring, 3 score display board on (on the buckets). Only the LED columns, physical buttons, and the safety barriers are global and not shared. these components can either be sprawning 2 instances of the same type (in 1 vs 2 team mode) or by designing them to have one instance to support both modes. Decision can be mixed and based on whichever has simplier and cleaner implementation

First we will test with only in the game Play state, controlling the 1 robot with keyboard. In a single team mode. The choice of turning team A versus team B on or off is probably in yaml. Either one of the teams or both teams can be turned on or off for testing and debugging purposes. 

We then test other sub systems, you can give me a testing plan for the remaining parts.

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

### Step 1 — User finishes marking up §2 ✅ DONE (2026-06-02)

### Step 2 — Produce architecture docs (in progress)

Status per doc:

- ✅ `docs/architecture/SYSTEM_MAP.md` — confirmed 2026-06-02.
- ✅ `docs/architecture/BUS.md` — confirmed 2026-06-04.
- ✅ `docs/architecture/CONFIG.md` — confirmed 2026-06-04.
- ⏳ `docs/architecture/LOGGING.md` — **next up.** EventRecorder design,
  per-game folder spec, `index.jsonl` schema, replay tool spec,
  no-video policy, external-PC file-pull. Already referenced from
  CONFIG.md `recorder.*` and BUS.md §11 — must land before P4.
- ⏳ `docs/architecture/SUPERVISOR.md` — heartbeat protocol, respawn
  policies per process, shutdown sequence. Already referenced from
  CONFIG.md and SYSTEM_MAP.md §7 — must land before P7/P13.
- ⏳ `docs/architecture/OVERVIEW.md` — narrative onboarding doc that
  stitches SYSTEM_MAP + BUS + CONFIG + LOGGING + SUPERVISOR together.
  Audience: a new contributor.
- 🕓 `docs/architecture/subsystems/<name>.md` — **deferred.** Write one
  per subsystem alongside its implementation phase (P2..P12), not
  upfront. Writing them now would just go stale before P2 is built.

### Step 3 — Produce `docs/MIGRATION_PLAN.md`
Phased plan from the current single-process threaded code to the target
architecture. Each phase must be independently runnable and reviewable.

**First milestone (from §2.K):** single team, keyboard input, robot
moving in pybullet, Play-state only. Everything else stubbed/disabled
via YAML. The phases below build up to and past that milestone.

Suggested phases (to be refined):

- **P0 — Repo reshape.** Move root `.md` files into `docs/` (rewriting
  the stale ones in the same pass), move/archive
  `incoming_code/bullet_collision_keyboard_explorer.py` and the old
  `tests/test_*.py` exploration scripts into `archive/`, create
  `src/{core,subsystems,apps}/` skeleton. The old
  `jogging_controller` velocity/acceleration knobs already live in
  CONFIG.md as `tuning.robot.max_velocity_deg_s` /
  `max_acceleration_deg_s2` — no extra renaming work, just delete the
  stale fields in `src/`. No behavior change.
- **P1 — Introduce ZMQ bus + YAML config skeleton.** Add `core/bus.py`,
  publish a single `state.full` snapshot from the existing GameController
  loop, keep everything else identical. Add `config/profiles/dev.yaml`
  with an `active_teams: [a]` field even though only team A exists yet.
  Tool `tools/bus_tap.py` prints traffic.
- **P2 — First milestone: keyboard → sim robot → pygame dashboard,
  single team, Play-state only.** Wire up:
  - `apps/keyboard_haptic_sim/` — keyboard producer publishing
    `telem.haptic.a` (replaces missing ESP32 boards).
  - `subsystems/robot/SimRobotIO` — pybullet-backed; consumes
    `cmd.robot.target.a`, publishes `telem.robot.actual.a`. URDF and
    meshes lifted from `incoming_code/ur10e_robot/`.
  - `subsystems/jogging/` — in-process inside GC for now, no collision
    check yet.
  - `apps/gamemaster_ui/` — minimal pygame window showing per-joint
    dial vs actual (the visualizer from
    `incoming_code/bullet_collision_keyboard_explorer.py`), game state,
    and per-process Hz boxes.
  - GameController locked to Play stage; state machine stubbed.
  This is the smoke test the rest of the architecture hangs off of.
- **P3 — Full game state machine.** Idle → Tutorial → Play → Conclusion
  → reset, with the playback-animation Idle, "all scrolled / timeout"
  Tutorial exit, and per-bucket score recap in Conclusion (§2.A).
- **P4 — Split EventRecorder out as its own process.** Per-game folder
  layout + `index.jsonl` (also records final scores per §2.E.34).
- **P5 — Split DisplayBroadcaster out as its own process.** Pure
  SUB → UDP bridge. RPi protocol unchanged.
- **P6 — Split LightColumnController out.** First slow subscriber.
  Validates CONFLATE pattern.
- **P7 — Introduce Launcher with profiles** (no respawn yet). YAML
  profile selects which processes start and which teams are active
  (`team_a_only` / `team_b_only` / `both_teams`). Real vs Sim impls
  picked by profile.
- **P8 — Split HapticIO out.** First two-way I/O subsystem on the bus.
  Keyboard producer from P2 stays as the Sim impl behind the same
  interface (§2.B.16).
- **P9 — Split RobotIO out** using `incoming_code/rtde_core.py`. Real
  UR10e becomes available; SimRobotIO stays for dev.
- **P10 — CollisionWorker pool (16) + JoggingPlanner-as-process per team.**
  REQ → ROUTER/DEALER → REP wired up. Self-collision and world-collision
  toggles in YAML (default on, §2.D.28). Request schema is a **bundle**
  of independent joint configurations per BUS.md §8.
  - **P10-bench — Throughput / latency benchmark (gate before P10 ships).**
    Run a synthetic JP that issues `req.collision_check` bundles of
    size N ∈ {1, 2, 4, 8, 16, 32, 64} against a pool of W ∈ {1, 4, 8, 16}
    workers, with a realistic per-config pybullet cost. Record p50 / p95 /
    p99 latency per bundle, total checks/sec, and worker CPU
    utilization. Compare against the prior single-config measurements
    that motivated bundling, so we can confirm whether the
    ROUTER/DEALER pattern keeps the throughput win or whether we
    should fall back to single-check requests with a deeper DEALER
    buffer. Output goes in `docs/benchmarks/collision_router_dealer.md`
    and the chosen default bundle size is recorded in CONFIG.md under
    `tuning.collision.bundle_size`.
- **P11 — Add the remaining hardware subsystems**, in any order, each
  with its own RS-485 USB adapter:
  - `WeightSensorIO` (+ Sim + manual score adjust UI, §2.E.31/32).
  - `ScoreboardBroadcaster`.
  - `BucketController`.
  - `ButtonController` (drives the physical stop button → soft stop
    into Reset, §2.A.3).
  - `SafetyBarrierController`.
- **P12 — Two-team bring-up.** Enable `both_teams` profile. Verify each
  per-team process pair starts and that shared/global processes serve
  both. Skeleton tracking + prosody from external PCs land in
  EventRecorder.
- **P13 — Add Supervisor heartbeats + respawn.** Per-process policies.
- **P14 — Crash files + retry-aware collision-check client.**

Each phase ends with a working system. The user should be able to stop
at any phase and still have a usable game (or, before P2, a usable
keyboard-driven sim).

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
- The `docs/` folder currently contains only `docs/architecture/`
  with `SYSTEM_MAP.md`, `BUS.md`, and `CONFIG.md`. Everything else
  under `docs/` (overview, logging, supervisor, subsystems/, migration
  plan, archived legacy `.md`) lands during Step 2 and P0.
- Conversation thread that produced this file: planning conversation
  started 2026-06-01, system map written 2026-06-02. If resuming in a
  new session, the user does not need a recap — just read the files
  listed in §4 in order.
