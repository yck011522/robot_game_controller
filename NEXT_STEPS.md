# Next Steps — Architecture Refactor

**Purpose:** Temporary working file to (1) capture feature inventory decisions and
(2) hand off context to a future Copilot session if this conversation ends.

**Status:** P0-P3 are complete. P2 was revalidated on 2026-06-05
(headless regression, local `dev_keyboard` smoke, and benchmark sweep), and P3
was closed the same day after the real-robot team-B bring-up path was validated.
Next active phase: P4 dashboard bring-up.

**Created:** 2026-06-02
**Last updated:** 2026-06-05

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

First we will test in the game Play state, controlling 1 robot with keyboard in single-team mode. The team choice is driven by YAML. P2 closed on the sim profile with team A active; the first real-hardware single-team bring-up must now be team B because the available hardware is wired that way. Either one team or both teams can still be turned on or off for testing and debugging purposes.

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

1. **[README.md](README.md)** — entry point and doc index.
2. **[docs/architecture/OVERVIEW.md](docs/architecture/OVERVIEW.md)** — one-paragraph
   system summary and the four ideas that drive every other architecture doc.
3. **[docs/architecture/SYSTEM_MAP.md](docs/architecture/SYSTEM_MAP.md)** — the
   process catalog, mermaid diagram, edge table, pybullet respawn pattern,
   bus topology, open items. **The authoritative reference for the target
   architecture.**
4. **[docs/MIGRATION_PLAN.md](docs/MIGRATION_PLAN.md)** — phased path from
   legacy single-process code to the target architecture. Read this to find
   the current phase.
5. **NEXT_STEPS.md** (this file) — live decisions and feature inventory.
6. Domain references (kept, now under `docs/`):
   - [docs/GAME_MECHANICS.md](docs/GAME_MECHANICS.md)
   - [docs/NETWORK_PROTOCOL.md](docs/NETWORK_PROTOCOL.md) — UDP payload to
     RPi displays; unchanged by the migration.
   - [docs/HAPTIC_PROTOCOL.md](docs/HAPTIC_PROTOCOL.md) — ESP32 dial wire
     protocol; unchanged by the migration.
   - [docs/LED_COLUMN.md](docs/LED_COLUMN.md)
   - [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)
7. Current source and archived legacy context:
   - [src/main.py](src/main.py) — compatibility stub; use `apps.launcher`.
   - [archive/game_controller.py](archive/game_controller.py) — archived single-process GC + game loop.
   - [src/game_settings.py](src/game_settings.py) — current shared state object.
   - [src/apps/state_broadcaster/__main__.py](src/apps/state_broadcaster/__main__.py) — current UDP display broadcaster (with optional session recording).
   - [src/apps/state_replayer/__main__.py](src/apps/state_replayer/__main__.py) — offline replay of a recorded session over UDP.
   - [src/gamemaster_ui.py](src/gamemaster_ui.py) — current Tk UI to be replaced.
   - [archive/jogging_controller.py](archive/jogging_controller.py) — archived single-process jogging logic.
   - [src/robot_interface.py](src/robot_interface.py)
   - [src/weight_sensor.py](src/weight_sensor.py)
   - [src/led_animation_controller.py](src/led_animation_controller.py)
   - [src/led_animations.py](src/led_animations.py)
   - [src/led_serial.py](src/led_serial.py)
   - [src/led_controller.py](src/led_controller.py)
   - [src/enumerate_usb.py](src/enumerate_usb.py)
   - [src/port_registry.py](src/port_registry.py)
8. Code to integrate (currently outside `src/`):
   - [incoming_code/rtde_core.py](incoming_code/rtde_core.py) — UR10e
     RTDE bridge → becomes the real `RobotIO` impl in P3.
   - [incoming_code/ur10e_robot/](incoming_code/ur10e_robot/) — URDF +
     meshes for the collision worker and SimRobotIO (consumed in P2).
   - [archive/bullet_collision_keyboard_explorer.py](archive/bullet_collision_keyboard_explorer.py)
     — pybullet sandbox; collision logic + viewer extracted in P2.
   - [archive/bullet_collision_keyboard_explorer_design.md](archive/bullet_collision_keyboard_explorer_design.md)
     — design spec for the sandbox above; reference reading for P2.

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
moving in pybullet, Play-state only. That milestone is now complete.

Current rollout snapshot:

- **P0-P3 are complete.** P2 is fully closed: the headless regression
  passes, the collision benchmark sweep has been run and archived under
  `tools/`, and the local `dev_keyboard` smoke was rerun on 2026-06-05. P3 is
  also closed: the team-B `dev_one_robot_keyboard` profile, RTDE robot path,
  passive viewer path, and startup pose sync are all in place.
- **P4 is next.** Dashboard bring-up now starts from the already-working
  team-B real-robot stack.
- **Deferred to P6:** safety-barrier hardware enforcement and physical
  admin buttons no longer block P3/P4; dev profiles keep those subsystems
  `null` until the later hardware-subsystems phase.
- **After P4:** P5 real haptics on team B, P6 remaining hardware on team B,
  P7 full game cycle, then P8 adds team A as the
  second real team.
- The authoritative detailed phase plan lives in
  [docs/MIGRATION_PLAN.md](docs/MIGRATION_PLAN.md). Keep this file as a
  short handoff and decision log rather than a second competing phase
  breakdown.

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
- Local validation on this machine should use
  `C:\Users\yck01\miniconda3\envs\game\python.exe` (or an activated
  `game` env that resolves there).
- Type hints: no strict policy; clarity over rigor.
- The `docs/` folder currently contains only `docs/architecture/`
  with `SYSTEM_MAP.md`, `BUS.md`, and `CONFIG.md`. Everything else
  under `docs/` (overview, logging, supervisor, subsystems/, migration
  plan, archived legacy `.md`) lands during Step 2 and P0.
- Conversation thread that produced this file: planning conversation
  started 2026-06-01, system map written 2026-06-02. If resuming in a
  new session, the user does not need a recap — just read the files
  listed in §4 in order.
