# Game Master UI — Feature List

This file tracks the **UI/control surface status**, not the test status.
Items are marked complete when the feature is present in the current UI/codepath.
If a control exists but is only partially wired to runtime behavior, that is noted inline.

## Design Principles

- **Autonomous arcade machine** — the game runs itself through all 5 stages without supervision.
- **Game Master UI is a maintenance/debug tool** — displayed on a hidden physical monitor, accessed on-site or via TeamViewer.
- **Not player-facing** — functional and information-dense, not pretty.
- **Non-blocking** — UI runs on its own thread at ~50 Hz, never interferes with the game loop.

---

## 1. Game State & Autonomous Control

- [x] Current stage indicator (Idle → Tutorial → Game On → Conclusion → Reset)
- [x] Autonomous auto-cycle: game loops through all 5 stages like an arcade machine
- [x] Manual stage override — force stage changes via the UI
- [x] Software emergency stop — halts the software control loop immediately (still supplements physical hardwired e-stop chain)

## 2. Timing Settings

- [x] Game duration (adjustable, e.g. 2–3 minutes)
- [x] Tutorial duration (adjustable, e.g. 30–45 seconds)
- [ ] Tutorial readiness threshold (min players completing tutorial to proceed)
- [x] Conclusion duration (adjustable)
- [x] Reset duration (adjustable)
- [x] Live countdown display for current stage

## 3. Haptic & Control Parameters

- [x] `tracking_kp` — tracking force stiffness
- [x] `tracking_kd` — tracking force damping
- [x] `tracking_max_torque` — tracking force cap
- [x] `bounds_kp` — hard stop stiffness at joint limits
- [x] OOB kick toggle (enable/disable)
- [x] OOB kick amplitude
- [ ] OOB kick pulse interval
- [x] Gear ratio (global)
- [x] Rate limiter `max_velocity_dps` (dial-side, how fast the target moves)
- [x] Robot `max_velocity_dps` (arm-side, how fast the robot moves)

Notes:
Some of these controls already exist in the UI, but not all of them are fully applied live to the running haptic/robot pipeline yet. Runtime plumbing still needs to be completed and verified.

## 4. Joint Limits & Safety

- [ ] Per-joint min/max angle (editable, takes effect immediately)
- [ ] Collision avoidance toggle (future — when collision avoidance is implemented)

## 5. System Health & Frequency Dashboard

- [x] Connection status for haptic controllers (aggregate count)
- [ ] Connection status for each ESP32 board (green/red)
- [ ] Connection status for each robot arm (green/red)
- [x] Game loop Hz
- [x] Robot physics Hz (simulated) / RTDE Hz (real robot)
- [ ] FOC control Hz per motor (reported from ESP32 telemetry in UI)
- [ ] Haptic serial writer Hz per board
- [ ] Any other HAL control loop frequency
- [ ] Per-joint readout: dial position, commanded, clamped, rate-limited, robot actual
- [ ] Latency measurement (real robot)

Notes:
The UI currently shows robot and clamped joint positions plus several aggregate health metrics. It does not yet expose the full per-joint pipeline breakdown or per-board status.

## 6. Scoring

- [x] Live score per team (from simulated/live weight-sensor interface)
- [ ] Manual score adjustment (sensor fallback)
- [x] High score display
- [ ] High score leaderboard — view, reset, seed

## 7. Game Profiles & Logging

- [ ] Named presets — save/load configurations (e.g. "Easy", "Hard", "Demo", "Maintenance")
  - Each preset bundles: duration, gear ratios, force parameters, rate limits, robot speed
- [ ] Profile scheduling — rotate different profiles automatically (e.g. alternate Easy/Hard every N games)
- [ ] Session logging — record each game: timestamp, profile used, scores, duration
- [ ] Analytics view — historical data: average scores, win rates, per-profile comparison

---

## Explicitly Excluded

- **Team/player management** — museum setting, no way to track individual players. One person may control all six dials.
- **Joint lock/handicap** — disabling a single joint makes the task impossible.

---

## Implementation Notes

- UI technology: Tkinter (lightweight, no external dependencies, native on all platforms)
- UI update rate: ~50 Hz on its own thread
- Communication: reads/writes shared game state via thread-safe registers (same pattern as HapticSystem and SimulatedRobotInterface)
- Current implementation lives in `src/gamemaster_ui.py`
