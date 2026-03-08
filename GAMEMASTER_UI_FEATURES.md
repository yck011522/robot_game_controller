# Game Master UI — Feature List

## Design Principles

- **Autonomous arcade machine** — the game runs itself through all 5 stages without supervision.
- **Game Master UI is a maintenance/debug tool** — displayed on a hidden physical monitor, accessed on-site or via TeamViewer.
- **Not player-facing** — functional and information-dense, not pretty.
- **Non-blocking** — UI runs on its own thread at ~50 Hz, never interferes with the game loop.

---

## 1. Game State & Autonomous Control

- [ ] Current stage indicator (Idle → Tutorial → Game On → Conclusion → Reset)
- [ ] Autonomous auto-cycle: game loops through all 5 stages like an arcade machine
- [ ] Manual stage override — force-advance, go back, restart, pause the autonomous cycle
- [ ] Software emergency stop — halts all robot motion immediately (supplements physical hardwired e-stop chain)

## 2. Timing Settings

- [ ] Game duration (adjustable, e.g. 2–3 minutes)
- [ ] Tutorial duration (adjustable, e.g. 30–45 seconds)
- [ ] Tutorial readiness threshold (min players completing tutorial to proceed)
- [ ] Conclusion duration (adjustable)
- [ ] Reset duration (adjustable)
- [ ] Live countdown display for current stage

## 3. Haptic & Control Parameters

- [ ] `tracking_kp` — tracking force stiffness
- [ ] `tracking_kd` — tracking force damping
- [ ] `tracking_max_torque` — tracking force cap
- [ ] `bounds_kp` — hard stop stiffness at joint limits
- [ ] OOB kick toggle (enable/disable)
- [ ] OOB kick amplitude
- [ ] OOB kick pulse interval
- [ ] Gear ratio (per-joint or global)
- [ ] Rate limiter `max_velocity_dps` (dial-side, how fast the target moves)
- [ ] Robot `max_velocity_dps` (arm-side, how fast the robot moves)

## 4. Joint Limits & Safety

- [ ] Per-joint min/max angle (editable, takes effect immediately)
- [ ] Collision avoidance toggle (future — when collision avoidance is implemented)

## 5. System Health & Frequency Dashboard

- [ ] Connection status for each ESP32 board (green/red)
- [ ] Connection status for each robot arm (green/red)
- [ ] Game loop Hz
- [ ] Robot physics Hz (simulated) / RTDE Hz (real robot)
- [ ] FOC control Hz per motor (reported from ESP32 telemetry)
- [ ] Haptic serial writer Hz per board
- [ ] Any other HAL control loop frequency
- [ ] Per-joint readout: dial position, commanded, clamped, rate-limited, robot actual
- [ ] Latency measurement (real robot)

## 6. Scoring

- [ ] Live score per team (from weight sensors)
- [ ] Manual score adjustment (sensor fallback)
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
