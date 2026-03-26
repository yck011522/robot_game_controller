# Testing Plan

This document outlines the phased testing plan for the robot game controller. Each phase builds on the previous one and is designed to be testable with the hardware available at the time.

**Current hardware available:**
- 3x ESP32 boards (CH340 USB), each driving 2 FOC motors (6 dials total)
- Motor IDs already provisioned: 11–16 (Team 1). Future Team 2 controllers will be 21–26.
- No robotic arm (simulated in software)

---

## Phase 1 — Serial Communication Foundation

### Goal
Establish reliable simultaneous communication with all 3 ESP32 boards.

### What to Implement

Implemented in `src/haptic_serial.py` with a register-based, self-managing architecture:

- **`HapticSystem`** — Top-level class. Upper app creates it with expected motor IDs (e.g. `[11..16]`), calls `start()`. Background discovery thread auto-finds and connects boards. Upper app reads/writes motor state by motor ID — no COM ports or board knowledge needed.
- **`_BoardConnection`** — Internal, one per ESP32 board. Owns two threads:
  - **Reader thread**: Blocking `readline()`, parses `T` frames, updates telemetry registers
  - **Writer thread**: 50 Hz loop, sends `C` commands from control registers + queued one-off commands
- **`TelemetryFrame`** — Frozen dataclass (motor_id, angle, speed, torque, timestamp). Thread-safe snapshot passed to the upper app.
- **Auto-discovery**: Enumerates COM ports by VID/PID (`1A86:7522` / `1A86:7523`), probes with `V` and `I` commands, matches motor IDs against expected set.
- **Watchdog + reconnect**: If no telemetry received for 0.5 seconds, board is marked disconnected and cleaned up. Next discovery scan reconnects it (possibly on a different COM port).
- **Motor IDs**: Already provisioned as 11–16 (Team 1). Future Team 2 will be 21–26.
- **Control gating**: Writer thread only sends `C` commands after the upper app has called `set_control()` at least once, avoiding unexpected motor movement on connect.

### Tests to Run
- [x] All 3 boards discovered and identified automatically
- [x] Motor IDs 11–16 read correctly from the 3 boards
- [x] Telemetry streams from all 6 motors simultaneously without data loss
- [x] Dial positions read correctly (turn each dial, verify decidegree values make sense)
- [ ] Sequence numbers increment and echo correctly — *not explicitly verified in standalone test (no C commands sent), but infrastructure is in place*
- [ ] FOC rate reported in a reasonable range (~500–600 Hz) — *not displayed in standalone test, but parsed and stored internally*
- [x] System handles USB disconnect/reconnect gracefully

### Test Results

**Date: 2026-03-08**

**Test 1 — Initial run (before DTR fix):**
- All 3 boards were discovered and connected, but took ~8 seconds due to an ESP32 reset caused by pyserial toggling DTR/RTS on serial open. The boards entered their calibration mode before resuming normal operation.
- After the reset delay, all 6 motors (IDs 11–16) were correctly identified across COM7, COM8, COM9.
- Telemetry streamed for all 6 motors. Dial angles displayed correctly in degrees.
- USB unplug/replug test: detected disconnect instantly (serial error) and reconnected on next discovery scan (~3s). Worked very smoothly.
- Board reset button test: the 2.0s watchdog timeout was too close to the ESP32 reboot time (~2s), so the gap was not detected — the board recovered before the watchdog triggered.

**Fix applied — DTR/RTS disabled, watchdog tightened to 0.5s:**
- Serial port now opened with `dtr=False, rts=False` to prevent ESP32 auto-reset.
- Watchdog timeout reduced from 2.0s to 0.5s.

**Test 2 — After DTR fix:**
- All 3 boards connected immediately on first run (no reset). Connection completed in under 1 second.
- Second run also connected immediately — no difference between first and subsequent runs.
- Board reset button: now correctly detected as a disconnect within 0.5s, board reconnects after reboot.
- USB unplug/replug: continues to work instantly as before.
- All 6 motor angles displayed correctly and updated in real-time.

**Conclusion:** All primary Phase 1 goals met. Sequence number echo and FOC rate display not explicitly tested in the standalone script but are implemented and can be verified in Phase 3 when the control loop is active.

---

## Phase 2 — Input Processing Pipeline

### Goal
Transform raw dial positions into commanded joint angles through the safety pipeline.

### What to Implement

Implemented in `src/jogging_controller.py` with three classes:

- **`JointConfig`** — Static per-joint configuration (motor_id, gear_ratio, min/max angle, max velocity). Set once at startup, not modified during gameplay.
- **`JointState`** — Mutable dataclass tracking one joint through the pipeline stages: `raw_dial_decideg` → `dial_deg` → `commanded_deg` → `clamped_deg` → `throttled_deg` → `planned_deg`. Created by the jogging controller, passed downstream to the motion planner.
- **`JoggingController`** — Stateful processor (not threaded). Called each tick by the main game loop. Applies unit conversion, gearing, static range clamping, and rate limiting. Also provides conversion helpers between joint space and dial space.

### Configuration Parameters

| Parameter | Per-joint | Example Value |
|-----------|-----------|---------------|
| Gear ratio | Yes | 10:1 |
| Joint min angle | Yes | -180° |
| Joint max angle | Yes | +180° |
| Max joint velocity | Yes | 30°/s |

### Tests to Run
- [x] Raw dial angle correctly converted through gear ratio for each joint
- [x] Output clamped at joint limits (command past limit, verify output stays at limit)
- [x] Rate limiter works: spin dial quickly, verify output ramps smoothly at configured max velocity
- [x] Rate limiter tracks: when dial stops, output eventually reaches the dial's commanded position
- [x] Collision stub passes through values unchanged
- [x] All 6 joints process independently and correctly

### Test Results

**Date: 2026-03-08**

**Test — Standalone jogging_controller.py:**
- All 6 motors (11–16) connected and processed through the jogging pipeline.
- Gear ratio of 10:1 applied correctly: commanded angles are 1/10th of dial degrees.
- `planned_deg` equals `throttled_deg` as expected (no motion planner yet).
- All 6 joints displayed independent values simultaneously.
- Control loop measured at **48.8 Hz** (dt ~20.3 ms), confirming the display/control decoupling works correctly.
- Motor bounds (±18000 decideg for ±180° joint limits with 10:1 gear) applied on connect via `HapticSystem(motor_bounds=...)`.
- Rate limiter and clamping infrastructure in place but not stress-tested — dials were near zero during test. Full stress testing deferred to Phase 3 when haptic tracking feedback will make the effects directly observable and tunable.

**Conclusion:** Primary Phase 2 goals met. Unit conversion, gearing, and independent joint processing verified. Rate limiting and clamping code is implemented and loop timing is confirmed correct; physical stress testing deferred to Phase 3 when feedback forces make limiting behavior directly testable.

---

## Phase 3 — Haptic Feedback Loop

### Goal
Close the feedback loop from the processed pipeline output back to the haptic dials so players can feel the system state.

### What to Implement

#### Tracking Feedback
- Convert the pipeline's filtered joint position (or simulated robot position) back through the gear ratio into dial-space decidegrees
- Send this as the `pos0`/`pos1` fields of the `C` command to each board
- The ESP32's tracking PD controller creates a restoring force toward this position
- When the user leads ahead of the rate-limited target, they feel resistance proportional to the error

#### Bounds Feedback
- Convert each joint's `[min_angle, max_angle]` back through the gear ratio into dial-space decidegrees
- Send these as the `min0`/`max0`/`min1`/`max1` fields of the `C` command
- The ESP32's bounds restoration creates a hard spring at the limits
- OOB kick provides periodic pulse force when outside bounds

#### The 50 Hz Feedback Loop
```
Every 20ms:
  1. Read telemetry from all 3 boards (6 dial angles)
  2. Apply gearing → 6 raw joint commands
  3. Clamp to joint ranges
  4. Rate-limit → smoothed targets
  5. Collision check (stub)
  6. Update simulated robot position (lag toward filtered target)
  7. Convert robot position back through gear ratio → dial-space
  8. Convert joint limits back through gear ratio → dial-space bounds
  9. Send C command to each board with position + bounds
```

### Haptic Parameter Tuning

| Parameter | Firmware Default | Tuning Notes |
|-----------|-----------------|--------------|
| `tracking_kp` | 5.0 | Spring stiffness for tracking. Higher = stiffer feel. |
| `tracking_kd` | 0.1 | Damping. Higher = more resistance to fast motion. |
| `tracking_max_torque` | 2.0 A | Upper bound on tracking force. |
| `bounds_kp` | 20.0 | Hard stop stiffness. Should be noticeably stronger than tracking. |
| `bounds_max_torque` | 3.0 A | Upper bound on bounds force. |
| `oob_kick_amplitude` | 1.0 A | Kick vibration strength at limits. |
| `oob_kick_pulse_interval_ms` | 40 ms | Kick repetition rate. |

### Tests to Run
- [x] Tracking force felt when dial leads ahead of rate-limited output
- [x] Tracking force disappears when robot "catches up" to dial position
- [x] Hard stop felt at joint limits
- [x] OOB kick vibration felt when pushing past joint limits
- [x] Feedback feels natural and responsive at 50 Hz update rate
- [x] No oscillation or instability in the tracking feedback loop
- [ ] Bounds correctly update if joint limits are changed at runtime — *deferred: no runtime limit changes implemented yet*
- [ ] Tune `tracking_kp`/`tracking_kd` for comfortable feel — record chosen values — *firmware defaults (kp=5.0, kd=0.1) feel acceptable; will tune during Phase 4 end-to-end testing*

### Test Results

**Date: 2026-03-08**

**Test — Standalone jogging_controller.py with haptic feedback loop:**
- Tracking force clearly felt when spinning a dial quickly — rate limiter holds `planned_deg` back, creating a spring force via the ESP32's tracking PD controller.
- Tracking force disappears after the rate-limited position catches up to the dial.
- Hard stop felt at ±180° joint limits (M16 confirmed clamped at +180.0° after 18+ turns).
- OOB kick vibration felt when pushing past joint limits.
- Feedback responsive at 48.6 Hz game loop (dt ~20.6 ms).
- No oscillation or instability observed.
- On exit, dials track back toward zero (position=0 sent in finally block with 100ms flush delay).
- Firmware defaults used: tracking_kp=5.0, tracking_kd=0.1, bounds_kp=20.0.

**Conclusion:** All primary Phase 3 goals met. Haptic feedback loop is closed and working. Runtime bounds update and parameter tuning deferred to Phase 4.

---

## Phase 4 — Simulated Robot Arm

### Goal
A mock robot arm with simple dynamics, so the full end-to-end experience can be tested and tuned without real robot hardware.

### What to Implement

Implemented in `src/simulated_robot.py` with the same threaded register-model pattern as `HapticSystem`:

- **`SimulatedRobotInterface`** — Self-threaded simulated robot arm. Internal physics thread runs at 200 Hz, computing first-order dynamics (speed-limited ramp toward target).
  - `send_target(joint_targets: dict[int, float])` — write target positions (degrees)
  - `get_position(joint_id) → float` — read actual position (degrees)
  - `get_all_positions() → dict[int, float]` — read all positions
  - `start()` / `stop()` — lifecycle management
- **First-order dynamics**: `position += clamp(target - position, -max_velocity * dt, +max_velocity * dt)`
- **Same interface pattern as future `URRobotInterface`** — game loop code won't change when the real robot arrives.
- **Haptic feedback tracks robot actual position** — not the rate-limited planned position. The user feels where the robot *is*, creating a natural two-layer resistance (rate limiter + robot lag).

### Configuration Parameters

| Parameter | Per-joint | Example Value |
|-----------|-----------|---------------|
| Simulated max joint speed | All (single value) | 30°/s |
| Physics update rate | All | 200 Hz |

### Tests to Run
- [x] Simulated arm position lags behind commanded target
- [x] Simulated arm eventually reaches the commanded position
- [x] Full end-to-end loop works: Dial → Pipeline → Sim Arm → Haptic Feedback → Dial
- [x] Turning a dial quickly: feel tracking resistance, then arm catches up
- [x] Pushing past a joint limit: feel hard stop + kick
- [x] All 6 joints behave independently and correctly
- [x] System runs stably over extended periods (minutes)
- [ ] Tune gear ratios, rate limits, and haptic gains for good game feel — record chosen values — *deferred to game design phase*

### Test Results

**Date: 2026-03-08**

**Test — Standalone jogging_controller.py with SimulatedRobotInterface:**
- SimulatedRobotInterface runs on a separate physics thread at 200 Hz (measured 199.6 Hz).
  - Initial implementation hit 64 Hz due to Windows timer resolution (15.6ms granularity). Fixed with hybrid sleep + spin-wait using `time.perf_counter()` and `time.sleep(0)` to yield the GIL.
- Game loop steady at ~49 Hz (50 Hz target).
- Haptic feedback now tracks **robot actual position** instead of planned position, creating natural two-layer resistance.
- Latency simulation tested at 0, 50, 100, and 200 ms:
  - All values: haptic feel remains stable with no oscillation or instability.
  - Observable effect: Robot(°) column in CLI shows delayed response when starting/stopping dial movement. Delay proportional to configured latency.
  - Haptic feel largely unchanged across latency values — the tracking PD controller on the ESP32 handles the lag gracefully.
- Robot position correctly lags behind commanded target (visible in CLI when spinning dial quickly).
- Robot position catches up to commanded target when dial stops (verified via CLI readout converging).
- Hard stops and OOB kicks still felt at joint limits (unchanged from Phase 3).
- All 6 joints behave independently.
- System ran for multiple minutes of testing across many runs without any stability issues.
- Firmware defaults still in use: tracking_kp=5.0, tracking_kd=0.1, bounds_kp=20.0.

**Technical note — Windows high-frequency timer:**
The physics thread uses a hybrid timing strategy. For short cycle times (< 20ms), `Event.wait()` is skipped entirely in favor of a spin-wait loop using `time.perf_counter()`. Each spin iteration calls `time.sleep(0)` to yield the GIL so serial I/O threads can run. For longer cycle times, `Event.wait()` handles the bulk of the delay with spin-wait only for the final portion.

**Conclusion:** All primary Phase 4 goals met. Simulated robot arm integrates cleanly into the end-to-end loop. Latency simulation works correctly but haptic feel is robust across tested range (0–200ms). Parameter tuning deferred to game design phase.

---

## Phase 5 — Monitoring & Tuning UI (Optional)

### Goal
Real-time visibility into system state for debugging and parameter tuning.

### What to Implement
Implemented so far in `src/gamemaster_ui.py`, `src/game_controller.py`, and `src/state_publisher.py`:
- Tkinter Game Master UI with:
  - stage indicator, countdown, auto-cycle toggle, manual stage override, software e-stop
  - real-time joint visualization (robot vs clamped position)
  - timing controls
  - haptic/control parameter controls
  - score display and per-bucket breakdown
  - simulator sliders for haptic dials and bucket weights
  - aggregate system-health display
- UDP state publisher for remote display nodes

Still to validate and/or complete:
- Real-time display of:
  - 6 dial raw positions (decidegrees)
  - 6 geared joint commands (degrees)
  - 6 filtered targets after clamping + rate limiting (degrees)
  - 6 simulated robot positions (degrees)
  - Communication health: seq lag, FOC rate, dropped frames
- Live parameter adjustment: gear ratios, rate limits, joint ranges, haptic gains
- Could be terminal-based (e.g., `curses` / `rich`) or a lightweight web UI

### Tests to Run
- [ ] All values display correctly and update in real-time
- [ ] Parameter changes take effect immediately
- [ ] UI does not degrade communication timing

### Test Results

_No tests run yet._

---

## Additional Outstanding Test Items

Add these after the original phase list so the testing plan matches the current codebase.

### Runtime Parameter Propagation

- [ ] Changing haptic parameters in the Game Master UI updates the live ESP32 controllers immediately
- [ ] Changing `gear_ratio` at runtime updates dial-to-joint conversion and feedback consistently
- [ ] Changing `dial_max_velocity_dps` at runtime updates the active rate limiter without restarting
- [ ] Changing `robot_max_velocity_dps` at runtime updates the active simulated/live robot interface without restarting
- [ ] Changing joint limits at runtime updates both software clamping and haptic dial bounds immediately
- [ ] OOB kick enable/disable and pulse settings are applied live and verified physically

### Health / Telemetry Visibility

- [ ] FOC rate parsed from ESP32 telemetry is surfaced correctly in the UI
- [ ] Sequence-number echo / ack is exposed in diagnostics and verified during closed-loop operation
- [ ] UI health metrics remain accurate during disconnect/reconnect events

### Autonomous Stage Logic

- [ ] Idle exits only on intended dial-motion threshold and does not false-trigger
- [ ] Manual stage override behaves correctly from every stage
- [ ] Software e-stop halts output safely and system resumes cleanly after release
- [ ] Reset stage returns gameplay state to a known baseline

### State Publisher / Display Network

- [ ] UDP state packets match `NETWORK_PROTOCOL.md`
- [ ] One or more receiver nodes can join mid-session and recover without restart
- [ ] Receiver handles stale/no-signal conditions correctly

### Future Hardware Integration

- [ ] Real RS-485 weight sensor protocol implemented and validated on all buckets
- [ ] Real robot interface implemented and validated for startup, motion, shutdown, and fault handling
- [ ] Collision-aware motion planner validated against self-collision and environment constraints

## Notes & Observations

_Space for general notes, observations, and decisions made during testing._
