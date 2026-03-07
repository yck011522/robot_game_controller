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
- **Device discovery**: Enumerate COM ports by VID/PID (`1A86:7522` / `1A86:7523`), probe each with `V` command, read motor IDs with `I` command
- **Device map**: Build mapping of `motor_id → COM port` for all 6 motors
- **Motor ID provisioning**: Motor IDs are already assigned (11–16 for Team 1, 21–26 for future Team 2). Discovery reads these IDs to build the device map.
- **Telemetry reader**: Open all 3 serial ports, parse `T` frames at 50 Hz, aggregate into a unified data structure (6 motor angles, speeds, torques, FOC rates)
- **Command sender**: Send `C` commands at 50 Hz to each board
- **Sequence tracking**: Maintain per-board sequence counters, detect command lag via telemetry seq echo

### Tests to Run
- [ ] All 3 boards discovered and identified automatically
- [ ] Motor IDs 11–16 read correctly from the 3 boards
- [ ] Telemetry streams from all 6 motors simultaneously without data loss
- [ ] Dial positions read correctly (turn each dial, verify decidegree values make sense)
- [ ] Sequence numbers increment and echo correctly
- [ ] FOC rate reported in a reasonable range (~500–600 Hz)
- [ ] System handles USB disconnect/reconnect gracefully

### Test Results

_No tests run yet._

---

## Phase 2 — Input Processing Pipeline

### Goal
Transform raw dial positions into commanded joint angles through the safety pipeline.

### What to Implement
- **Gearing ratio**: Configurable per joint (e.g., 10:1 means 10 full dial rotations = 1 full joint rotation). Raw dial decidegrees divided by gear ratio to produce joint angle.
- **Joint range clamping**: Configurable `[min_angle, max_angle]` per joint. Output clamped to range.
- **Rate limiting**: Configurable max velocity per joint (deg/s). Output angle changes are capped per time step so the target moves smoothly toward the user's command without exceeding the speed limit.
- **Collision detection stub**: Pass-through function with the correct interface (`6 joint angles in → 6 joint angles out`) to be replaced later with real collision logic.

### Configuration Parameters

| Parameter | Per-joint | Example Value |
|-----------|-----------|---------------|
| Gear ratio | Yes | 10:1 |
| Joint min angle | Yes | -180° |
| Joint max angle | Yes | +180° |
| Max joint velocity | Yes | 30°/s |

### Tests to Run
- [ ] Raw dial angle correctly converted through gear ratio for each joint
- [ ] Output clamped at joint limits (command past limit, verify output stays at limit)
- [ ] Rate limiter works: spin dial quickly, verify output ramps smoothly at configured max velocity
- [ ] Rate limiter tracks: when dial stops, output eventually reaches the dial's commanded position
- [ ] Collision stub passes through values unchanged
- [ ] All 6 joints process independently and correctly

### Test Results

_No tests run yet._

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
- [ ] Tracking force felt when dial leads ahead of rate-limited output
- [ ] Tracking force disappears when robot "catches up" to dial position
- [ ] Hard stop felt at joint limits
- [ ] OOB kick vibration felt when pushing past joint limits
- [ ] Feedback feels natural and responsive at 50 Hz update rate
- [ ] No oscillation or instability in the tracking feedback loop
- [ ] Bounds correctly update if joint limits are changed at runtime
- [ ] Tune `tracking_kp`/`tracking_kd` for comfortable feel — record chosen values

### Test Results

_No tests run yet._

---

## Phase 4 — Simulated Robot Arm

### Goal
A mock robot arm with simple dynamics, so the full end-to-end experience can be tested and tuned without real robot hardware.

### What to Implement
- **First-order dynamics**: Each joint moves toward its filtered target at a configurable max speed, simulating real robot lag. E.g., a simple model: `position += clamp(target - position, -max_speed * dt, +max_speed * dt)`
- **Joint state**: Maintain the "actual" position of all 6 joints
- **Exchangeable interface**: The simulated arm exposes the same interface as a future real robot driver:
  - `send_target(joint_angles: [6])` — command the arm
  - `get_current_position() → [6]` — read current joint positions

### Configuration Parameters

| Parameter | Per-joint | Example Value |
|-----------|-----------|---------------|
| Simulated max joint speed | Yes | 30°/s |
| Response time constant | Yes | ~200 ms |

### Tests to Run
- [ ] Simulated arm position lags behind commanded target
- [ ] Simulated arm eventually reaches the commanded position
- [ ] Full end-to-end loop works: Dial → Pipeline → Sim Arm → Haptic Feedback → Dial
- [ ] Turning a dial quickly: feel tracking resistance, then arm catches up
- [ ] Pushing past a joint limit: feel hard stop + kick
- [ ] All 6 joints behave independently and correctly
- [ ] System runs stably over extended periods (minutes)
- [ ] Tune gear ratios, rate limits, and haptic gains for good game feel — record chosen values

### Test Results

_No tests run yet._

---

## Phase 5 — Monitoring & Tuning UI (Optional)

### Goal
Real-time visibility into system state for debugging and parameter tuning.

### What to Implement
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

## Notes & Observations

_Space for general notes, observations, and decisions made during testing._
