# Haptic Controller Communication Protocol

Consolidated protocol specification for the Phase 5 single-dial firmware and host integration.

This document replaces the overlapping protocol content in `HAPTIC_PROTOCOL.md` and `PROTOCOL.md` and incorporates the latest required behavior from `HAPTIC_FIRMWARE_P5.md`.

## 0. Overview of the device

The controller controlls a BLDC motor with magnetic sensor with FOC using torque control mode.
A composite torque from independently enabled effects. Their torques are summed.

| Mode | Default | Description |
|------|---------|-------------|
| Target Tracking | Enabled | PD controller driving the dial toward the latest `C` target. Uses `tracking_kp`, `tracking_kd`, and `tracking_max_torque`. |
| Bounds restoration | Enabled | Strong proportional spring when angle exceeds the latest `min` or `max`. Uses `bounds_kp` and `bounds_max_torque`. |
| OOB kick | Enabled | Pulsed corrective force while outside bounds. Uses `oob_kick_amplitude` and `oob_kick_pulse_interval_ms`. |
| Detent | Obsolete / No Implementation | Spring-like snap to evenly spaced positions. Uses `detent_distance`, `detent_kp`, and `detent_max_torque`. |
| Vibration | Obsolete / No Implementation | Periodic pulse for testing. Uses `vibration_amplitude` and `vibration_pulse_interval_ms`. |


## 1. Status and Precedence

- Hardware model: one ESP32 board controls one dial
- Transport: USB serial UART
- This document is the normative communication contract between upperlevel game controller and the haptic dial.

## 2. Scope

P5 requires the following single-dial contract:

- One USB serial device maps to one persistent `dial_id`.
- The host continuously streams a live tracking target and active min/max soft bounds.
- The firmware streams telemetry automatically after port open.
- The firmware supports a `Set Current Position` command that rebases the logical dial angle without causing a physical yank.

The firmware does not need protocol backward compatibility with the earlier two-dials-per-board format or anything earlier.

## 3. Physical Layer

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| USB chip | CH340 |

Known VID/PID pairs for auto-detection:

| Chip | VID | PID |
|------|-----|-----|
| CH340 | `0x1A86` | `0x7523` |

## 4. Framing and Encoding

- Line format: ASCII, comma-separated fields, terminated by `\n`
- `\r` should be ignored
- Maximum line length: 127 bytes excluding newline
- All numeric fields are ASCII decimal integers
- Sequence numbers are host-selected `uint32` values and wrap at `2^32`

Wire units:

| Quantity | Wire unit | Conversion |
|----------|-----------|------------|
| Angle | decidegrees | `rad = decideg × pi / 1800` |
| Speed | decidegrees/s | direct |
| Torque | milliamps or millivolt | `1000 mA = 1.0 A` |
| PD gains | x1000 fixed-point | `float = wire_value / 1000.0` |
| Torque limits | x1000 fixed-point | `float = wire_value / 1000.0` |
| Detent distance | x1000 fixed-point decidegrees | `rad = (wire_value / 1000.0) × pi / 1800` |
| FOC rate | Hz | direct |
| Telemetry interval | milliseconds | direct |

Recommended numeric ranges:

- Angle range: `±1,080,000` decidegrees, equivalent to `±30` full rotations
- Telemetry torque output should be clamped to `±10,000` mA

## 5. Identity Model

Each board exposes one persistent `dial_id` stored in flash/NVS.

- `0` means unconfigured
- `1..255` are assigned IDs

Recommended logical IDs for compatibility with the current host naming:

- Team A: `11, 12, 13, 14, 15, 16`
- Team B: `21, 22, 23, 24, 25, 26`

Older host code may still refer to these as `motor_id`, but this protocol treats them as `dial_id`.

## 6. Host to Controller Commands

All commands follow this shape:

```text
<CMD>,<seq>[,<fields>...]\n
```

`seq` is echoed in the command response where applicable and the last processed `C` sequence appears in telemetry.

### 6.1 `C` - Control Update

Primary real-time command sent continuously by the host. Frequency may jitter between 10Hz to 100Hz. 

Format:

```text
C,<seq>,<target>,<min>,<max>\n
```

Fields:

- `target`: tracking target angle in decidegrees
- `min`: active lower soft bound in decidegrees
- `max`: active upper soft bound in decidegrees

Behavior:

- The latest valid control frame wins. No need to process older frames.
- The firmware applies the most recent target and bounds on every local control cycle.
- The firmware should not wait for an explicit ack round-trip before using new values.
- If `min > max`, discard the frame and retain the previous valid control state.
- Normal host operating rate is approximately `50 Hz`.
- The firmware should also tolerate a sustained higher-rate command stream during validation.

Response:

- No direct response.
- Next telemetry echoes the last processed `C` sequence number.

### 6.2 `R` - Set Current Position

Digitally sets the current logical dial position without requiring physical motion and without producing a jerk toward an older target.

Format:

```text
R,<seq>,<current_pos>\n
```

Fields:

- `current_pos`: desired current dial angle in decidegrees

Required behavior:

- Update the internal angle estimate immediately.
- Clear any internally estimated velocity to zero.
- Clear any integrator, derivative memory, or pulse timing state that could create an impulse.
- Clear out-of-bounds kick timing state so an old pulse does not fire after the coordinate change.
- Make the next telemetry frame report the new current angle.
- Update the internal tracking target to `current_pos` unless a newer `C` command has already been processed.

Response:

```text
R,<seq>\n
```

### 6.3 `S` - Set Runtime Parameter

Used for infrequent tuning or mode changes.
Reads or writes the persistent parameter stored in flash/NVS.

Query format:

```text
S,<seq>,<param_name>\n
```

Set Format:

```text
S,<seq>,<param_name>,<value>\n
```

Response for both:

```text
S,<seq>,<param_name>,<value>\n
```

Behavior:

- Parameter writes take effect immediately.
- Parameters need to persist across reboot.
- Unknown parameter names are ignored and the command is still acknowledged, but the value is replaced with '?'.

Supported parameter names:

| Parameter name | Unit on wire | Description | Default / target |
|----------------|--------------|-------------|------------------|
| `tracking_kp` | x1000 | Target Position Tracking proportional gain | `10.0` |
| `tracking_kd` | x1000 | Target Position Tracking derivative gain | `0.1` |
| `tracking_max_torque` | x1000 | Target Position Tracking Max tracking torque in A | `5.0` |
| `bounds_kp` | x1000 | Bounds Restoration Force gain | `20.0` |
| `bounds_max_torque` | x1000 | Max Bounds Restoration torque in A | `1.0` |
| `detent_kp` | x1000 | Digital Detent spring gain | `5.0` |
| `detent_distance` | x1000 decideg | Digital Detent spacing | about `10 deg` |
| `detent_max_torque` | x1000 | Digital Detent Max torque in A | `1.0` |
| `vibration_amplitude` | x1000 | Vibration pulse amplitude in A (OBSOLETE / No Impl)| `1.0` |
| `vibration_pulse_interval_ms` | ms | Vibration pulse interval (OBSOLETE / No Impl)| `1000` ms |
| `oob_kick_amplitude` | x1000 | OUT-OF-BOUND kick amplitude in A | `2.0` |
| `oob_kick_pulse_interval_ms` | ms | OUT-OF-BOUND kick pulse interval | `40` ms |
| `enable_tracking` | `0` or `1` | Enable Target Position Tracking | enabled |
| `enable_bounds_restoration` | `0` or `1` | Enable Bounds Restoration Force | enabled |
| `enable_oob_kick` | `0` or `1` | Enable OOB kick | enabled |
| `enable_detent` | `0` or `1` | Enable detent mode | disabled |
| `enable_vibration` | `0` or `1` | Enable vibration mode | disabled |
| `telemetry_interval` | ms | Telemetry reporting period | P5 target default `10` ms; older docs used `20` ms |

Notes:

- OOB Kicks are implemented as a torque ripple (square waveform) by reducing the typical torque by 'oob_kick_amplitude'.
- The P5 target is a `100 Hz` telemetry default, but the host may still explicitly set this parameter during startup.

### 6.4 `I` - Identity Get/Set

Reads or writes the persistent `dial_id` stored in flash/NVS.

Query format:

```text
I,<seq>\n
```

Set format:

```text
I,<seq>,<dial_id>\n
```

Response for both:

```text
I,<seq>,<dial_id>\n
```

### 6.5 `V` - Version Query

Format:

```text
V,<seq>\n
```

Response:

```text
V,<seq>,<fw_version>\n
```

### 6.6 `E` - Echo / Ping

Format:

```text
E,<seq>\n
```

Response:

```text
E,<seq>\n
```

## 7. Controller to Host Telemetry

Telemetry begins automatically when the serial port is opened. No subscription command is required.

Firmware should auto-rebase the present shaft angle to logical `0` on boot before closed-loop tracking runs.
The host may still send an explicit `R` command during startup to align the dial with the robot's measured pose.

Format:

```text
T,<dial_id>,<seq>,<ang>,<spd>,<tor>,<foc_rate>,<status_bits>\n
```

Fields:

- `dial_id`: persistent logical dial identity
- `seq`: last processed `C` sequence number, or `0` if none received yet
- `ang`: current dial angle in decidegrees
- `spd`: current dial speed in decidegrees per second (filtered, if necessary to remove noise)
- `tor`: current applied torque in milliamps
- `foc_rate`: measured local control loop rate in Hz
- `status_bits`: decimal ASCII bitfield describing runtime state

`status_bits` layout:

- bit 0: tracking enabled
- bit 1: bounds restoration enabled
- bit 2: OOB kick enabled
- bit 3: detent enabled
- bit 4: vibration enabled
- bit 5: dial is currently outside `[min, max]`
- bit 6: fault active
- bit 7 and above: reserved

If `status_bits` cannot be fully implemented during early bring-up, the field should still be present and may temporarily report `0`.

Example:

```text
T,11,42,1805,500,150,1100,35
```

Interpretation:

- Dial ID `11`
- Last processed host control sequence `42`
- Angle `180.5 deg`
- Speed `50.0 deg/s`
- Torque `0.15 A`
- FOC rate `1100 Hz`
- Bits `0`, `1`, and `5` set

## 8. Expected Host Interaction

Typical startup sequence:

1. Host opens the serial port.
2. Board starts streaming telemetry automatically.
3. Host queries version and identity.
4. Host sends infrequent parameter updates if needed.
5. Host sends `R,<seq>,<current_pos>` using the robot's measured actual position.
6. Host begins steady `C` streaming.

Normal operation:

- Host streams the measured robot position as the tracking target.
- Host streams the current min/max dial bounds.
- Firmware computes tracking torque, bounds restoration, and OOB kick locally.

Special modes:

- Host may disable tracking or other modes through `S` writes.
- Host may widen bounds or disable bounds-related effects.
- Host may send `R` before resuming normal operation.

The firmware should not depend on game-stage-specific host knowledge. It only reacts to the latest target, bounds, and runtime settings.

## 9. Error Handling

- Buffer overflow: if a command line exceeds 127 bytes, discard the partial line, latch protocol fault state through telemetry for a short hold window, and do not print ad hoc error text onto the serial wire.
- Malformed commands: discard commands with missing required fields; `status_bits` bit 6 may be set until a later valid command clears the fault state.
- Unknown command letters: ignore them and treat them as a protocol fault.
- Unknown `S` parameter names: ignore them and still acknowledge the `S` command.
- Fault behavior: avoid uncontrolled torque output on encoder failure, driver fault, invalid control values, or impossible bounds such as `min > max`.

In a faulted state:

- torque output should be clamped safe or disabled
- telemetry should continue if possible
- `fault_active` should be visible in `status_bits`

## 10. Device Discovery and Provisioning

Multiple devices will be connected to same host computer, automatic discovery will be performed by host computer
based on the following:

1. Enumerate serial ports matching the known VID/PID pairs.
2. Probe each port with `V,<seq>\n` and wait for `V,<seq>,<version>\n`.
3. Filter through any telemetry lines that arrive before the version response.
4. Query identity with `I,<seq>\n`.
5. Build a device map of `dial_id -> COM port`.
6. If provisioning is required, write `dial_id` with `I,<seq>,<dial_id>\n` or use the repository tooling.

Known repository provisioning tools:

- `tools/deploy_set_id.py`
- `tools/motor_id_calibration.py`

## 11. Performance Targets for P5

Required targets:

- Local FOC or torque loop should sustain at least `800 Hz` in steady state.
- Preferred operating target is around `1000 Hz` or higher if practical.
- Host control stream must support sustained `50 Hz` `C` traffic without serial buffer buildup.
- Firmware should support at least `50 Hz` telemetry and should target a `100 Hz` default.
- A processed `C` sequence should appear in telemetry within `100 ms` worst case.
- A processed `R` command should be reflected in telemetry within one telemetry period.

Recommended targets:

- Telemetry jitter should be low enough for clean host-side velocity estimation.
- USB reconnect after cable replug should recover without a power cycle.
- `V`, `I`, `E`, and `S` should continue to work while telemetry is streaming.

## 13. Example Interaction

```text
Host                                  Controller
 |                                         |
 |  (open serial port)                    |
 |                                         |---- T,21,0,12,0,0,1080,0
 |---- V,1                                 |
 |                                         |---- V,1,0.3.0
 |---- I,2                                 |
 |                                         |---- I,2,21
 |---- S,10,telemetry_interval,10          |
 |                                         |---- S,10
 |---- R,11,0                              |
 |                                         |---- R,11
 |                                         |---- T,21,0,0,0,0,1095,0
 |---- C,100,120,-1800,1800                |
 |                                         |---- T,21,100,115,40,120,1102,3
 |---- C,101,130,-1800,1800                |
 |                                         |---- T,21,101,128,35,118,1098,3
```

## 14. Non-Goals

The P5 communication contract does not require:

- support for multi-dial boards
- backward compatibility with protocol version `0.2.0`
- a binary transport
- firmware awareness of host game stages or modes by name# Haptic Communication Protocol
