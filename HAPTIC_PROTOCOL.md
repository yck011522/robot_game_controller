# Haptic Controller Communication Protocol

**Firmware version:** 0.2.0
**Transport:** USB Serial (UART), 230400 baud, 8N1
**Line format:** ASCII, comma-separated fields, terminated by `\n` (newline). `\r` is ignored.
**Max line length:** 127 bytes (excluding newline).

Each ESP32 board drives **two motors** (motor 0 and motor 1). Motors are identified by persistent IDs stored in NVS flash.

---

## 1. Physical Layer

| Parameter       | Value              |
|-----------------|--------------------|
| Baud rate       | 230400             |
| Data bits       | 8                  |
| Parity          | None               |
| Stop bits       | 1                  |
| Flow control    | None               |
| USB chip        | CH340 |

Known VID/PID pairs for auto-detection:

| Chip      | VID    | PID    |
|-----------|--------|--------|
| CH340     | 0x1A86 | 0x7522 |
| CH340     | 0x1A86 | 0x7523 |


---

## 2. Units & Encoding Conventions

All numeric fields are transmitted as **ASCII decimal integers** (no floating point on the wire).

| Quantity     | Wire unit       | Conversion                              |
|--------------|-----------------|-----------------------------------------|
| Angle        | Decidegrees     | 1 decidegree = 0.1°. `rad = decideg × π / 1800` |
| Torque       | Milliamps (mA)  | 1000 mA = 1.0 A                        |
| PD gains     | ×1000 fixed-point | `float = wire_value / 1000.0`         |
| Detent distance | ×1000 fixed-point (decidegrees) | `float = (wire_value / 1000.0) × π / 1800` |
| FOC rate     | Hz (integer)    | Direct reading                          |
| Telemetry interval | Milliseconds | Direct reading (no ×1000 scaling)    |
| Sequence number | uint32       | Monotonically increasing, wraps at 2³² |

Angle range: ±1,080,000 decidegrees (±30 full rotations).
Torque clamp: ±10,000 mA on telemetry output.

---

## 3. Commands (Host → Controller)

All commands follow the pattern: `<CMD>,<seq>[,<fields>...]\n`

The `seq` field is a uint32 sequence number chosen by the host. The controller echoes it back in the response (where applicable) and includes the last-processed `C` command's seq in every telemetry frame.

### 3.1 `C` — Control (Position Update)

Sends target positions and optional angle bounds for both motors. This is the **primary real-time command** sent every control cycle.

**Format:**
```
C,<seq>,<pos0>,<pos1>[,<min0>,<max0>[,<min1>,<max1>]]\n
```

| Field  | Type   | Required | Description |
|--------|--------|----------|-------------|
| seq    | uint32 | Yes | Sequence number |
| pos0   | long   | Yes | Target position for motor 0 (decidegrees) |
| pos1   | long   | Yes | Target position for motor 1 (decidegrees) |
| min0   | long   | No  | Lower angle bound for motor 0 (decidegrees) |
| max0   | long   | No  | Upper angle bound for motor 0 (decidegrees) |
| min1   | long   | No  | Lower angle bound for motor 1 (decidegrees) |
| max1   | long   | No  | Upper angle bound for motor 1 (decidegrees) |

**Response:** None (acknowledged implicitly via telemetry seq echo).

**Timing:** Send at a steady rate, typically **50 Hz**. The controller applies the latest values on every FOC cycle (~500 Hz). Sending faster than the serial link can handle will cause buffering; sending slower is fine but reduces tracking responsiveness.

**Example:**
```
C,42,1800,-900\n          → Set motor 0 to +180.0°, motor 1 to -90.0°
C,43,1800,-900,-3600,3600,-1800,1800\n  → Same positions, motor 0 bounds ±360°, motor 1 bounds ±180°
```

### 3.2 `S` — Set Parameter

Modifies a single runtime parameter. Changes take effect immediately but are **not persisted** across reboots (except motor IDs, which use the `I` command).

**Format:**
```
S,<seq>,<param_name>,<value>\n
```

| Field      | Type   | Description |
|------------|--------|-------------|
| seq        | uint32 | Sequence number |
| param_name | string | Parameter name (see table below) |
| value      | long   | Integer value (interpretation depends on parameter) |

**Response:**
```
S,<seq>\n
```

**Timing:** On-demand. Send when configuration needs to change.

#### Available Parameters

Most parameter values use **×1000 fixed-point** encoding: send `5000` to set a float value of `5.0`.

| Parameter name          | Unit (wire)       | Description | Default (float) |
|-------------------------|-------------------|-------------|------------------|
| `tracking_kp_0`         | ×1000             | Tracking proportional gain, motor 0 | 5.0 |
| `tracking_kp_1`         | ×1000             | Tracking proportional gain, motor 1 | 5.0 |
| `tracking_kd_0`         | ×1000             | Tracking derivative gain (damping), motor 0 | 0.1 |
| `tracking_kd_1`         | ×1000             | Tracking derivative gain (damping), motor 1 | 0.1 |
| `detent_kp_0`           | ×1000             | Detent spring gain, motor 0 | 5.0 |
| `detent_kp_1`           | ×1000             | Detent spring gain, motor 1 | 5.0 |
| `bounds_kp_0`           | ×1000             | Bounds restoration gain, motor 0 | 20.0 |
| `bounds_kp_1`           | ×1000             | Bounds restoration gain, motor 1 | 20.0 |
| `detent_distance_0`     | ×1000 (decideg)   | Detent spacing, motor 0. Converted: `(val/1000) × π/1800` rad | ~10° |
| `detent_distance_1`     | ×1000 (decideg)   | Detent spacing, motor 1 | ~10° |
| `vibration_amplitude_0` | ×1000             | Vibration pulse amplitude (A), motor 0 | 1.0 |
| `vibration_amplitude_1` | ×1000             | Vibration pulse amplitude (A), motor 1 | 1.0 |
| `oob_kick_amplitude_0`  | ×1000             | Out-of-bounds kick amplitude (A), motor 0 | 1.0 |
| `oob_kick_amplitude_1`  | ×1000             | Out-of-bounds kick amplitude (A), motor 1 | 1.0 |
| `tracking_max_torque_0` | ×1000             | Max tracking torque (A), motor 0 | 2.0 |
| `tracking_max_torque_1` | ×1000             | Max tracking torque (A), motor 1 | 2.0 |
| `bounds_max_torque_0`   | ×1000             | Max bounds restoration torque (A), motor 0 | 3.0 |
| `bounds_max_torque_1`   | ×1000             | Max bounds restoration torque (A), motor 1 | 3.0 |
| `detent_max_torque_0`   | ×1000             | Max detent torque (A), motor 0 | 1.0 |
| `detent_max_torque_1`   | ×1000             | Max detent torque (A), motor 1 | 1.0 |
| `vibration_pulse_interval_ms_0` | Milliseconds (raw, no ×1000) | Vibration pulse interval, motor 0 | 1000 ms |
| `vibration_pulse_interval_ms_1` | Milliseconds (raw, no ×1000) | Vibration pulse interval, motor 1 | 1000 ms |
| `oob_kick_pulse_interval_ms_0`  | Milliseconds (raw, no ×1000) | OOB kick pulse interval, motor 0 | 40 ms |
| `oob_kick_pulse_interval_ms_1`  | Milliseconds (raw, no ×1000) | OOB kick pulse interval, motor 1 | 40 ms |
| `enable_tracking_0`     | 0 or 1            | Enable position tracking, motor 0 | 1 (enabled) |
| `enable_tracking_1`     | 0 or 1            | Enable position tracking, motor 1 | 1 (enabled) |
| `enable_detent_0`       | 0 or 1            | Enable detent mode, motor 0 | 0 (disabled) |
| `enable_detent_1`       | 0 or 1            | Enable detent mode, motor 1 | 0 (disabled) |
| `enable_bounds_restoration_0` | 0 or 1      | Enable bounds restoration, motor 0 | 1 (enabled) |
| `enable_bounds_restoration_1` | 0 or 1      | Enable bounds restoration, motor 1 | 1 (enabled) |
| `enable_oob_kick_0`     | 0 or 1            | Enable OOB kick, motor 0 | 1 (enabled) |
| `enable_oob_kick_1`     | 0 or 1            | Enable OOB kick, motor 1 | 1 (enabled) |
| `enable_vibration_0`    | 0 or 1            | Enable vibration mode, motor 0 | 0 (disabled) |
| `enable_vibration_1`    | 0 or 1            | Enable vibration mode, motor 1 | 0 (disabled) |
| `telemetry_interval`    | Milliseconds (raw, no ×1000) | Telemetry reporting period | 20 ms (50 Hz) |

Unknown parameter names are silently ignored.

**Example:**
```
S,100,tracking_kp_0,8000\n   → Set motor 0 tracking Kp to 8.0
S,101,telemetry_interval,10\n → Set telemetry to 100 Hz (10 ms)
```

### 3.3 `I` — Identity (Get/Set Motor IDs)

Reads or writes the persistent motor identity stored in NVS flash. Motor IDs survive reboots and are included in every telemetry frame, allowing the host to map physical devices to logical motor numbers.

**Query format:**
```
I,<seq>\n
```

**Set format:**
```
I,<seq>,<id0>,<id1>\n
```

| Field | Type   | Description |
|-------|--------|-------------|
| seq   | uint32 | Sequence number |
| id0   | uint8  | Identity for motor 0 (0 = unconfigured, 1–255 = assigned) |
| id1   | uint8  | Identity for motor 1 (0 = unconfigured, 1–255 = assigned) |

**Response (both query and set):**
```
I,<seq>,<motor_id_0>,<motor_id_1>\n
```

**Timing:** On-demand. Typically used once during initial provisioning or device discovery.

**Example:**
```
I,1\n             → Query current IDs. Response: I,1,3,4
I,2,5,6\n        → Set motor 0 to ID 5, motor 1 to ID 6. Response: I,2,5,6
```

### 3.4 `V` — Version Query

Returns the firmware version string.

**Format:**
```
V,<seq>\n
```

**Response:**
```
V,<seq>,<fw_version>\n
```

**Timing:** On-demand. Used during device discovery to confirm the connected device is running the expected firmware.

**Example:**
```
V,1\n    → Response: V,1,0.2.0
```

### 3.5 `E` — Echo (Ping)

Round-trip latency test. The controller echoes the sequence number immediately.

**Format:**
```
E,<seq>\n
```

**Response:**
```
E,<seq>\n
```

**Timing:** On-demand. Useful for measuring serial round-trip time.

---

## 4. Telemetry (Controller → Host)

The controller emits telemetry frames autonomously at a configurable interval (default 20 ms / 50 Hz). Telemetry starts streaming as soon as the serial port is opened — no subscription command is needed.

**Format:**
```
T,<motor_id_0>,<motor_id_1>,<seq>,<ang0>,<ang1>,<spd0>,<spd1>,<tor0>,<tor1>,<foc_rate>\n
```

| Field      | Type   | Description |
|------------|--------|-------------|
| motor_id_0 | uint8  | Persistent identity of motor 0 |
| motor_id_1 | uint8  | Persistent identity of motor 1 |
| seq        | uint32 | Sequence number of the last processed `C` command (0 if none received) |
| ang0       | long   | Current angle of motor 0 (decidegrees) |
| ang1       | long   | Current angle of motor 1 (decidegrees) |
| spd0       | long   | Current speed of motor 0 (decidegrees/s) |
| spd1       | long   | Current speed of motor 1 (decidegrees/s) |
| tor0       | long   | Current applied torque on motor 0 (milliamps) |
| tor1       | long   | Current applied torque on motor 1 (milliamps) |
| foc_rate   | long   | FOC loop rate (Hz), measured over 200 ms windows. Range: 0–2000 |

**Example:**
```
T,3,4,42,1805,-892,500,-300,150,-200,1100
```
Interpretation: Motor IDs 3 and 4, last host seq 42, motor 0 at 180.5° moving at 50.0°/s, motor 1 at −89.2° moving at −30.0°/s, torques 0.15 A and −0.20 A, FOC running at 1100 Hz.

---

## 5. Typical Interaction Sequence

```
Host                                Controller
 │                                      │
 │  (open serial port)                  │
 │                                      │──── T,0,0,0,0,0,0,0,0,0,1100   (auto-streaming)
 │                                      │──── T,0,0,0,5,−3,0,0,0,0,1100
 │                                      │
 │── V,1                               │     (version query)
 │                                      │──── V,1,0.2.0
 │                                      │
 │── I,2                               │     (read motor IDs)
 │                                      │──── I,2,0,0                (unconfigured)
 │                                      │
 │── I,3,1,2                           │     (assign IDs 1 and 2, only if needed)
 │                                      │──── I,3,1,2
 │                                      │
 │── S,10,telemetry_interval,20        │     (confirm 50 Hz telemetry, only if needed)
 │                                      │──── S,10
 │                                      │
 │  ┌─ 50 Hz control loop ────────┐    │
 │  │ C,100,0,0,-3600,3600,-3600,3600  │     (set positions + bounds)
 │  │                              │    │──── T,1,2,100,2,-1,10,-5,50,-30,1100
 │  │ C,101,100,50                 │    │
 │  │                              │    │──── T,1,2,101,98,48,200,100,120,-80,1100
 │  │ ...                          │    │
 │  └──────────────────────────────┘    │
```

---

## 6. Error Handling

- **Buffer overflow:** If a command line exceeds 127 bytes, the buffer is reset and `ERROR: Serial buffer overflow` is printed. The malformed command is discarded.
- **Malformed commands:** Commands with missing required fields are silently ignored (no error response).
- **Unknown command letters:** Silently ignored.
- **Unknown `S` parameter names:** Silently ignored.

---

## 7. Device Discovery & Multi-Controller Setup

When multiple ESP32 controllers are connected via USB:

1. **Enumerate** all serial ports matching known VID/PID pairs (see Section 1).
2. **Probe** each port by sending `V,<seq>\n` and waiting for `V,<seq>,<version>\n` (timeout ~1.5 s). Filter through any telemetry `T,...` lines that arrive first.
3. **Read identity** with `I,<seq>\n` to get the motor IDs assigned to each board.
4. **Build a device map:** `{(motor_id_0, motor_id_1): "COMx", ...}` so the host can address motors by logical ID.
5. **Assign identity** (one-time provisioning): Use the `motor_id_calibration.py` tool or send `I,<seq>,<id0>,<id1>\n` to write persistent IDs. The calibration tool detects which motor is which by monitoring telemetry angle changes while the user physically moves each motor.

---

## 8. Torque Control Modes

The controller computes a composite torque from multiple independently-enabled effects. These are configured via the `S` command parameters and the `C` command bounds. The modes are not mutually exclusive and their torques are summed.

| Mode | Default | Description |
|------|---------|-------------|
| **Tracking** | Enabled | PD controller driving the motor towards `tracking_position` (set by `C` command). Gains: `tracking_kp`, `tracking_kd`. Max torque: `tracking_max_torque`. |
| **Bounds restoration** | Enabled | Strong proportional spring when angle exceeds `bounds_min_angle` / `bounds_max_angle` (set by `C` command). Gain: `bounds_kp`. Max torque: `bounds_max_torque`. |
| **OOB kick** | Enabled | Pulsed corrective force when outside bounds. Amplitude: `oob_kick_amplitude`. Interval: `oob_kick_pulse_interval_ms`. |
| **Detent** | Disabled | Spring-like snap to evenly-spaced detent positions. Spacing: `detent_distance`. Gain: `detent_kp`. Max torque: `detent_max_torque`. |
| **Vibration** | Disabled | Periodic pulse for testing. Amplitude: `vibration_amplitude`. Interval: `vibration_pulse_interval_ms`. |

All modes can be enabled or disabled at runtime via the `S` command using the `enable_<mode>_<motor>` parameters (e.g., `enable_detent_0`). Send `0` to disable, `1` to enable. Modes are not mutually exclusive — their torques are summed.
