# Haptic Firmware Update Spec for P5

Firmware-facing specification for the haptic controller update needed by
P5 of the migration plan.

Audience: the ESP32 haptic firmware developer.

Status: proposed host contract for the P5 bring-up.

This document supersedes the two-dials-per-board assumptions in
[HAPTIC_PROTOCOL.md](HAPTIC_PROTOCOL.md) for the P5 hardware path. Backward
compatibility with the old dual-dial wire format is not required.

---

## 1. Scope

P5 changes the haptic hardware and host contract in three important ways:

1. One ESP32 board now manages exactly one dial, not two.
2. The dial must support a high-rate tracking target equal to the live
   measured robot joint position outside Tutorial.
3. The dial must support a high-rate min/max soft bound so the firmware can
   generate the out-of-bounds wall and kick feel locally.

The firmware should continue using a simple USB serial ASCII protocol so the
host-side HapticIO process can discover boards, stream commands, and convert
telemetry into the runtime bus topics.

---

## 2. Summary of Required Changes

Compared with protocol version 0.2.0, the P5 firmware must change the
following behavior.

- Replace the old two-motor board model with a single-dial board model.
- Replace paired identity `(motor_id_0, motor_id_1)` with one persistent
  `dial_id` per board.
- Replace paired control frames with a single-dial control frame carrying one
  tracking target and one min/max bound pair.
- Replace paired telemetry frames with a single-dial telemetry frame carrying
  one angle, one speed, one torque, and one loop-rate measurement.
- Replace per-motor parameter names using `_0` and `_1` suffixes with plain
  per-dial parameter names.
- Add a reseat command that changes the controller's current dial position
  estimate digitally without causing a physical yank.

---

## 3. Functional Requirements

### 3.1 Single-Dial Board

Each ESP32 board controls one dial only.

- One USB serial device maps to one `dial_id`.
- One board exposes one encoder angle, one dial velocity, one torque output,
  and one local control loop rate.
- The host will still use six logical dial IDs per team.

Recommended logical IDs for compatibility with the existing host naming:

- Team A: `11, 12, 13, 14, 15, 16`
- Team B: `21, 22, 23, 24, 25, 26`

The host software may keep calling these values `motor_id` in older code, but
for the new firmware they are semantically `dial_id`.

### 3.2 High-Rate Runtime Inputs

The firmware must accept the following high-rate inputs from the host.

- `tracking_target`: the dial position the tracking controller should follow.
  Outside Tutorial, this is the measured robot joint position, not the planned
  robot target.
- `bounds_min` and `bounds_max`: the current reachable envelope for the dial.
  When the dial exceeds these bounds, the local bounds restoration and OOB kick
  logic should act immediately without waiting for additional host decisions.

These values are expected to arrive continuously at approximately 50 Hz.

### 3.3 Reseat / Set Current Position

The firmware must support a command that digitally changes the controller's
current dial position estimate.

This command is required for:

- startup sync, so the dial can be aligned to the robot's actual pose before
  tracking is enabled
- transitions into Tutorial, when the dial may need a fresh local zero or a
  new reference pose
- transitions out of Tutorial, so tracking can resume from the current real
  dial pose without an impulse toward an old target

Critical behavior of reseat:

- It must change the reported dial angle immediately.
- It must not cause a physical jerk toward the old target.
- It must reset any internal velocity estimate used by the tracking loop.
- It must reset OOB kick timing state so an old out-of-bounds pulse does not
  fire immediately after reseat.
- It should set the internal tracking target to the reseated angle unless a
  newer control frame has already arrived.

---

## 4. Expected Host-Side Behavior

The firmware should be designed around the following host behavior.

### 4.1 Startup

1. Host opens the serial port.
2. Board begins streaming telemetry automatically.
3. Host queries version and identity.
4. Host sends infrequent parameter updates if needed.
5. Host sends a reseat command using the robot's measured actual position.
6. Host begins the steady 50 Hz control stream.

### 4.2 Normal Play / Idle / Conclusion

During all game states except Tutorial:

- host streams the measured robot position as the tracking target
- host streams the current min/max dial bounds
- firmware locally computes tracking torque, bounds restoration, and OOB kick

### 4.3 Tutorial

During Tutorial, host behavior may differ.

- Host may disable tracking using an infrequent parameter write.
- Host may disable bounds restoration and OOB kick, or send very wide bounds.
- Host may send a reseat command at Tutorial entry or exit.

The firmware should not assume the same runtime mode is always active.

---

## 5. Proposed Protocol Version

Proposed firmware version: `0.3.0`

Transport remains unchanged unless the firmware developer finds a compelling
implementation reason to change it.

- Transport: USB serial UART
- Baud rate: `230400`
- Framing: ASCII lines terminated by `\n`
- `\r` should be ignored
- Maximum line length target: `127` bytes excluding newline

All numeric fields are ASCII decimal integers.

Units:

- angle on wire: decidegrees
- speed on wire: decidegrees per second
- torque on wire: milliamps
- gains and torque limits: fixed-point `x1000`
- telemetry interval: milliseconds

---

## 6. Command Set

All commands follow this shape:

```text
<CMD>,<seq>[,<fields>...]\n
```

`seq` is a host-selected unsigned 32-bit sequence number.

### 6.1 `C` - Control Update

Primary real-time command sent continuously by the host.

Format:

```text
C,<seq>,<target>,<min>,<max>\n
```

Fields:

- `target`: tracking target angle in decidegrees
- `min`: active lower soft bound in decidegrees
- `max`: active upper soft bound in decidegrees

Behavior:

- The latest valid control frame wins.
- The board applies the most recent target and bounds on every local FOC cycle.
- The board should not wait for an explicit ack round-trip before using the new
  values.
- If `min > max`, discard the frame and keep the previous valid control state.

Response:

- No direct response.
- Telemetry echoes the last processed `C` sequence number.

Rationale:

- The host's high-rate contract only needs tracking target and live bounds.
- OOB kick enable, gains, torque limits, and related tuning remain infrequent
  parameter writes.

### 6.2 `R` - Reseat Current Position

Digitally sets the dial's current position estimate without requiring the user
to physically move the dial and without producing a yank.

Format:

```text
R,<seq>,<current_pos>\n
```

Fields:

- `current_pos`: desired current dial angle in decidegrees

Required behavior:

- Update the internal angle estimate immediately.
- Clear any internally estimated velocity to zero.
- Clear any integrator, derivative memory, or pulse timer state that would
  otherwise create an impulse.
- Make the next telemetry frame report the reseated angle.
- Update the internal tracking target to `current_pos` unless a newer `C`
  command has already been processed.

Response:

```text
R,<seq>\n
```

### 6.3 `S` - Set Runtime Parameter

Used for infrequent tuning changes.

Format:

```text
S,<seq>,<param_name>,<value>\n
```

Response:

```text
S,<seq>\n
```

Expected parameter names for the single-dial firmware:

- `tracking_kp`
- `tracking_kd`
- `tracking_max_torque`
- `bounds_kp`
- `bounds_max_torque`
- `detent_kp`
- `detent_distance`
- `detent_max_torque`
- `vibration_amplitude`
- `vibration_pulse_interval_ms`
- `oob_kick_amplitude`
- `oob_kick_pulse_interval_ms`
- `enable_tracking`
- `enable_bounds_restoration`
- `enable_oob_kick`
- `enable_detent`
- `enable_vibration`
- `telemetry_interval`

Notes:

- Parameter names no longer use `_0` or `_1` suffixes.
- Parameter writes take effect immediately.
- Parameters do not need to persist across reboot, except identity if stored by
  the `I` command.

### 6.4 `I` - Identity Get/Set

Reads or writes the persistent `dial_id` stored in flash.

Query format:

```text
I,<seq>\n
```

Set format:

```text
I,<seq>,<dial_id>\n
```

Response:

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

---

## 7. Telemetry

Telemetry should begin automatically as soon as the serial port is opened.

Format:

```text
T,<dial_id>,<seq>,<ang>,<spd>,<tor>,<foc_rate>,<status_bits>\n
```

Fields:

- `dial_id`: persistent logical dial identity
- `seq`: last processed `C` sequence number, or `0` if none received yet
- `ang`: current dial angle in decidegrees
- `spd`: current dial speed in decidegrees per second
- `tor`: current applied torque in milliamps
- `foc_rate`: measured local control loop rate in Hz
- `status_bits`: compact runtime state flags defined below

Recommended `status_bits` layout:

- bit 0: tracking enabled
- bit 1: bounds restoration enabled
- bit 2: OOB kick enabled
- bit 3: detent enabled
- bit 4: vibration enabled
- bit 5: dial currently outside `[min, max]`
- bit 6: fault active
- bit 7 and above: reserved for future use

If `status_bits` is difficult to implement immediately, it may be shipped as
`0` during first bring-up, but the field should still exist so the line format
does not change later.

---

## 8. Expected Behavior Details

### 8.1 Tracking

- Tracking follows the most recently received `target` from `C`.
- Outside Tutorial, host will send the measured robot position as this target.
- Tracking should remain stable even when the target updates at only 50 Hz,
  while the local FOC loop runs much faster.

### 8.2 Bounds Restoration and OOB Kick

- `min` and `max` from the latest `C` define the active soft bounds.
- When the dial exceeds the active range, the board should apply the bounds
  restoration spring and, if enabled, OOB kick pulses.
- OOB kick generation is local firmware behavior. The host does not need to
  stream one explicit kick command per pulse.
- When the dial is back inside bounds, kick pulses must stop.

### 8.3 Reseat Behavior

Reseat is the most critical new behavior for P5.

After a valid `R` command:

- the next telemetry frame must reflect the new angle
- the dial must not try to travel toward a stale pre-reseat target
- the dial must remain calm if the shaft is physically stationary
- any old OOB pulse timer state must be cleared

### 8.4 Fault Handling

At minimum, the firmware should avoid uncontrolled torque output in these
conditions:

- encoder read failure
- overcurrent or driver fault
- invalid control frame values
- impossible bound values such as `min > max`

In a faulted state:

- torque output should be clamped safe or disabled
- telemetry should continue if possible
- `fault_active` should be visible in `status_bits`

---

## 9. Performance Requirements

These are the expected performance targets for an acceptable P5 firmware.

### 9.1 Required

- Local FOC / torque loop should sustain at least `800 Hz` in steady state.
- Preferred target is around `1000 Hz` or higher if practical.
- Host control stream must accept continuous `50 Hz` `C` commands without
  serial buffer buildup.
- Telemetry must support at least `50 Hz` and should default to `100 Hz`
  (`telemetry_interval = 10 ms`).
- A processed `C` sequence number should appear in telemetry within `100 ms`
  worst case.
- A reseat command should be reflected in telemetry within one telemetry period.

### 9.2 Recommended

- Telemetry jitter should remain low enough that the host can estimate dial
  velocity cleanly from consecutive frames.
- USB reconnect after cable replug should recover without power-cycling the
  board.
- Commands `V`, `I`, `E`, and `S` should still work while telemetry is
  streaming.

---

## 10. Acceptance Criteria

The following behaviors are critical for firmware acceptance.

### 10.1 Discovery and Identity

- On port open, telemetry starts automatically.
- `V,<seq>` returns a valid firmware version string.
- `I,<seq>` returns exactly one persistent `dial_id`.
- `I,<seq>,<dial_id>` updates the stored identity and survives reboot.

### 10.2 Steady Control Loop

- With the host sending `C` at `50 Hz` for at least five minutes, the board
  maintains stable operation without serial overflow or missed telemetry.
- Telemetry reports a healthy `foc_rate` throughout the test.

### 10.3 Reseat Without Yank

Test setup:

- Hold the dial physically still.
- Enable tracking and bounds restoration.
- Send a few `C` frames to establish a nonzero target.
- Then send `R,<seq>,0`.

Acceptance:

- next telemetry frame reports angle near zero
- reported speed returns near zero immediately
- no visible or felt jerk occurs
- torque does not spike as if the dial were still chasing the old target

### 10.4 Bounds Enforcement

Test setup:

- Send `C` with a narrow bound window around zero.
- Manually push the dial outside the allowed range.

Acceptance:

- the dial produces the expected wall feel
- OOB kick pulses only while outside the range
- pulses stop promptly once the dial re-enters bounds

### 10.5 Tutorial Transition

Test setup:

- Enter a mode where tracking is disabled.
- Reseat the dial.
- Re-enable tracking.
- Resume normal `C` updates.

Acceptance:

- no impulse occurs during the transition
- the dial resumes tracking from the current physical pose
- bounds behavior resumes correctly after the transition

### 10.6 Burn-In

- Run for at least `30 minutes` with live telemetry and control frames.
- No serial parser lockup, runaway torque, or unrecoverable fault should occur.

---

## 11. Example Interaction Sequence

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

Interpretation of the telemetry example:

- `dial_id = 21`
- last processed control sequence is `101`
- current angle is `12.8 deg`
- current speed is `3.5 deg/s`
- current torque is `0.118 A`
- loop is running at about `1098 Hz`

---

## 12. Non-Goals for P5

The firmware does not need to solve these before P5 bring-up:

- multi-dial boards
- protocol backward compatibility with version 0.2.0
- persistent storage for every tuning parameter
- a rich binary transport
- on-board understanding of game stages; stage handling stays on the host side

---

## 13. Implementation Notes for the Host Team

These notes are here so the firmware developer can see the host intent.

- The host-side HapticIO process will discover boards by USB VID/PID, then map
  `dial_id -> COM port`.
- The host will publish runtime telemetry onto `telem.haptic.<team>` as six
  arrays: dial position, dial velocity, connection state, and board loop rate.
- The host will eventually consume `cmd.haptic.<team>` from GameController and
  translate it into the serial `C` stream defined here.
- The existing simulator already seeds virtual dial position from the robot's
  measured actual pose. The real firmware should support the same workflow via
  the `R` command.