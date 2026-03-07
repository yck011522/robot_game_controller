# Robot Game Controller

A competitive game where two teams of six players each control a robotic arm through haptic feedback dials. Each player manipulates one joint of the robot using a custom FOC-controlled motor that acts as both an absolute position input and a haptic feedback device.

## Concept

- **Two teams**, each with **six players**
- Each team controls a **6-DOF robotic arm**
- Each player turns a **haptic dial** that commands one joint of the robot
- The dials provide **force feedback** so players can feel:
  - Resistance when they command faster than the robot can move
  - Hard stops and kick vibrations when hitting joint limits or collision zones
  - The robot "catching up" to their commanded position

## Hardware

### Haptic Controllers

Each dial is a brushless motor driven by a Field-Oriented Control (FOC) loop running at ~500–600 Hz on an ESP32 board. Two motors are paired per ESP32 board, for a total of **3 boards per team** (6 joints).

The ESP32 controllers communicate over USB serial at 230400 baud. The host sends position targets and angle bounds at 50 Hz, and receives telemetry (angle, speed, torque, FOC rate) at 50 Hz. See [PROTOCOL.md](PROTOCOL.md) for the full communication protocol.

### Robotic Arm

Each team's robotic arm has 6 joints. The game controller host reads the haptic dials, processes the inputs through a safety pipeline, and sends the filtered commands to the robot.

## Software Architecture

```
┌────────────────────────── Main Game Loop (~50 Hz) ──────────────────────────┐
│                                                                              │
│  1. Read haptic dials    ← HapticSystem     (self-threaded, register model) │
│  2. Read robot position  ← RobotInterface   (self-threaded, register model) │
│  3. Jog processing       ← JoggingController (called each tick, stateful)   │
│  4. Motion planning      ← MotionPlanner     (called each tick, collision)  │
│  5. Send to robot        ← RobotInterface.set_target()                      │
│  6. Haptic feedback      ← HapticSystem.set_control()                       │
│  7. Scoring / display    ← ScoringSystem, DisplaySystem (self-threaded)     │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

**I/O subsystems** (haptic controllers, robot arm, load cells, displays) are each self-threaded with a register model — they manage their own communication timing internally. The main game loop reads and writes to them as shared registers.

**Processing stages** (jogging controller, motion planner) are stateful processors called synchronously by the main game loop each tick. They are not threaded — game logic stays sequential and easy to reason about.

### Haptic Controllers (`HapticSystem`)

See [PROTOCOL.md](PROTOCOL.md) for the ESP32 communication protocol. The `HapticSystem` auto-discovers controllers by USB VID/PID, manages reader/writer threads per board, and provides a motor-ID-based register interface.

### Robot Arm (`RobotInterface`)

Each team's robotic arm is a UR robot communicated with via the **RTDE (Real-Time Data Exchange)** protocol library. RTDE supports up to 500 Hz bidirectional communication. The `RobotInterface` runs its own thread at the RTDE native rate and exposes a register model:
- **Read**: current joint positions (updated at up to 500 Hz internally)
- **Write**: target joint positions (sent at up to 500 Hz internally)

The main game loop reads the current robot position early in each tick and writes the planned target at the end — decoupled from the RTDE update rate.

### Jogging Controller

Processes raw dial inputs into throttled joint targets: unit conversion → gearing → static range clamping → rate limiting. Stateful (tracks rate-limited positions). Does not handle collision detection.

### Motion Planner

Synthesizes all 6 joint targets with collision awareness. Takes the throttled targets from the jogging controller, checks for self-collision and environment collision, and produces the final planned target for each joint. May constrain some joints while allowing others to move freely.

### Haptic Feedback Loop

The planned position (or the robot's actual position) is sent back to the haptic controllers as a tracking target. The ESP32's PD controller creates a restoring force, so players feel:

- **Tracking resistance** — When the player leads ahead of the rate-limited target
- **Bounds restoration + OOB kick** — When the player pushes past a joint limit or into a detected collision zone

## Long-Term Plan

1. **Serial communication layer** — Reliable multi-board communication, device discovery, motor ID provisioning
2. **Input processing pipeline** — Gearing, clamping, rate limiting, collision detection (stubbed)
3. **Haptic feedback loop** — Closed-loop feedback from processed position back to dials
4. **Simulated robot arm** — Mock arm with simple dynamics for testing without hardware
5. **Real robot arm integration** — Connect to actual robot arm, replace simulated interface
6. **Collision detection** — Implement real collision checking against robot self-collision and environment
7. **Game logic** — Scoring, timing, team management, game modes
8. **Monitoring & tuning UI** — Real-time dashboard for diagnostics and parameter adjustment

## Documentation

- [PROTOCOL.md](PROTOCOL.md) — ESP32 haptic controller communication protocol
- [TESTING_PLAN.md](TESTING_PLAN.md) — Phased testing plan and test results log
