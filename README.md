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
┌─────────────────────────────────────────────────────────────┐
│                    Game Controller Host (PC)                 │
│                                                              │
│  ┌──────────┐   ┌──────────────┐   ┌───────────────────┐   │
│  │  Serial   │──▶│   Input      │──▶│  Joint Pipeline   │   │
│  │  Manager  │   │  Processor   │   │                   │   │
│  │ (3 ports) │   │  (gearing,   │   │  1. Gearing Ratio │   │
│  │           │   │   mapping)   │   │  2. Range Clamp   │   │
│  └──────────┘   └──────────────┘   │  3. Rate Limiter   │   │
│       ▲                            │  4. Collision Check │   │
│       │                            └────────┬──────────┘   │
│  ┌──────────┐   ┌──────────────┐            │              │
│  │  Haptic   │◀──│  Feedback    │◀───────────┘              │
│  │  Feedback │   │  Generator   │                           │
│  │  Sender   │   │              │──▶ Robot Arm Interface    │
│  └──────────┘   └──────────────┘     (real or simulated)    │
└─────────────────────────────────────────────────────────────┘
```

### Joint Processing Pipeline

Raw dial positions go through the following stages before being sent to the robot:

1. **Gearing Ratio** — Maps dial rotations to joint rotations (e.g., 10 dial turns = 1 joint rotation)
2. **Range Clamping** — Constrains commanded angles to each joint's allowable range
3. **Rate Limiting** — Caps the speed at which a joint command can change, producing smooth motion
4. **Collision Detection** — Checks that the commanded pose won't cause the robot to collide with itself or its environment

### Haptic Feedback Loop

The filtered position (or the robot's actual position) is sent back to the haptic controllers as a tracking target. The ESP32's PD controller creates a restoring force, so players feel:

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
