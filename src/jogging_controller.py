"""Jogging controller for haptic dial inputs.

Processes raw dial positions from the haptic controllers through:
  1. Unit conversion (decidegrees → degrees)
  2. Gearing (dial space → joint space)
  3. Static range clamping (fixed mechanical limits)
  4. Rate limiting (max joint velocity)

The output is a set of JointState objects with throttled joint targets.
The motion planner (Phase 3+) will further refine these with collision
awareness before sending to the robot.

This module is stateful (rate limiter) but not threaded — the main game
loop calls update() each tick.

Usage:
    from jogging_controller import JoggingController, JointConfig

    configs = [
        JointConfig(motor_id=11, gear_ratio=10.0, min_angle_deg=-180, max_angle_deg=180, max_velocity_dps=30),
        # ... one per joint
    ]
    jogger = JoggingController(configs)

    # Called each tick by the main game loop:
    states = jogger.update(dial_angles={11: 18000, 12: -9000, ...}, dt=0.020)
    # states[11].throttled_deg → rate-limited joint angle in degrees
"""

from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------


@dataclass
class JointConfig:
    """Static configuration for one joint. Set once at startup."""

    motor_id: int  # which haptic motor drives this joint (e.g., 11)
    gear_ratio: float  # dial rotations per joint rotation (e.g., 10.0)
    min_angle_deg: float  # fixed mechanical lower limit in robot joint space (degrees)
    max_angle_deg: float  # fixed mechanical upper limit in robot joint space (degrees)
    max_velocity_dps: float  # max robot joint speed (degrees/second in joint space)


@dataclass
class JointState:
    """Mutable state of one joint evolving through the processing pipeline.

    Created by the JoggingController, passed to the motion planner which
    fills in planned_deg. The main game loop uses this to send feedback.
    """

    motor_id: int
    raw_dial_decideg: int = 0  # raw telemetry value from haptic controller
    dial_deg: float = 0.0  # after unit conversion (decideg → deg)
    commanded_deg: float = 0.0  # after gearing (what user is asking for)
    clamped_deg: float = 0.0  # after static range clamping
    throttled_deg: float = 0.0  # after rate limiting
    planned_deg: float = 0.0  # after motion planner (set by downstream)


# ---------------------------------------------------------------------------
# JoggingController
# ---------------------------------------------------------------------------


class JoggingController:
    """Processes raw dial inputs into throttled joint targets.

    Handles: unit conversion, gearing, static range clamping, rate limiting.
    Does NOT handle: collision detection, motion planning, haptic feedback.

    Stateful: tracks rate-limited position per joint across time steps.
    Not threaded: called by the main game loop each tick.
    """

    def __init__(self, configs: list[JointConfig]):
        self._configs: dict[int, JointConfig] = {c.motor_id: c for c in configs}

        # Rate limiter state: {motor_id: current_throttled_deg}
        # Initialized to None — first update sets it to the clamped value.
        self._throttled: dict[int, Optional[float]] = {
            c.motor_id: None for c in configs
        }

    @property
    def motor_ids(self) -> list[int]:
        """List of motor IDs this controller is configured for."""
        return list(self._configs.keys())

    def update(self, dial_angles: dict[int, int], dt: float) -> dict[int, JointState]:
        """Process one time step.

        Args:
            dial_angles: {motor_id: angle_in_decidegrees} from telemetry.
                         Motors not present in this dict are skipped.
            dt: seconds since last update (e.g., 0.020 for 50 Hz).

        Returns:
            {motor_id: JointState} for each joint that had input data.
            planned_deg is set equal to throttled_deg as a default.
        """
        results: dict[int, JointState] = {}

        for motor_id, config in self._configs.items():
            if motor_id not in dial_angles:
                continue

            raw_decideg = dial_angles[motor_id]

            # 1. Unit conversion: decidegrees → degrees
            dial_deg = raw_decideg / 10.0

            # 2. Gearing: dial space → joint space
            commanded_deg = dial_deg / config.gear_ratio

            # 3. Static range clamping
            clamped_deg = max(
                config.min_angle_deg, min(config.max_angle_deg, commanded_deg)
            )

            # 4. Rate limiting
            if self._throttled[motor_id] is None:
                # First update: snap to clamped position (no startup jump)
                throttled_deg = clamped_deg
            else:
                max_step = config.max_velocity_dps * dt
                diff = clamped_deg - self._throttled[motor_id]
                if abs(diff) <= max_step:
                    throttled_deg = clamped_deg
                elif diff > 0:
                    throttled_deg = self._throttled[motor_id] + max_step
                else:
                    throttled_deg = self._throttled[motor_id] - max_step

            self._throttled[motor_id] = throttled_deg

            # Build state object
            state = JointState(
                motor_id=motor_id,
                raw_dial_decideg=raw_decideg,
                dial_deg=dial_deg,
                commanded_deg=commanded_deg,
                clamped_deg=clamped_deg,
                throttled_deg=throttled_deg,
                planned_deg=throttled_deg,  # default; motion planner overwrites
            )
            results[motor_id] = state

        return results

    def reset(self):
        """Clear rate limiter state. Next update will snap to current position."""
        for motor_id in self._throttled:
            self._throttled[motor_id] = None

    # --- Conversion helpers ------------------------------------------------

    def joint_deg_to_dial_decideg(self, motor_id: int, joint_deg: float) -> int:
        """Convert a joint angle (degrees) to dial space (decidegrees)."""
        config = self._configs[motor_id]
        dial_deg = joint_deg * config.gear_ratio
        return int(round(dial_deg * 10.0))

    def dial_decideg_to_joint_deg(self, motor_id: int, decideg: int) -> float:
        """Convert a dial angle (decidegrees) to joint space (degrees)."""
        config = self._configs[motor_id]
        return (decideg / 10.0) / config.gear_ratio

    def joint_limits_to_dial_bounds(self, motor_id: int) -> tuple[int, int]:
        """Get the static joint limits converted to dial space (decidegrees)."""
        config = self._configs[motor_id]
        min_decideg = self.joint_deg_to_dial_decideg(motor_id, config.min_angle_deg)
        max_decideg = self.joint_deg_to_dial_decideg(motor_id, config.max_angle_deg)
        return (min_decideg, max_decideg)


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time
    import sys
    import os

    # Add src directory to path for importing haptic_serial
    sys.path.insert(0, os.path.dirname(__file__))
    from haptic_serial import HapticSystem

    TEAM_1_MOTORS = [11, 12, 13, 14, 15, 16]

    # Configure joints — same defaults for all for now
    configs = [
        JointConfig(
            motor_id=mid,
            gear_ratio=10.0,
            min_angle_deg=-180.0,
            max_angle_deg=180.0,
            max_velocity_dps=5.0,
        )
        for mid in TEAM_1_MOTORS
    ]

    jogger = JoggingController(configs)

    # Compute dial bounds from joint configs — applied automatically on board connect
    motor_bounds = {
        mid: jogger.joint_limits_to_dial_bounds(mid) for mid in TEAM_1_MOTORS
    }
    system = HapticSystem(expected_motor_ids=TEAM_1_MOTORS, motor_bounds=motor_bounds)
    system.start()

    print(f"Looking for motors: {TEAM_1_MOTORS}")
    print("Waiting for all motors to connect...")

    try:
        # --- Wait for connection ---
        while not system.all_connected:
            connected = sorted(system.connected_motor_ids)
            print(
                f"\r  Connected: {connected} ({len(connected)}/{len(TEAM_1_MOTORS)})",
                end="",
                flush=True,
            )
            time.sleep(0.5)

        print(f"\nAll {len(TEAM_1_MOTORS)} motors connected!\n")

        # --- Print header ---
        DISPLAY_INTERVAL_S = 0.050  # Update display at ~5 Hz (printing is slow)

        header = "  Motor | Commanded(°) | Clamped(°) | Throttled(°) | Rate-limited?"
        separator = "  ------+--------------+------------+--------------+--------------"
        print(header)
        print(separator)
        # Pre-print blank lines for cursor-up overwrite
        for _ in TEAM_1_MOTORS:
            print()
        print()  # extra line for status bar

        last_time = time.time()
        last_display_time = 0.0
        loop_count = 0
        latest_states: dict[int, JointState] = {}

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now
            loop_count += 1

            # Read all dial positions
            telemetry = system.get_all_telemetry()
            dial_angles = {}
            for mid, t in telemetry.items():
                if t is not None:
                    dial_angles[mid] = t.angle

            # Process through jogging controller (runs every tick at full speed)
            states = jogger.update(dial_angles, dt)
            latest_states.update(states)

            # Display at a slower rate to avoid console bottleneck
            display_elapsed = now - last_display_time
            if display_elapsed >= DISPLAY_INTERVAL_S:
                hz = loop_count / display_elapsed if display_elapsed > 0 else 0
                lines = []
                for mid in TEAM_1_MOTORS:
                    s = latest_states.get(mid)
                    if s:
                        rate_flag = (
                            "*" if abs(s.commanded_deg - s.throttled_deg) > 0.1 else " "
                        )
                        lines.append(
                            f"  M{mid:>2}   | {s.commanded_deg:>+11.1f} | {s.clamped_deg:>+9.1f} | {s.throttled_deg:>+11.1f} | {rate_flag}"
                        )
                    else:
                        lines.append(
                            f"  M{mid:>2}   |         --- |       --- |         --- |  "
                        )
                lines.append(f"  Loop: {hz:5.1f} Hz  dt: {dt*1000:5.1f} ms")

                # Move cursor up and overwrite
                num_lines = len(TEAM_1_MOTORS) + 1  # data lines + status bar
                output = "\n".join(lines)
                print(f"\033[{num_lines}A{output}", flush=True)

                last_display_time = now
                loop_count = 0

            time.sleep(0.020)  # ~50 Hz control loop

    except KeyboardInterrupt:
        print(f"\n\nShutting down...")
    finally:
        system.stop()
        print("Done.")
