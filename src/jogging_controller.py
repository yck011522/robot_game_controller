"""Jogging controller for haptic dial inputs.

Processes raw dial positions from the haptic controllers through:
  1. Unit conversion (decidegrees → degrees)
  2. Gearing (dial space → joint space)
  3. Static range clamping (fixed mechanical limits)
  4. Rate limiting (max joint velocity, anchored to actual robot position)

The output is a set of JointState objects with throttled joint targets.
The motion planner (Phase 3+) will further refine these with collision
awareness before sending to the robot.

This module is stateless — the rate limiter anchors to the robot's actual
position each tick, so no internal state carries across calls.
Not threaded: the main game loop calls update() each tick.

Usage:
    from jogging_controller import JoggingController, JointConfig

    configs = [
        JointConfig(motor_id=11, gear_ratio=10.0, min_angle_deg=-180, max_angle_deg=180, max_velocity_dps=30),
        # ... one per joint
    ]
    jogger = JoggingController(configs)

    # Called each tick by the main game loop:
    states = jogger.update(dial_angles={11: 18000, ...}, dt=0.020,
                           robot_positions={11: 5.2, ...})
    # states[11].throttled_deg → rate-limited joint angle in degrees
"""

from dataclasses import dataclass, field


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

    Created by the JoggingController, passed to the collision detector
    which fills in planned_deg, safe_min_deg, and safe_max_deg.
    The main game loop uses this to send robot targets and haptic feedback.

    Pipeline:
        1. JoggingController.update() fills raw → dial → commanded → clamped → throttled.
           Sets planned_deg = throttled_deg as default.
        2. CollisionDetector (future) overwrites planned_deg if the throttled
           target is unreachable, and writes safe_min_deg / safe_max_deg —
           the dynamic movable range for this joint given all other joints'
           current positions.
        3. GameController reads planned_deg to send to robot, and uses
           safe_min/max to drive haptic feedback bounds + player display.
    """

    motor_id: int
    raw_dial_decideg: int = 0  # raw telemetry value from haptic controller
    dial_deg: float = 0.0  # after unit conversion (decideg → deg)
    commanded_deg: float = 0.0  # after gearing (what user is asking for)
    clamped_deg: float = 0.0  # after static range clamping
    throttled_deg: float = 0.0  # after rate limiting
    planned_deg: float = 0.0  # after collision detection (sent to robot)
    safe_min_deg: float = -360.0  # dynamic lower limit from collision detector
    safe_max_deg: float = 360.0  # dynamic upper limit from collision detector


# ---------------------------------------------------------------------------
# JoggingController
# ---------------------------------------------------------------------------


class JoggingController:
    """Processes raw dial inputs into throttled joint targets.

    Handles: unit conversion, gearing, static range clamping, rate limiting.
    Does NOT handle: collision detection, motion planning, haptic feedback.

    Stateless: the rate limiter anchors to the robot's actual position
    each tick.  No internal state carries across update() calls.

    Not threaded: called by the main game loop each tick.
    """

    def __init__(self, configs: list[JointConfig]):
        self._configs: dict[int, JointConfig] = {c.motor_id: c for c in configs}

    @property
    def motor_ids(self) -> list[int]:
        """List of motor IDs this controller is configured for."""
        return list(self._configs.keys())

    def update(
        self,
        dial_angles: dict[int, int],
        dt: float,
        robot_positions: dict[int, float],
    ) -> dict[int, JointState]:
        """Process one time step.

        Args:
            dial_angles: {motor_id: angle_in_decidegrees} from telemetry.
                         Motors not present in this dict are skipped.
            dt: seconds since last update (e.g., 0.020 for 50 Hz).
            robot_positions: {motor_id: angle_in_degrees} — actual robot
                joint positions read this tick.  Used as the anchor for
                rate limiting so the target never outruns the real arm.
                Must contain all configured motor IDs.

        Returns:
            {motor_id: JointState} for each joint that had input data.
            planned_deg is set equal to throttled_deg as a default.
        """
        assert robot_positions is not None, "robot_positions is required"
        missing = self._configs.keys() - robot_positions.keys()
        assert not missing, f"robot_positions missing joints: {sorted(missing)}"

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

            # 4. Rate limiting — anchored to robot's actual position
            #    so the target never outruns the real arm.
            anchor = robot_positions[motor_id]
            max_step = config.max_velocity_dps * dt
            diff = clamped_deg - anchor
            if abs(diff) <= max_step:
                throttled_deg = clamped_deg
            elif diff > 0:
                throttled_deg = anchor + max_step
            else:
                throttled_deg = anchor - max_step

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

        # --------------------------------------------------------------
        # TODO: Collision Detection (Phase — future)
        #
        # At this point all joints have their throttled_deg computed.
        # The collision detector runs HERE, after the per-joint loop,
        # because it needs the full set of joint targets simultaneously.
        #
        # Input:
        #   - results: dict[int, JointState] with throttled_deg set
        #   - robot_positions (already a parameter of update())
        #
        # For each joint j, holding all OTHER joints at their current
        # actual robot positions:
        #   1. Compute the collision-free range [safe_min, safe_max]
        #      that joint j can move through without hitting anything.
        #      Write to results[j].safe_min_deg / safe_max_deg.
        #   2. If results[j].throttled_deg is outside [safe_min, safe_max],
        #      clamp it and write the clamped value to results[j].planned_deg.
        #      Otherwise planned_deg stays equal to throttled_deg.
        #
        # The safe_min/max values are used downstream by:
        #   - Haptic feedback: convert to dial-space bounds and send to
        #     the haptic controller so the user feels resistance at the
        #     dynamic collision boundary (replacing the current static
        #     _motor_bounds).
        #   - Player display: LED column or UI shows the available range.
        #   - Session logger: logged at 50 Hz for post-game analysis.
        #
        # The collision model itself is likely a separate class
        # (CollisionDetector) that the JoggingController holds a
        # reference to, or that is injected at construction time.
        # It wraps a URDF/kinematic model and performs per-joint
        # swept-volume or signed-distance checks.
        # --------------------------------------------------------------

        return results

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
    from robot_interface import SimulatedRobotInterface

    # --- Settings ---
    TEAM_1_MOTORS = [11, 12, 13, 14, 15, 16]
    GAME_LOOP_HZ = 50
    DISPLAY_HZ = 20
    ROBOT_MAX_VELOCITY_DPS = (
        30.0  # Simulated robot max velocity in degrees/second (same for all joints)
    )
    ROBOT_LATENCY_MS = 50.0  # simulated round-trip latency (try 50, 100, 200 to test)

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
    robot = SimulatedRobotInterface(
        joint_ids=TEAM_1_MOTORS,
        max_velocity_dps=ROBOT_MAX_VELOCITY_DPS,
        latency_ms=ROBOT_LATENCY_MS,
    )
    system.start()
    robot.start()

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
        DISPLAY_INTERVAL_S = 1.0 / DISPLAY_HZ

        header = "  Motor | Commanded(°) | Clamped(°) | Throttled(°) | Robot(°) | Rate-limited?"
        separator = "  ------+--------------+------------+--------------+----------+--------------"
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

            # Read robot actual positions (before jogging so rate limiter can anchor)
            robot_positions = robot.get_all_positions()

            # Process through jogging controller (runs every tick at full speed)
            states = jogger.update(dial_angles, dt, robot_positions)
            latest_states.update(states)

            # Send rate-limited targets to the simulated robot
            robot_targets = {mid: s.planned_deg for mid, s in states.items()}
            robot.send_target(robot_targets)

            # --- Haptic feedback: track robot ACTUAL position, not planned ---
            for mid in states:
                robot_deg = robot_positions.get(mid, 0.0)
                feedback_pos = jogger.joint_deg_to_dial_decideg(mid, robot_deg)
                min_b, max_b = motor_bounds[mid]
                system.set_control(
                    mid, position=feedback_pos, min_bound=min_b, max_bound=max_b
                )

            # Display at a slower rate to avoid console bottleneck
            display_elapsed = now - last_display_time
            if display_elapsed >= DISPLAY_INTERVAL_S:
                hz = loop_count / display_elapsed if display_elapsed > 0 else 0
                lines = []
                for mid in TEAM_1_MOTORS:
                    s = latest_states.get(mid)
                    r_deg = robot_positions.get(mid, 0.0)
                    if s:
                        rate_flag = (
                            "*" if abs(s.commanded_deg - s.throttled_deg) > 0.1 else " "
                        )
                        lines.append(
                            f"  M{mid:>2}   | {s.commanded_deg:>+11.1f} | {s.clamped_deg:>+9.1f} | {s.throttled_deg:>+11.1f} | {r_deg:>+7.1f} | {rate_flag}"
                        )
                    else:
                        lines.append(
                            f"  M{mid:>2}   |         --- |       --- |         --- |     --- |  "
                        )
                lines.append(
                    f"  Game Loop: {hz:5.1f} Hz | Robot Physics: {robot.actual_hz:5.1f} Hz | Latency: {ROBOT_LATENCY_MS:.0f} ms"
                )

                # Move cursor up and overwrite
                num_lines = len(TEAM_1_MOTORS) + 1  # data lines + status bar
                output = "\n".join(lines)
                print(f"\033[{num_lines}A{output}", flush=True)

                last_display_time = now
                loop_count = 0

            time.sleep(1.0 / GAME_LOOP_HZ)

    except KeyboardInterrupt:
        print(f"\n\nShutting down...")
    finally:
        # Send position=0 to all motors before stopping, so dials track back to zero
        for mid in TEAM_1_MOTORS:
            min_b, max_b = motor_bounds[mid]
            system.set_control(mid, position=0, min_bound=min_b, max_bound=max_b)
        time.sleep(0.1)  # give writer threads time to send the final C commands
        robot.stop()
        system.stop()
        print("Done.")
