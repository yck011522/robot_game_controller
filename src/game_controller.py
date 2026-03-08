"""Central game controller — orchestrates subsystems and runs the game loop.

Owns all subsystems (HapticSystem, JoggingController, SimulatedRobotInterface)
and the 5-stage game state machine. Reads/writes GameSettings as a shared
register so the Game Master UI can observe and control the game.

Runs its own thread (the "game loop" at ~50 Hz). NOT the main thread —
Tkinter needs the main thread.

Usage:
    settings = GameSettings()
    controller = GameController(settings)
    controller.start()
    ...
    controller.stop()
"""

import time
import threading
from typing import Optional

from game_settings import GameSettings
from jogging_controller import JoggingController, JointConfig, JointState
from haptic_serial import HapticSystem
from robot_interface import SimulatedRobotInterface

# Game loop target frequency
_GAME_LOOP_HZ = 50

# Game stages in order
STAGES = ["Idle", "Tutorial", "GameOn", "Conclusion", "Reset"]


class GameController:
    """Central orchestrator — game loop + state machine.

    The game loop runs at ~50 Hz on its own thread:
      1. Read dials via HapticSystem
      2. Process through JoggingController
      3. Send targets to robot
      4. Read robot positions
      5. Send haptic feedback
      6. Update GameSettings with observable state
      7. Advance game stage if needed
    """

    def __init__(self, settings: GameSettings):
        self._settings = settings
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Motor IDs for Team 1
        self._motor_ids = list(range(11, 17))

        # --- Build subsystems (using current settings) ---
        self._jogger: Optional[JoggingController] = None
        self._haptic: Optional[HapticSystem] = None
        self._robot: Optional[SimulatedRobotInterface] = None
        self._motor_bounds: dict[int, tuple[int, int]] = {}

        # Stage timer
        self._stage_start_time: float = 0.0

        # Game loop Hz measurement
        self._loop_count = 0
        self._measure_start = 0.0

    # --- Lifecycle ---------------------------------------------------------

    def start(self):
        """Build subsystems from current settings and start the game loop thread."""
        s = self._settings

        # Build joint configs from settings
        configs = [
            JointConfig(
                motor_id=mid,
                gear_ratio=s.get("gear_ratio"),
                min_angle_deg=s.get("joint_min_deg").get(mid, -180.0),
                max_angle_deg=s.get("joint_max_deg").get(mid, 180.0),
                max_velocity_dps=s.get("dial_max_velocity_dps"),
            )
            for mid in self._motor_ids
        ]
        self._jogger = JoggingController(configs)

        # Compute dial bounds
        self._motor_bounds = {
            mid: self._jogger.joint_limits_to_dial_bounds(mid)
            for mid in self._motor_ids
        }

        # Create subsystems
        self._haptic = HapticSystem(
            expected_motor_ids=self._motor_ids,
            motor_bounds=self._motor_bounds,
        )
        self._robot = SimulatedRobotInterface(
            joint_ids=self._motor_ids,
            max_velocity_dps=s.get("robot_max_velocity_dps"),
            latency_ms=0.0,
        )

        # Start subsystems
        self._haptic.start()
        self._robot.start()

        # Initialize stage
        s.set("current_stage", "Idle")
        self._stage_start_time = time.time()

        # Start game loop thread
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._game_loop, name="game-loop", daemon=True
        )
        self._thread.start()

    def stop(self):
        """Stop the game loop and all subsystems."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)

        # Zero out dials before stopping
        if self._haptic and self._motor_bounds:
            for mid in self._motor_ids:
                min_b, max_b = self._motor_bounds.get(mid, (0, 0))
                self._haptic.set_control(mid, position=0, min_bound=min_b, max_bound=max_b)
            time.sleep(0.1)

        if self._robot:
            self._robot.stop()
        if self._haptic:
            self._haptic.stop()

    # --- Properties --------------------------------------------------------

    @property
    def haptic_system(self) -> Optional[HapticSystem]:
        return self._haptic

    @property
    def robot(self) -> Optional[SimulatedRobotInterface]:
        return self._robot

    @property
    def motor_ids(self) -> list[int]:
        return list(self._motor_ids)

    # --- Game loop ---------------------------------------------------------

    def _game_loop(self):
        dt_target = 1.0 / _GAME_LOOP_HZ
        last_time = time.time()
        self._measure_start = time.time()
        self._loop_count = 0
        latest_states: dict[int, JointState] = {}

        while not self._stop_event.is_set():
            now = time.time()
            dt = now - last_time
            last_time = now
            self._loop_count += 1

            # Check emergency stop
            if self._settings.get("emergency_stop"):
                # Send zero velocity / hold position
                time.sleep(dt_target)
                continue

            # --- 1. Read dials ---
            telemetry = self._haptic.get_all_telemetry()
            dial_angles = {}
            for mid, t in telemetry.items():
                if t is not None:
                    dial_angles[mid] = t.angle

            # --- 2. Process through jogging controller ---
            states = self._jogger.update(dial_angles, dt)
            latest_states.update(states)

            # --- 3. Send targets to robot ---
            robot_targets = {mid: s.planned_deg for mid, s in states.items()}
            self._robot.send_target(robot_targets)

            # --- 4. Read robot positions ---
            robot_positions = self._robot.get_all_positions()

            # --- 5. Send haptic feedback ---
            for mid in states:
                robot_deg = robot_positions.get(mid, 0.0)
                feedback_pos = self._jogger.joint_deg_to_dial_decideg(mid, robot_deg)
                min_b, max_b = self._motor_bounds[mid]
                self._haptic.set_control(
                    mid, position=feedback_pos, min_bound=min_b, max_bound=max_b
                )

            # --- 6. Update settings with observable state ---
            self._update_observable_state(latest_states, robot_positions)

            # --- 7. Advance game stage ---
            self._advance_stage()

            # Sleep
            elapsed = time.time() - now
            sleep_time = dt_target - elapsed
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)

    def _update_observable_state(
        self,
        states: dict[int, JointState],
        robot_positions: dict[int, float],
    ):
        """Push current loop data into GameSettings for the UI to read."""
        s = self._settings

        # Measure game loop Hz
        now = time.time()
        measure_elapsed = now - self._measure_start
        if measure_elapsed >= 0.5:
            hz = self._loop_count / measure_elapsed
            s.set("game_loop_hz", hz)
            self._loop_count = 0
            self._measure_start = now

        # Robot physics Hz
        if self._robot:
            s.set("robot_physics_hz", self._robot.actual_hz)

        # Joint readouts
        dial_pos = {}
        cmd_deg = {}
        clamp_deg = {}
        throttle_deg = {}
        robot_deg = {}

        for mid in self._motor_ids:
            st = states.get(mid)
            if st:
                dial_pos[mid] = st.dial_deg
                cmd_deg[mid] = st.commanded_deg
                clamp_deg[mid] = st.clamped_deg
                throttle_deg[mid] = st.throttled_deg
            robot_deg[mid] = robot_positions.get(mid, 0.0)

        s.update(
            dial_position=dial_pos,
            commanded_deg=cmd_deg,
            clamped_deg=clamp_deg,
            throttled_deg=throttle_deg,
            robot_actual_deg=robot_deg,
        )

        # FOC rates from telemetry (placeholder — needs haptic_serial to expose foc_rate)
        # s.set("foc_hz", {...})

        # Connection status
        if self._haptic:
            connected = self._haptic.connected_motor_ids
            total = len(self._motor_ids)
            s.set("haptic_connected_count", f"{len(connected)}/{total}")

    def _advance_stage(self):
        """Auto-advance through game stages based on timers."""
        s = self._settings

        # Check for manual override
        override = s.get("manual_override")
        if override and override in STAGES:
            s.update(manual_override="", current_stage=override)
            self._stage_start_time = time.time()
            return

        stage = s.get("current_stage")
        elapsed = time.time() - self._stage_start_time

        if not s.get("auto_cycle"):
            # In manual mode, just update countdown
            duration = self._stage_duration(stage)
            if duration > 0:
                remaining = max(0, duration - elapsed)
                s.set("stage_countdown_s", int(remaining))
            return

        duration = self._stage_duration(stage)
        if duration > 0:
            remaining = max(0, duration - elapsed)
            s.set("stage_countdown_s", int(remaining))

            if elapsed >= duration:
                self._next_stage()
        else:
            # Idle has no timer — it waits for dial movement
            s.set("stage_countdown_s", 0)
            self._check_idle_exit()

    def _stage_duration(self, stage: str) -> float:
        """Return the duration in seconds for a given stage. 0 = no timer."""
        s = self._settings
        if stage == "Idle":
            return 0  # waits for player input
        elif stage == "Tutorial":
            return s.get("tutorial_duration_s")
        elif stage == "GameOn":
            return s.get("game_duration_s")
        elif stage == "Conclusion":
            return s.get("conclusion_duration_s")
        elif stage == "Reset":
            return s.get("reset_duration_s")
        return 0

    def _check_idle_exit(self):
        """In Idle stage, check if any dial has moved enough to start."""
        # Check if any commanded_deg is beyond a threshold (e.g. 18 deg = 180 dial deg)
        cmd = self._settings.get("commanded_deg")
        if cmd:
            for mid, deg in cmd.items():
                if abs(deg) > 18.0:  # 180° dial movement / 10 gear ratio
                    self._next_stage()
                    return

    def _next_stage(self):
        """Advance to the next stage in the cycle."""
        s = self._settings
        current = s.get("current_stage")
        try:
            idx = STAGES.index(current)
            next_stage = STAGES[(idx + 1) % len(STAGES)]
        except ValueError:
            next_stage = "Idle"

        s.set("current_stage", next_stage)
        self._stage_start_time = time.time()
