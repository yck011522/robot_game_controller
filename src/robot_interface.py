"""Robot arm interfaces.

This module contains robot interface implementations that share a common
pattern: self-threaded with a register model. The game loop writes joint
targets and reads actual positions through thread-safe methods.

Currently implemented:
  - SimulatedRobotInterface — first-order dynamics for testing without hardware
  - URRobotInterface — real UR robot via RTDE protocol (ur_rtde library)

Usage:
    robot = SimulatedRobotInterface(
        joint_ids=[11, 12, 13, 14, 15, 16],
        max_velocity_dps=60.0,
    )
    robot.start()

    robot.send_target({11: 90.0, 12: -45.0})  # degrees
    positions = robot.get_all_positions()       # {11: 12.3, 12: -5.1, ...}

    robot.stop()
"""

import math
import time
import threading
from collections import deque
from typing import Optional


_DEFAULT_PHYSICS_HZ = 200


class SimulatedRobotInterface:
    """Simulated robot arm — self-threaded, register-model interface.

    Each joint moves toward its target at a configurable max velocity,
    modelling first-order dynamics (speed-limited ramp). No inertia,
    gravity, or kinematics — purely joint-space.

    Thread-safe: send_target() and get_position() can be called from
    any thread while the physics loop runs internally.
    """

    def __init__(
        self,
        joint_ids: list[int],
        max_velocity_dps: float = 60.0,
        latency_ms: float = 0.0,
        physics_hz: int = _DEFAULT_PHYSICS_HZ,
    ):
        """
        Args:
            joint_ids: Motor/joint IDs this robot manages (e.g. [11..16]).
            max_velocity_dps: Max joint speed in degrees/second (same for all joints).
            latency_ms: Simulated round-trip latency. Half is applied to commands
                        (send_target delay) and half to feedback (get_position delay).
                        Set to 0 for instantaneous communication.
            physics_hz: Internal physics update rate.
        """
        self._joint_ids = list(joint_ids)
        self._max_velocity_dps = max_velocity_dps
        self._latency_s = latency_ms / 1000.0
        self._physics_hz = physics_hz

        # --- Registers (protected by lock) ---
        self._lock = threading.Lock()
        # Actual positions (written by physics thread, read by game loop)
        self._positions: dict[int, float] = {jid: 0.0 for jid in joint_ids}
        # Target positions used by physics (updated from command buffer)
        self._targets: dict[int, float] = {jid: 0.0 for jid in joint_ids}

        # --- Latency buffers ---
        # Command buffer: targets wait here before being applied
        self._cmd_buffer: deque[tuple[float, dict[int, float]]] = deque()
        # Feedback buffer: position snapshots wait here before being visible
        self._feedback_buffer: deque[tuple[float, dict[int, float]]] = deque()
        # Latest feedback snapshot available to the game loop
        self._feedback_positions: dict[int, float] = {jid: 0.0 for jid in joint_ids}

        self._physics_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Measured physics loop rate
        self._actual_hz: float = 0.0

    @property
    def joint_ids(self) -> list[int]:
        return list(self._joint_ids)

    @property
    def actual_hz(self) -> float:
        """Measured physics loop frequency (Hz)."""
        return self._actual_hz

    # --- Lifecycle ---------------------------------------------------------

    def start(self):
        """Start the internal physics thread."""
        self._stop_event.clear()
        self._physics_thread = threading.Thread(
            target=self._physics_loop,
            name="sim-robot-physics",
            daemon=True,
        )
        self._physics_thread.start()

    def stop(self):
        """Stop the physics thread."""
        self._stop_event.set()
        if self._physics_thread and self._physics_thread.is_alive():
            self._physics_thread.join(timeout=2.0)

    # --- Write interface (target) ------------------------------------------

    def send_target(self, joint_targets: dict[int, float]):
        """Set target positions for joints (degrees).

        With latency > 0, targets are buffered and applied after half
        the round-trip latency has elapsed.
        """
        half_latency = self._latency_s / 2.0
        if half_latency <= 0:
            with self._lock:
                for jid, target in joint_targets.items():
                    if jid in self._targets:
                        self._targets[jid] = target
        else:
            deliver_at = time.time() + half_latency
            with self._lock:
                self._cmd_buffer.append((deliver_at, dict(joint_targets)))

    # --- Read interface (actual position) ----------------------------------

    def get_position(self, joint_id: int) -> Optional[float]:
        """Get the current actual position of a joint (degrees).

        With latency > 0, returns a position delayed by half the
        round-trip latency.
        """
        self._flush_feedback_buffer()
        with self._lock:
            return self._feedback_positions.get(joint_id)

    def get_all_positions(self) -> dict[int, float]:
        """Get current actual positions for all joints (degrees)."""
        self._flush_feedback_buffer()
        with self._lock:
            return dict(self._feedback_positions)

    def _flush_feedback_buffer(self):
        """Promote feedback snapshots whose delay has elapsed."""
        now = time.time()
        with self._lock:
            while self._feedback_buffer and self._feedback_buffer[0][0] <= now:
                _, snapshot = self._feedback_buffer.popleft()
                self._feedback_positions = snapshot

    # --- Physics thread ----------------------------------------------------

    def _physics_loop(self):
        """First-order dynamics: each joint ramps toward its target."""
        dt = 1.0 / self._physics_hz
        half_latency = self._latency_s / 2.0
        loop_count = 0
        measure_start = time.perf_counter()

        while not self._stop_event.is_set():
            cycle_start = time.perf_counter()
            loop_count += 1

            # Measure actual Hz every ~0.5s
            measure_elapsed = cycle_start - measure_start
            if measure_elapsed >= 0.5:
                self._actual_hz = loop_count / measure_elapsed
                loop_count = 0
                measure_start = cycle_start

            with self._lock:
                # Drain command buffer — apply targets whose delay has elapsed
                now = time.time()
                while self._cmd_buffer and self._cmd_buffer[0][0] <= now:
                    _, targets = self._cmd_buffer.popleft()
                    for jid, target in targets.items():
                        if jid in self._targets:
                            self._targets[jid] = target

                # Physics step
                for jid in self._joint_ids:
                    current = self._positions[jid]
                    target = self._targets[jid]
                    diff = target - current
                    max_step = self._max_velocity_dps * dt
                    if abs(diff) <= max_step:
                        self._positions[jid] = target
                    elif diff > 0:
                        self._positions[jid] = current + max_step
                    else:
                        self._positions[jid] = current - max_step

                # Push position snapshot into feedback buffer
                if half_latency > 0:
                    visible_at = time.time() + half_latency
                    self._feedback_buffer.append((visible_at, dict(self._positions)))
                else:
                    self._feedback_positions = dict(self._positions)

            # Hybrid sleep: sleep most of the budget, then spin-wait.
            # time.sleep(0) in the spin loop yields the GIL so other
            # Python threads (game loop, serial I/O) can run.
            elapsed = time.perf_counter() - cycle_start
            remaining = dt - elapsed
            if remaining > 0.0015:
                self._stop_event.wait(remaining - 0.0015)
            while time.perf_counter() - cycle_start < dt:
                if self._stop_event.is_set():
                    return
                time.sleep(0)  # yield GIL to other threads


# ---------------------------------------------------------------------------
# UR Robot (RTDE) Interface
# ---------------------------------------------------------------------------

class URRobotInterface:
    """Real UR robot interface via RTDE — self-threaded, register-model.

    Runs a background thread at the configured frequency (default 500 Hz)
    that sends servoJ commands and reads actual joint positions.  The game
    loop communicates through the same send_target / get_all_positions API
    as SimulatedRobotInterface.

    All public methods work in **degrees** and use motor ID keys (e.g. 11–16)
    to stay consistent with the rest of the game pipeline.  The RTDE layer
    uses radians and 0-indexed joint arrays internally.

    Recovery: if the robot enters a protective stop or the control script
    crashes, the loop will attempt to clear the error via the Dashboard
    Client, reconnect, and resume servo control automatically.

    Thread-safe: send_target() and get_all_positions() can be called from
    any thread while the RTDE loop runs internally.
    """

    # UR joints are indexed 0–5; motor IDs are 11–16.
    _JOINT_INDEX_OFFSET = 11
    _RECONNECT_DELAY_S = 2.0  # seconds to wait before reconnecting after error

    def __init__(
        self,
        joint_ids: list[int],
        robot_ip: str = "192.168.56.101",
        frequency: float = 500.0,
        velocity: float = 0.5,
        acceleration: float = 0.5,
        lookahead_time: float = 0.1,
        gain: float = 300.0,
    ):
        """
        Args:
            joint_ids: Motor/joint IDs this robot manages (e.g. [11..16]).
            robot_ip: IP address of the UR robot or simulator.
            frequency: RTDE control loop frequency in Hz.
            velocity: servoJ velocity parameter (not used by UR firmware).
            acceleration: servoJ acceleration parameter (not used by UR firmware).
            lookahead_time: servoJ smoothing [0.03–0.2s].
            gain: servoJ tracking stiffness [100–2000].
        """
        self._joint_ids = list(joint_ids)
        self._robot_ip = robot_ip
        self._frequency = frequency
        self._dt = 1.0 / frequency

        # servoJ parameters
        self._velocity = velocity
        self._acceleration = acceleration
        self._lookahead_time = lookahead_time
        self._gain = gain

        # --- Registers (protected by lock) ---
        self._lock = threading.Lock()
        self._targets: dict[int, float] = {jid: 0.0 for jid in joint_ids}
        self._positions: dict[int, float] = {jid: 0.0 for jid in joint_ids}
        self._targets_dirty = False  # True after first send_target call

        self._rtde_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()  # set once initial positions are read
        self._actual_hz: float = 0.0
        self._connected = False

    @property
    def joint_ids(self) -> list[int]:
        return list(self._joint_ids)

    @property
    def actual_hz(self) -> float:
        """Measured RTDE loop frequency (Hz)."""
        return self._actual_hz

    @property
    def connected(self) -> bool:
        """Whether the RTDE interfaces are currently connected."""
        return self._connected

    # --- Helpers: motor ID <-> UR joint index ------------------------------

    def _motor_id_to_index(self, motor_id: int) -> int:
        return motor_id - self._JOINT_INDEX_OFFSET

    def _index_to_motor_id(self, index: int) -> int:
        return index + self._JOINT_INDEX_OFFSET

    # --- Lifecycle ---------------------------------------------------------

    def start(self):
        """Connect to the robot and start the RTDE control loop thread.

        Blocks until the initial joint positions have been read from the
        robot (up to 10 seconds), so the game loop never sees zero-
        initialized positions.
        """
        self._stop_event.clear()
        self._ready_event.clear()
        self._rtde_thread = threading.Thread(
            target=self._rtde_loop,
            name="ur-rtde-loop",
            daemon=True,
        )
        self._rtde_thread.start()
        if not self._ready_event.wait(timeout=10.0):
            print("[URRobotInterface] WARNING: timed out waiting for initial position read")

    def stop(self):
        """Stop the RTDE loop and disconnect cleanly."""
        self._stop_event.set()
        if self._rtde_thread and self._rtde_thread.is_alive():
            self._rtde_thread.join(timeout=5.0)

    # --- Write interface (target) ------------------------------------------

    def send_target(self, joint_targets: dict[int, float]):
        """Set target positions for joints (degrees).

        Targets are applied on the next RTDE loop iteration.
        Only joints present in joint_ids are accepted.
        """
        with self._lock:
            for jid, target in joint_targets.items():
                if jid in self._targets:
                    self._targets[jid] = target
            self._targets_dirty = True

    # --- Read interface (actual position) ----------------------------------

    def get_position(self, joint_id: int) -> Optional[float]:
        """Get the current actual position of a joint (degrees)."""
        with self._lock:
            return self._positions.get(joint_id)

    def get_all_positions(self) -> dict[int, float]:
        """Get current actual positions for all joints (degrees)."""
        with self._lock:
            return dict(self._positions)

    # --- RTDE thread -------------------------------------------------------

    def _read_initial_positions(self, rtde_r) -> list[float]:
        """Read current joint positions from robot and populate registers."""
        actual_q = rtde_r.getActualQ()
        with self._lock:
            for i, jid in enumerate(self._joint_ids):
                deg = math.degrees(actual_q[i])
                self._positions[jid] = deg
                # Only set targets if the game loop hasn't sent any yet
                if not self._targets_dirty:
                    self._targets[jid] = deg
        self._ready_event.set()
        return actual_q

    def _try_clear_protective_stop(self):
        """Attempt to clear a protective stop via the Dashboard Client.

        The UR controller requires a 5-second cooldown after a protective
        stop before it can be unlocked.  This method waits, unlocks, and
        re-enables the robot so the RTDE script can be re-uploaded.
        """
        import dashboard_client
        dash = None
        try:
            dash = dashboard_client.DashboardClient(self._robot_ip)
            dash.connect()

            safety = dash.safetystatus()
            print(f"[URRobotInterface] Safety status: {safety}")

            if "PROTECTIVE" in safety.upper():
                print("[URRobotInterface] Waiting 5s for protective stop cooldown ...")
                # Wait in 0.5s increments so we can bail if stop is requested
                for _ in range(10):
                    if self._stop_event.is_set():
                        return
                    time.sleep(0.5)
                dash.closeSafetyPopup()
                dash.unlockProtectiveStop()
                print("[URRobotInterface] Protective stop unlocked")
                # Give the robot time to transition back to RUNNING mode
                time.sleep(1.0)

            # Ensure robot is powered on and brake is released
            mode = dash.robotmode()
            print(f"[URRobotInterface] Robot mode: {mode}")
            if "RUNNING" not in mode.upper():
                print("[URRobotInterface] Attempting power on + brake release ...")
                dash.powerOn()
                time.sleep(2.0)
                dash.brakeRelease()
                time.sleep(2.0)

        except Exception as e:
            print(f"[URRobotInterface] Dashboard recovery failed: {e}")
        finally:
            if dash:
                try:
                    dash.disconnect()
                except Exception:
                    pass

    def _rtde_loop(self):
        """Main RTDE loop with automatic reconnection on error.

        Outer loop handles connection/reconnection.  Inner loop runs the
        500 Hz servoJ cycle.  On error (protective stop, script crash,
        network loss) the outer loop attempts recovery and reconnects.
        """
        import rtde_control
        import rtde_receive

        while not self._stop_event.is_set():
            rtde_c = None
            rtde_r = None

            try:
                # Connect
                print(f"[URRobotInterface] Connecting to {self._robot_ip} ...")
                flags = (
                    rtde_control.RTDEControlInterface.FLAG_VERBOSE
                    | rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT
                )
                rtde_c = rtde_control.RTDEControlInterface(
                    self._robot_ip, self._frequency, flags
                )
                rtde_r = rtde_receive.RTDEReceiveInterface(self._robot_ip)
                self._connected = True
                print("[URRobotInterface] Connected")

                # Read initial position
                actual_q = self._read_initial_positions(rtde_r)

                # Servo loop
                loop_count = 0
                measure_start = time.perf_counter()
                servo_q = list(actual_q)  # working copy in radians

                while not self._stop_event.is_set():
                    t_start = rtde_c.initPeriod()
                    loop_count += 1

                    # Measure Hz
                    now = time.perf_counter()
                    measure_elapsed = now - measure_start
                    if measure_elapsed >= 0.5:
                        self._actual_hz = loop_count / measure_elapsed
                        loop_count = 0
                        measure_start = now

                    # Read actual joint positions from robot
                    actual_q = rtde_r.getActualQ()
                    with self._lock:
                        for i, jid in enumerate(self._joint_ids):
                            self._positions[jid] = math.degrees(actual_q[i])

                        # Build servo target from latest targets (degrees -> radians)
                        if self._targets_dirty:
                            for i, jid in enumerate(self._joint_ids):
                                servo_q[i] = math.radians(self._targets[jid])

                    # Send servoJ command
                    rtde_c.servoJ(
                        servo_q,
                        self._velocity,
                        self._acceleration,
                        self._dt,
                        self._lookahead_time,
                        self._gain,
                    )

                    rtde_c.waitPeriod(t_start)

            except Exception as e:
                print(f"[URRobotInterface] RTDE error: {e}")
                self._connected = False
                self._actual_hz = 0.0

            finally:
                # Clean shutdown of this connection attempt
                self._connected = False
                if rtde_c:
                    try:
                        rtde_c.servoStop()
                        rtde_c.stopScript()
                    except Exception:
                        pass
                if rtde_r:
                    try:
                        rtde_r.disconnect()
                    except Exception:
                        pass

            # If we get here and stop wasn't requested, attempt recovery
            if not self._stop_event.is_set():
                print(f"[URRobotInterface] Will attempt recovery in {self._RECONNECT_DELAY_S}s ...")
                self._try_clear_protective_stop()
                # Wait before reconnecting (interruptible)
                self._stop_event.wait(self._RECONNECT_DELAY_S)


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import math

    joint_ids = [11, 12, 13, 14, 15, 16]
    robot = SimulatedRobotInterface(
        joint_ids=joint_ids,
        max_velocity_dps=30.0,
    )
    robot.start()

    print("Simulated robot — ramping joint 11 to +90°, others stay at 0°")
    robot.send_target({11: 90.0})

    try:
        for _ in range(100):
            positions = robot.get_all_positions()
            vals = "  ".join(f"J{jid}: {positions[jid]:>+7.1f}" for jid in joint_ids)
            print(f"\r  {vals}", end="", flush=True)
            time.sleep(0.050)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        print("\nDone.")
