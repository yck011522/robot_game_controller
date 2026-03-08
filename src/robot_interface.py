"""Robot arm interfaces.

This module contains robot interface implementations that share a common
pattern: self-threaded with a register model. The game loop writes joint
targets and reads actual positions through thread-safe methods.

Currently implemented:
  - SimulatedRobotInterface — first-order dynamics for testing without hardware

Future:
  - URRobotInterface — real UR robot via RTDE protocol

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

            # Hybrid sleep: coarse sleep + spin-wait for precision.
            # Windows Event.wait() has ~15.6ms resolution, so we only
            # use it when remaining time exceeds that granularity.
            elapsed = time.perf_counter() - cycle_start
            sleep_time = dt - elapsed
            if sleep_time > 0.02:
                self._stop_event.wait(sleep_time - 0.015)
            # Spin-wait for remaining time, yielding GIL each iteration
            # so other Python threads (serial I/O) can run.
            while time.perf_counter() - cycle_start < dt:
                if self._stop_event.is_set():
                    return
                time.sleep(0)


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
