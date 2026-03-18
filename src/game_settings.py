import threading
from dataclasses import dataclass, field
from typing import Dict, Any


@dataclass
class GameSettings:
    # --- Timing ---
    # Duration of the main gameplay round in seconds (e.g. 120 or 180)
    game_duration_s: int = 180
    # Duration of the tutorial/demo stage in seconds
    tutorial_duration_s: int = 45
    # Duration of the conclusion/score display stage in seconds
    conclusion_duration_s: int = 30
    # Duration to wait during reset/recycle stage in seconds
    reset_duration_s: int = 30
    # When True the system auto-cycles through stages like an arcade
    auto_cycle: bool = True

    # --- Haptic & Control ---
    # Tracking PD stiffness (proportional gain). Unit: unitless (controller-defined scale)
    tracking_kp: float = 5.0
    # Tracking derivative/damping gain. Unit: unitless
    tracking_kd: float = 0.1
    # Maximum torque/current allowed for tracking (amps). Unit: A
    tracking_max_torque: float = 2.0
    # Hard-stop stiffness applied at joint bounds (unitless gain)
    bounds_kp: float = 20.0
    # Enable out-of-bound (OOB) kick pulses at limits
    oob_kick_enabled: bool = True
    # Amplitude of OOB kick (units depend on firmware mapping, typically amps)
    oob_kick_amplitude: float = 1.0
    # Pulse interval for OOB kick in milliseconds
    oob_kick_pulse_interval_ms: int = 40
    # Default gear ratio between dial and joint (dial degrees : joint degrees)
    gear_ratio: float = 10.0
    # Max velocity applied at the dial-side rate limiter (degrees per second)
    dial_max_velocity_dps: float = 5.0
    # Max velocity for the simulated/real robot (degrees per second)
    robot_max_velocity_dps: float = 30.0

    # --- Joint limits ---
    # Per-joint minimum angle (degrees). Keys are motor IDs (e.g. 11..16)
    joint_min_deg: Dict[int, float] = field(
        default_factory=lambda: {mid: -180.0 for mid in range(11, 17)}
    )
    # Per-joint maximum angle (degrees)
    joint_max_deg: Dict[int, float] = field(
        default_factory=lambda: {mid: 180.0 for mid in range(11, 17)}
    )

    # --- Stage/state ---
    # Current high-level stage name (Idle, Tutorial, GameOn, Conclusion, Reset)
    current_stage: str = "Idle"
    # Seconds remaining in the current stage (countdown)
    stage_countdown_s: int = 0
    # Optional manual override command (string); empty when none
    manual_override: str = ""
    # Software emergency stop flag (True => motors/robot should be halted)
    emergency_stop: bool = False

    # --- Frequencies & health metrics ---
    # Observed game loop frequency (Hz)
    game_loop_hz: float = 0.0
    # Observed robot physics / RTDE loop frequency (Hz)
    robot_physics_hz: float = 0.0
    # Observed FOC (firmware) frequencies reported per motor ID (Hz)
    foc_hz: Dict[int, float] = field(default_factory=dict)
    # Observed haptic writer frequencies per board/motor (Hz)
    haptic_writer_hz: Dict[int, float] = field(default_factory=dict)
    # Haptic connection status string (e.g. "4/6")
    haptic_connected_count: str = "0/6"
    # Add more telemetry fields here as needed

    # --- Joint readouts ---
    # Latest dial encoder reading (decidegrees or degrees depending on source)
    dial_position: Dict[int, float] = field(default_factory=dict)
    # Latest commanded joint angle after unit conversion (degrees)
    commanded_deg: Dict[int, float] = field(default_factory=dict)
    # Latest clamped joint angle after applying static limits (degrees)
    clamped_deg: Dict[int, float] = field(default_factory=dict)
    # Latest throttled (rate-limited) joint target (degrees)
    throttled_deg: Dict[int, float] = field(default_factory=dict)
    # Latest robot actual joint positions (degrees)
    robot_actual_deg: Dict[int, float] = field(default_factory=dict)

    # --- Simulation ---
    # When True, use simulated haptic controllers instead of real hardware
    simulate_mode: bool = False
    # Simulated dial angles in joint degrees, keyed by motor ID.
    # Written by the simulator UI panel, read by SimulatedHapticSystem.
    sim_dial_angles: Dict[int, float] = field(
        default_factory=lambda: {mid: 0.0 for mid in range(11, 17)}
    )
    # Simulated bucket weights in grams, keyed by bucket ID (11-13, 21-23).
    # Written by the simulator UI panel, read by SimulatedWeightSensorSystem.
    sim_bucket_weights: Dict[int, float] = field(
        default_factory=lambda: {bid: 0.0 for bid in [11, 12, 13, 21, 22, 23]}
    )

    # --- Weight sensors ---
    # Latest raw weight readings from load cells in grams, keyed by bucket ID.
    # Bucket IDs: 11, 12, 13 (Team 1), 21, 22, 23 (Team 2)
    bucket_weights: Dict[int, float] = field(
        default_factory=lambda: {bid: 0.0 for bid in [11, 12, 13, 21, 22, 23]}
    )
    # Score multiplier per bucket. Harder-to-reach buckets get higher multipliers.
    bucket_multipliers: Dict[int, float] = field(
        default_factory=lambda: {
            11: 1.0,
            12: 2.0,
            13: 3.0,  # Team 1: easy, medium, hard
            21: 1.0,
            22: 2.0,
            23: 3.0,  # Team 2: easy, medium, hard
        }
    )
    # Weight sensor connection status string (e.g. "6/6")
    weight_sensor_connected_count: str = "0/6"
    # Weight sensor read frequency (Hz)
    weight_sensor_hz: float = 0.0

    # --- State publisher ---
    # UDP broadcast address for the state publisher (subnet broadcast)
    broadcast_addr: str = "255.255.255.255"
    # UDP port for the state publisher
    broadcast_port: int = 9000
    # Target publish frequency (Hz)
    publish_hz: float = 50.0
    # Observed publish frequency (Hz) written back by StatePublisher
    publisher_hz: float = 0.0

    # --- Scoring ---
    # Current team scores (weights). Separate fields for clarity.
    # Units: arbitrary score/weight (e.g. total weight in buckets)
    team1_score: float = 0.0
    team2_score: float = 0.0
    # Historical high score across both teams (single value)
    high_score: float = 0.0
    # Optional holder identifier for the high score (e.g., "Team1" or "Team2")
    high_score_holder: str = ""

    # --- Thread safety ---
    _lock: threading.Lock = field(
        default_factory=threading.Lock, repr=False, compare=False
    )

    def get(self, key: str) -> Any:
        with self._lock:
            return getattr(self, key)

    def set(self, key: str, value: Any):
        with self._lock:
            setattr(self, key, value)

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                k: getattr(self, k)
                for k in self.__dataclass_fields__
                if not k.startswith("_")
            }
