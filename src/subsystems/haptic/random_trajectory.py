"""Seeded random trajectory haptic source for real-robot validation.

This implementation pretends to be the Team B haptic input during a
long-running collision-avoidance validation. It moves a virtual
robot-space target at a fixed random six-axis velocity, converts that
target into dial-space positions, and lets the normal game controller
and jogging planner apply speed, acceleration, joint-limit, and
collision rules.
"""

from __future__ import annotations

import math
import random
import time
from typing import Any, Callable


PUBLISH_HZ = 50.0
DEFAULT_SEED = 12345
DEFAULT_SPEED_SCALE = 1.0
DEFAULT_MIN_AXIS_SPEED_FRACTION = 0.2
DEFAULT_PATH_TURNAROUND_DISTANCE_DEG = 3.0
DEFAULT_LIMIT_MARGIN_DEG = 1.0
DEFAULT_PROXIMITY_FLIP_DISTANCE_DEG = 3.0
DEFAULT_PROXIMITY_STALE_TICKS = 12
DEFAULT_WINDOW_SIZE = (430, 180)


class RandomTrajectoryHaptic:
    """Generate deterministic random robot target motion for validation.

    The haptic app calls `update_robot_actual` when robot telemetry arrives,
    `update_state_full` when controller state arrives, and `sample` once per
    haptic tick. The generated `dial_pos_rad` is an absolute target encoded
    in dial space, so the profile should use `tuning.haptic.input_mode:
    absolute`.
    """

    def __init__(
        self,
        *,
        team: str,
        profile,
        now_fn: Callable[[], float] | None = None,
    ) -> None:
        validation_cfg = _mapping(profile.tuning.get("random_trajectory_validation"))
        haptic_cfg = _mapping(profile.tuning.get("haptic"))
        robot_cfg = _mapping(profile.tuning.get("robot"))
        jogging_cfg = _mapping(profile.tuning.get("jogging"))

        self._team = team  # Team key used to read this team's state.full payload.
        self._now = now_fn or time.perf_counter  # Monotonic clock used for integration.
        self._rng = random.Random(_int_value(validation_cfg.get("seed"), DEFAULT_SEED))
        self._running = bool(validation_cfg.get("enabled_on_start", False))
        self._ui_enabled = bool(validation_cfg.get("ui_enabled", True))
        self._speed_scale = _nonnegative_float(
            validation_cfg.get("speed_scale"), DEFAULT_SPEED_SCALE
        )
        self._min_axis_speed_fraction = _clamped_float(
            validation_cfg.get("min_axis_speed_fraction"),
            DEFAULT_MIN_AXIS_SPEED_FRACTION,
            0.0,
            1.0,
        )
        self._turnaround_distance_deg = _positive_float(
            validation_cfg.get("path_turnaround_distance_deg"),
            _positive_float(
                jogging_cfg.get("path_cutoff_deg"),
                DEFAULT_PATH_TURNAROUND_DISTANCE_DEG,
            ),
        )
        self._limit_margin_rad = math.radians(
            _positive_float(validation_cfg.get("limit_margin_deg"), DEFAULT_LIMIT_MARGIN_DEG)
        )
        self._proximity_flip_distance_deg = _positive_float(
            validation_cfg.get("proximity_flip_distance_deg"),
            DEFAULT_PROXIMITY_FLIP_DISTANCE_DEG,
        )
        self._proximity_stale_ticks = max(
            1,
            _int_value(
                validation_cfg.get("proximity_stale_ticks"),
                DEFAULT_PROXIMITY_STALE_TICKS,
            ),
        )

        self._gear_ratio = _six_float_list(haptic_cfg.get("gear_ratio"), [1.0] * 6)
        self._q_min_rad = _six_deg_list(robot_cfg.get("q_limits_min_deg"), [-180.0] * 6)
        self._q_max_rad = _six_deg_list(robot_cfg.get("q_limits_max_deg"), [180.0] * 6)
        self._max_velocity_rad_s = _six_deg_list(
            robot_cfg.get("max_velocity_deg_s"), [20.0] * 6
        )

        self._latest_robot_q_rad: list[float] | None = None
        self._latest_planner_target_rad: list[float] | None = None
        self._robot_target_rad = [0.0] * 6
        self._robot_velocity_rad_s = [0.0] * 6
        self._seeded_from_robot = False
        self._run_requested = self._running
        self._running = False
        self._last_sample_s = self._now()
        self._randomize_count = 0
        self._last_randomize_reason = "startup"
        self._last_path_distance_deg: float | None = None
        self._prox_probe_offsets_deg: list[float] = []
        self._prox_hits: list[list[bool]] = [[] for _ in range(6)]
        self._prox_age_ticks: list[int] = [9999] * 6

        self._pygame = None
        self._screen = None
        self._font = None
        if self._ui_enabled:
            self._init_ui()

    @property
    def robot_velocity_rad_s(self) -> list[float]:
        """Return the current robot-space velocity for tests and diagnostics."""

        return list(self._robot_velocity_rad_s)

    @property
    def robot_target_rad(self) -> list[float]:
        """Return the current robot-space target for tests and diagnostics."""

        return list(self._robot_target_rad)

    def set_running(self, enabled: bool) -> None:
        """Set whether the generator is moving or holding position.

        This is called by the checkbox UI and by tests. A transition from
        running to paused latches the virtual target to the latest robot pose
        exactly once, which avoids continuously chasing robot feedback.
        """

        enabled = bool(enabled)
        self._run_requested = enabled
        if self._running == enabled:
            return
        self._running = enabled
        if enabled:
            if not self._seeded_from_robot:
                self._running = False
                self._last_randomize_reason = "waiting_for_robot_actual"
                return
            self._running = True
            self._last_sample_s = self._now()
            self._randomize_velocity(reason="resume")
            return
        self._running = False
        self._latch_to_latest_planner_target(reason="pause")

    def update_robot_actual(self, q_rad: list[float]) -> None:
        """Store robot feedback and seed the paused target once at startup."""

        if len(q_rad) < 6:
            return
        self._latest_robot_q_rad = [float(v) for v in q_rad[:6]]
        if not self._seeded_from_robot:
            self._latch_to_latest_robot(reason="initial_robot_actual")
            if self._run_requested:
                self._running = True
                self._last_sample_s = self._now()
                self._randomize_velocity(reason="startup_after_robot_actual")

    def update_state_full(self, body: dict[str, Any]) -> None:
        """Consume planner collision state and reroll blocked trajectories."""

        planner_target = _planner_target_rad(body, self._team)
        if planner_target is not None:
            self._latest_planner_target_rad = planner_target
        proximity = _proximity_state(body, self._team)
        if proximity is not None:
            self._prox_probe_offsets_deg = proximity["offsets_deg"]
            self._prox_hits = proximity["hits"]
            self._prox_age_ticks = proximity["age_ticks"]

        distance_deg = _path_collision_distance_deg(body, self._team)
        self._last_path_distance_deg = distance_deg
        if not self._running:
            return
        if distance_deg is None:
            return
        if distance_deg <= self._turnaround_distance_deg:
            self._latch_to_latest_planner_target(reason="path_collision_base")
            self._randomize_velocity(reason="path_collision")

    def sample(self) -> dict[str, Any] | None:
        """Return one haptic sample, or None before robot actual seeds it."""

        self._poll_ui()
        if not self._seeded_from_robot:
            self._robot_velocity_rad_s = [0.0] * 6
            return None

        now_s = self._now()
        dt_s = max(1e-3, min(0.1, now_s - self._last_sample_s))
        self._last_sample_s = now_s

        if self._running:
            self._integrate_target(dt_s)
        else:
            self._robot_velocity_rad_s = [0.0] * 6

        dial_pos_rad = [
            _robot_to_dial_rad(self._robot_target_rad[axis], self._gear_ratio[axis])
            for axis in range(6)
        ]
        dial_vel_rad_s = [
            _robot_to_dial_rad(self._robot_velocity_rad_s[axis], self._gear_ratio[axis])
            for axis in range(6)
        ]
        return {
            "dial_pos_rad": dial_pos_rad,
            "dial_vel_rad_s": dial_vel_rad_s,
            "board_connected": [True] * 6,
            "board_loop_hz": [int(PUBLISH_HZ)] * 6,
            "validation": {
                "running": self._running,
                "randomize_count": self._randomize_count,
                "last_randomize_reason": self._last_randomize_reason,
                "last_path_distance_deg": self._last_path_distance_deg,
                "robot_velocity_rad_s": list(self._robot_velocity_rad_s),
                "robot_target_rad": list(self._robot_target_rad),
            },
        }

    def close(self) -> None:
        """Close the optional Pygame UI resources."""

        if self._pygame is None:
            return
        try:
            self._pygame.quit()
        except Exception:
            pass

    def _init_ui(self) -> None:
        """Create the minimal checkbox UI used to start and stop motion."""

        import pygame

        pygame.init()
        self._pygame = pygame
        self._screen = pygame.display.set_mode(DEFAULT_WINDOW_SIZE)
        pygame.display.set_caption("Random trajectory validation")
        self._font = pygame.font.SysFont("Segoe UI", 20)

    def _poll_ui(self) -> None:
        """Process checkbox, keyboard, and redraw events without blocking."""

        if self._pygame is None or self._screen is None:
            return
        checkbox_rect = self._pygame.Rect(24, 28, 28, 28)
        for event in self._pygame.event.get():
            if event.type == self._pygame.QUIT:
                self.set_running(False)
            elif event.type == self._pygame.KEYDOWN and event.key == self._pygame.K_SPACE:
                self.set_running(not self._running)
            elif event.type == self._pygame.MOUSEBUTTONDOWN and event.button == 1:
                if checkbox_rect.collidepoint(event.pos):
                    self.set_running(not self._running)
        self._draw_ui(checkbox_rect)

    def _draw_ui(self, checkbox_rect) -> None:
        """Render the checkbox and live validation status text."""

        assert self._pygame is not None
        assert self._screen is not None
        assert self._font is not None
        bg_color = (22, 24, 28)
        box_color = (210, 220, 230)
        active_color = (80, 200, 130)
        text_color = (235, 238, 242)
        muted_color = (150, 160, 170)
        self._screen.fill(bg_color)
        self._pygame.draw.rect(self._screen, box_color, checkbox_rect, width=2)
        if self._running:
            inner = checkbox_rect.inflate(-8, -8)
            self._pygame.draw.rect(self._screen, active_color, inner)
        label = "Run random trajectory"
        status = "RUNNING" if self._running else "PAUSED"
        reason = f"last turn: {self._last_randomize_reason}"
        count = f"turns: {self._randomize_count}"
        self._screen.blit(self._font.render(label, True, text_color), (66, 28))
        self._screen.blit(self._font.render(status, True, active_color if self._running else muted_color), (24, 76))
        self._screen.blit(self._font.render(reason, True, muted_color), (24, 108))
        self._screen.blit(self._font.render(count, True, muted_color), (24, 138))
        self._pygame.display.flip()

    def _integrate_target(self, dt_s: float) -> None:
        """Advance the robot-space target and randomize at joint limits."""

        next_target = [
            self._robot_target_rad[axis] + self._robot_velocity_rad_s[axis] * dt_s
            for axis in range(6)
        ]
        hit_limit = False
        for axis in range(6):
            if next_target[axis] < self._q_min_rad[axis]:
                next_target[axis] = self._q_min_rad[axis]
                hit_limit = True
            elif next_target[axis] > self._q_max_rad[axis]:
                next_target[axis] = self._q_max_rad[axis]
                hit_limit = True
        self._robot_target_rad = next_target
        if hit_limit:
            self._randomize_velocity(reason="joint_limit")

    def _latch_to_latest_robot(self, *, reason: str) -> None:
        """Set the virtual target to the latest robot pose one time."""

        if self._latest_robot_q_rad is not None:
            self._robot_target_rad = [
                _clamp(
                    self._latest_robot_q_rad[axis],
                    self._q_min_rad[axis],
                    self._q_max_rad[axis],
                )
                for axis in range(6)
            ]
            self._seeded_from_robot = True
        self._robot_velocity_rad_s = [0.0] * 6
        self._last_randomize_reason = reason

    def _latch_to_latest_planner_target(self, *, reason: str) -> None:
        """Freeze the virtual target at the last planner servo target."""

        if self._latest_planner_target_rad is not None:
            self._robot_target_rad = [
                _clamp(
                    self._latest_planner_target_rad[axis],
                    self._q_min_rad[axis],
                    self._q_max_rad[axis],
                )
                for axis in range(6)
            ]
        self._robot_velocity_rad_s = [0.0] * 6
        self._last_randomize_reason = reason

    def _randomize_velocity(self, *, reason: str) -> None:
        """Choose a new fixed robot-space velocity within profile limits."""

        velocity: list[float] = []
        for axis in range(6):
            max_speed = abs(self._max_velocity_rad_s[axis]) * self._speed_scale
            min_speed = max_speed * self._min_axis_speed_fraction
            if max_speed <= 0.0:
                velocity.append(0.0)
                continue
            speed = self._rng.uniform(min_speed, max_speed)
            sign = -1.0 if self._rng.random() < 0.5 else 1.0
            sign = self._proximity_biased_sign(axis, sign)
            if self._robot_target_rad[axis] <= self._q_min_rad[axis] + self._limit_margin_rad:
                sign = 1.0
            elif self._robot_target_rad[axis] >= self._q_max_rad[axis] - self._limit_margin_rad:
                sign = -1.0
            velocity.append(sign * speed)
        self._robot_velocity_rad_s = velocity
        self._randomize_count += 1
        self._last_randomize_reason = reason

    def _proximity_biased_sign(self, axis: int, sign: float) -> float:
        """Flip sign if recent proximity probes say the chosen side is tight."""

        side_clearance = self._axis_clearance_deg(axis)
        if side_clearance is None:
            return sign
        chosen_key = "pos" if sign >= 0.0 else "neg"
        opposite_key = "neg" if chosen_key == "pos" else "pos"
        chosen_clearance = side_clearance.get(chosen_key)
        opposite_clearance = side_clearance.get(opposite_key)
        if chosen_clearance is None:
            return sign
        if chosen_clearance > self._proximity_flip_distance_deg:
            return sign
        if opposite_clearance is None or opposite_clearance > chosen_clearance:
            return -sign
        return sign

    def _axis_clearance_deg(self, axis: int) -> dict[str, float | None] | None:
        """Return nearest positive and negative proximity hit distances."""

        if axis < 0 or axis >= 6:
            return None
        if axis >= len(self._prox_age_ticks):
            return None
        if self._prox_age_ticks[axis] > self._proximity_stale_ticks:
            return None
        if axis >= len(self._prox_hits):
            return None
        axis_hits = self._prox_hits[axis]
        if len(axis_hits) != len(self._prox_probe_offsets_deg):
            return None

        nearest = {"pos": None, "neg": None}
        for offset_deg, hit in zip(self._prox_probe_offsets_deg, axis_hits):
            if not hit:
                continue
            distance_deg = abs(float(offset_deg))
            if offset_deg > 0.0:
                prev = nearest["pos"]
                nearest["pos"] = distance_deg if prev is None else min(prev, distance_deg)
            elif offset_deg < 0.0:
                prev = nearest["neg"]
                nearest["neg"] = distance_deg if prev is None else min(prev, distance_deg)
        return nearest


def _mapping(value: Any) -> dict[str, Any]:
    """Return a dictionary config node or an empty mapping."""

    return value if isinstance(value, dict) else {}


def _int_value(value: Any, default: int) -> int:
    """Coerce a config value to int, falling back on invalid input."""

    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _nonnegative_float(value: Any, default: float) -> float:
    """Coerce a config value to a nonnegative float."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return max(0.0, number)


def _positive_float(value: Any, default: float) -> float:
    """Coerce a config value to a positive float."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if number > 0.0 else default


def _clamped_float(value: Any, default: float, lo: float, hi: float) -> float:
    """Coerce a config value to float and clamp it to [lo, hi]."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    return _clamp(number, lo, hi)


def _six_float_list(value: Any, fallback: list[float]) -> list[float]:
    """Return exactly six floats from a profile list."""

    source = value if isinstance(value, list) else fallback
    out: list[float] = []
    for axis in range(6):
        try:
            out.append(float(source[axis]))
        except (IndexError, TypeError, ValueError):
            out.append(float(fallback[axis]))
    return out


def _six_deg_list(value: Any, fallback_deg: list[float]) -> list[float]:
    """Return exactly six radians converted from a degree profile list."""

    return [math.radians(v) for v in _six_float_list(value, fallback_deg)]


def _path_collision_distance_deg(body: dict[str, Any], team: str) -> float | None:
    """Extract forward-path first-hit distance from a state.full payload."""

    if not isinstance(body, dict):
        return None
    teams = body.get("teams")
    if not isinstance(teams, dict):
        return None
    team_body = teams.get(team)
    if not isinstance(team_body, dict):
        return None
    collision = team_body.get("collision")
    if not isinstance(collision, dict):
        return None
    first_hit = collision.get("first_hit")
    if not isinstance(first_hit, dict):
        return None
    distance = first_hit.get("distance_deg")
    if not isinstance(distance, (int, float)):
        return None
    return float(distance)


def _planner_target_rad(body: dict[str, Any], team: str) -> list[float] | None:
    """Extract the latest planner robot target from a state.full payload."""

    if not isinstance(body, dict):
        return None
    teams = body.get("teams")
    if not isinstance(teams, dict):
        return None
    team_body = teams.get(team)
    if not isinstance(team_body, dict):
        return None
    robot = team_body.get("robot")
    if not isinstance(robot, dict):
        return None
    target = robot.get("q_target_rad")
    if not isinstance(target, list) or len(target) < 6:
        return None
    try:
        return [float(v) for v in target[:6]]
    except (TypeError, ValueError):
        return None


def _proximity_state(body: dict[str, Any], team: str) -> dict[str, Any] | None:
    """Extract proximity hit masks from a state.full payload."""

    if not isinstance(body, dict):
        return None
    teams = body.get("teams")
    if not isinstance(teams, dict):
        return None
    team_body = teams.get(team)
    if not isinstance(team_body, dict):
        return None
    collision = team_body.get("collision")
    if not isinstance(collision, dict):
        return None
    offsets_raw = collision.get("prox_probe_offsets_deg")
    hits_raw = collision.get("prox_hits")
    ages_raw = collision.get("prox_age_ticks")
    if not isinstance(offsets_raw, list) or not isinstance(hits_raw, list):
        return None
    if not isinstance(ages_raw, list):
        return None
    try:
        offsets = [float(v) for v in offsets_raw]
        ages = [int(v) for v in ages_raw[:6]]
    except (TypeError, ValueError):
        return None
    if len(ages) < 6:
        ages.extend([9999] * (6 - len(ages)))

    hits: list[list[bool]] = []
    for axis_hits in hits_raw[:6]:
        if not isinstance(axis_hits, list):
            hits.append([])
            continue
        hits.append([bool(v) for v in axis_hits])
    while len(hits) < 6:
        hits.append([])
    return {
        "offsets_deg": offsets,
        "hits": hits,
        "age_ticks": ages[:6],
    }


def _robot_to_dial_rad(robot_rad: float, gear_ratio: float) -> float:
    """Convert robot-space radians to dial-space radians."""

    gear = float(gear_ratio)
    if abs(gear) < 1e-9:
        gear = 1.0
    return float(robot_rad) / gear


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value between inclusive lower and upper limits."""

    if value < lo:
        return lo
    if value > hi:
        return hi
    return value
