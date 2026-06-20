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
DEFAULT_WINDOW_SIZE = (860, 390)


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
        batch_cfg = _mapping(profile.tuning.get("batch_validation"))
        haptic_cfg = _mapping(profile.tuning.get("haptic"))
        robot_cfg = _mapping(profile.tuning.get("robot"))
        jogging_cfg = _mapping(profile.tuning.get("jogging"))

        self._team = team  # Team key used to read this team's state.full payload.
        self._now = now_fn or time.perf_counter  # Monotonic clock used for integration.
        self._rng = random.Random(_int_value(validation_cfg.get("seed"), DEFAULT_SEED))
        self._batch_mode = bool(batch_cfg.get("enabled", False))
        self._active_stage: str | None = None
        self._game_index = 1
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
        self._last_axis_decisions = [_empty_axis_decision(axis) for axis in range(6)]

        self._pygame = None
        self._screen = None
        self._font = None
        self._small_font = None
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

        self._run_requested = bool(enabled)
        self._sync_running(reason="operator")

    def reset_for_game(self, *, seed: int, game_index: int) -> None:
        """Reseed and re-anchor the virtual target during batch tutorial."""

        self._rng.seed(int(seed))
        self._game_index = max(1, int(game_index))
        self._randomize_count = 0
        self._last_randomize_reason = "batch_game_reset"
        self._running = False
        self._latch_to_latest_robot(reason="batch_game_reset")
        self._sync_running(reason="batch_game_reset")

    def update_robot_actual(self, q_rad: list[float]) -> None:
        """Store robot feedback and seed the paused target once at startup."""

        if len(q_rad) < 6:
            return
        self._latest_robot_q_rad = [float(v) for v in q_rad[:6]]
        if not self._seeded_from_robot:
            self._latch_to_latest_robot(reason="initial_robot_actual")
            self._sync_running(reason="startup_after_robot_actual")

    def update_state_full(self, body: dict[str, Any]) -> None:
        """Consume planner collision state and reroll blocked trajectories."""

        active_stage = body.get("active_stage")
        if isinstance(active_stage, str) and active_stage != self._active_stage:
            self._active_stage = active_stage
            self._sync_running(reason=f"stage_{active_stage}")

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
                "game_index": self._game_index,
                "randomize_count": self._randomize_count,
                "last_randomize_reason": self._last_randomize_reason,
                "last_path_distance_deg": self._last_path_distance_deg,
                "robot_velocity_rad_s": list(self._robot_velocity_rad_s),
                "robot_target_rad": list(self._robot_target_rad),
                "axis_decisions": [dict(item) for item in self._last_axis_decisions],
            },
        }

    def _sync_running(self, *, reason: str) -> None:
        """Apply operator intent plus batch stage gating to generator motion."""

        stage_allows_motion = not self._batch_mode or self._active_stage == "play"
        should_run = self._run_requested and stage_allows_motion
        if should_run and not self._seeded_from_robot:
            self._running = False
            self._last_randomize_reason = "waiting_for_robot_actual"
            return
        if should_run == self._running:
            return
        if should_run:
            self._running = True
            self._last_sample_s = self._now()
            self._randomize_velocity(reason=reason)
            return
        was_running = self._running
        self._running = False
        if was_running:
            self._latch_to_latest_planner_target(reason=reason)

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
        self._small_font = pygame.font.SysFont("Consolas", 15)

    def _poll_ui(self) -> None:
        """Process checkbox, keyboard, and redraw events without blocking."""

        if self._pygame is None or self._screen is None:
            return
        checkbox_rect = self._pygame.Rect(24, 24, 28, 28)
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
        """Render live trajectory, proximity, and direction decisions."""

        assert self._pygame is not None
        assert self._screen is not None
        assert self._font is not None
        assert self._small_font is not None
        bg_color = (22, 24, 28)
        box_color = (210, 220, 230)
        active_color = (80, 200, 130)
        text_color = (235, 238, 242)
        muted_color = (150, 160, 170)
        table_color = (40, 44, 50)
        line_color = (72, 78, 88)
        warning_color = (245, 170, 80)
        self._screen.fill(bg_color)
        self._pygame.draw.rect(self._screen, box_color, checkbox_rect, width=2)
        if self._running:
            inner = checkbox_rect.inflate(-8, -8)
            self._pygame.draw.rect(self._screen, active_color, inner)
        label = "Run random trajectory"
        status = "RUNNING" if self._running else "PAUSED"
        path = _format_distance(self._last_path_distance_deg)
        self._screen.blit(self._font.render(label, True, text_color), (66, 24))
        status_color = active_color if self._running else muted_color
        self._screen.blit(self._font.render(status, True, status_color), (660, 22))

        summary = (
            f"turns={self._randomize_count}  last={self._last_randomize_reason}  "
            f"path_hit_deg={path}  flip_if<= {self._proximity_flip_distance_deg:.1f}deg  "
            f"stale>{self._proximity_stale_ticks} ticks"
        )
        self._screen.blit(self._small_font.render(summary, True, muted_color), (24, 68))

        table_rect = self._pygame.Rect(18, 100, 824, 250)
        self._pygame.draw.rect(self._screen, table_color, table_rect)
        self._pygame.draw.rect(self._screen, line_color, table_rect, width=1)
        columns = [
            (30, "Axis"),
            (88, "Vel deg/s"),
            (178, "RNG->Out"),
            (238, "Target deg"),
            (342, "Prox -"),
            (432, "Prox +"),
            (522, "Age"),
            (580, "Decision used for last vector"),
        ]
        for x, title in columns:
            self._screen.blit(self._small_font.render(title, True, text_color), (x, 112))
        self._pygame.draw.line(self._screen, line_color, (24, 136), (836, 136), width=1)

        for axis in range(6):
            row_y = 146 + axis * 32
            if axis % 2 == 1:
                row_rect = self._pygame.Rect(24, row_y - 4, 812, 28)
                self._pygame.draw.rect(self._screen, (31, 35, 40), row_rect)
            decision = self._last_axis_decisions[axis]
            velocity_deg_s = math.degrees(self._robot_velocity_rad_s[axis])
            target_deg = math.degrees(self._robot_target_rad[axis])
            final_dir = str(decision.get("final_dir", "hold"))
            direction = f"{decision.get('rng_dir', 'hold')}->{final_dir}"
            direction_color = muted_color
            if final_dir == "pos":
                direction_color = active_color
            elif final_dir == "neg":
                direction_color = warning_color
            age = decision.get("prox_age_ticks")
            age_text = str(age) if isinstance(age, int) and age < 9000 else "stale"
            values = [
                (30, f"J{axis + 1}", text_color),
                (88, f"{velocity_deg_s:+7.2f}", direction_color),
                (178, direction, direction_color),
                (238, f"{target_deg:+8.2f}", text_color),
                (342, _format_distance(decision.get("prox_neg_deg")), muted_color),
                (432, _format_distance(decision.get("prox_pos_deg")), muted_color),
                (522, age_text, muted_color),
                (580, str(decision.get("decision", "")), text_color),
            ]
            for x, value, color in values:
                self._screen.blit(self._small_font.render(value, True, color), (x, row_y))

        footer = "Prox columns show nearest collision hit in each direction; '-' means no hit."
        self._screen.blit(self._small_font.render(footer, True, muted_color), (24, 362))
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
        decisions: list[dict[str, Any]] = []
        for axis in range(6):
            max_speed = abs(self._max_velocity_rad_s[axis]) * self._speed_scale
            min_speed = max_speed * self._min_axis_speed_fraction
            if max_speed <= 0.0:
                velocity.append(0.0)
                decisions.append(_empty_axis_decision(axis, decision="disabled"))
                continue
            speed = self._rng.uniform(min_speed, max_speed)
            rng_sign = -1.0 if self._rng.random() < 0.5 else 1.0
            sign, decision = self._proximity_sign_decision(axis, rng_sign)
            if self._robot_target_rad[axis] <= self._q_min_rad[axis] + self._limit_margin_rad:
                sign = 1.0
                decision = "joint_min_force_positive"
            elif self._robot_target_rad[axis] >= self._q_max_rad[axis] - self._limit_margin_rad:
                sign = -1.0
                decision = "joint_max_force_negative"
            velocity.append(sign * speed)
            decisions.append(
                self._axis_decision_record(
                    axis=axis,
                    rng_sign=rng_sign,
                    final_sign=sign,
                    speed_rad_s=speed,
                    decision=decision,
                )
            )
        self._robot_velocity_rad_s = velocity
        self._last_axis_decisions = decisions
        self._randomize_count += 1
        self._last_randomize_reason = reason

    def _proximity_biased_sign(self, axis: int, sign: float) -> float:
        """Flip sign if recent proximity probes say the chosen side is tight."""

        biased_sign, _decision = self._proximity_sign_decision(axis, sign)
        return biased_sign

    def _proximity_sign_decision(self, axis: int, sign: float) -> tuple[float, str]:
        """Return the proximity-biased sign and the decision reason."""

        side_clearance = self._axis_clearance_deg(axis)
        if side_clearance is None:
            return sign, "prox_missing_or_stale"
        chosen_key = "pos" if sign >= 0.0 else "neg"
        opposite_key = "neg" if chosen_key == "pos" else "pos"
        chosen_clearance = side_clearance.get(chosen_key)
        opposite_clearance = side_clearance.get(opposite_key)
        if chosen_clearance is None:
            return sign, "chosen_side_free"
        if chosen_clearance > self._proximity_flip_distance_deg:
            return sign, "chosen_side_far"
        if opposite_clearance is None or opposite_clearance > chosen_clearance:
            return -sign, "prox_flip"
        return sign, "opposite_side_tighter"

    def _axis_decision_record(
        self,
        *,
        axis: int,
        rng_sign: float,
        final_sign: float,
        speed_rad_s: float,
        decision: str,
    ) -> dict[str, Any]:
        """Build one row of UI/debug data for a generated velocity axis."""

        clearance = self._axis_clearance_deg(axis) or {}
        age = self._prox_age_ticks[axis] if axis < len(self._prox_age_ticks) else 9999
        return {
            "axis": axis + 1,
            "rng_dir": _direction_label(rng_sign),
            "final_dir": _direction_label(final_sign),
            "speed_deg_s": math.degrees(speed_rad_s),
            "signed_velocity_deg_s": math.degrees(final_sign * speed_rad_s),
            "prox_neg_deg": clearance.get("neg"),
            "prox_pos_deg": clearance.get("pos"),
            "prox_age_ticks": int(age),
            "prox_fresh": clearance != {},
            "decision": decision,
        }

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


def _empty_axis_decision(axis: int, *, decision: str = "not_generated") -> dict[str, Any]:
    """Return the default per-axis row shown before the first random vector."""

    return {
        "axis": axis + 1,
        "rng_dir": "hold",
        "final_dir": "hold",
        "speed_deg_s": 0.0,
        "signed_velocity_deg_s": 0.0,
        "prox_neg_deg": None,
        "prox_pos_deg": None,
        "prox_age_ticks": 9999,
        "prox_fresh": False,
        "decision": decision,
    }


def _direction_label(sign: float) -> str:
    """Format a sign as the compact direction label used in the UI."""

    if sign > 0.0:
        return "pos"
    if sign < 0.0:
        return "neg"
    return "hold"


def _format_distance(value: Any) -> str:
    """Format an optional degree distance for the dashboard table."""

    if isinstance(value, (int, float)):
        return f"{float(value):.1f}"
    return "-"


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
