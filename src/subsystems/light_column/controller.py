"""High-level, stateful light-column controller.

This is the "brain" of the LED system. It runs inside the 200 Hz process loop
and does two clearly separated jobs each tick:

1. :meth:`LedColumnController.update` reads the latest ``state.full`` game
   snapshot, dispatches to the per-stage render function, and writes the
   resulting colors **only into internal memory** (``self._strip_colors``). It
   never touches serial hardware.
2. :meth:`LedColumnController.pump` is the send scheduler. For each COM port it
   checks whether the configured inter-command spacing has elapsed and, if so,
   transmits the next strip's frame in a round-robin over that port's strips.
   Two ports drive 6 strips and one drives 4, so each strip is naturally
   refreshed at a slightly different rate - which is fine.

Keeping `update` (animation) and `pump` (transmission) separate means the
animation math is deterministic and unit-testable, and the RS485 timing is the
only thing that has to be careful.

Per-stage animation logic is split into one ``_render_<stage>`` method each so
the behaviour for any game stage can be tweaked in isolation later.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

from subsystems.light_column.frames import (
    BLUE,
    OFF,
    RED,
    WHITE,
    Color,
    make_color,
    scale,
    solid,
    two_color_split,
)
from subsystems.light_column.layout import LightColumnLayout
from subsystems.light_column.transport import LedTransport, build_strip_frame


# Default team colors. Team A is blue, Team B is red (matches the dashboard and
# the rest of the installation). Override via tuning.light_column.team_colors.
_DEFAULT_TEAM_COLORS: dict[str, Color] = {"a": BLUE, "b": RED}


@dataclass
class LightColumnConfig:
    """Tunable timing/appearance parameters for the light columns.

    All durations are in seconds. Defaults are sensible for the arena and can
    be overridden per-profile under ``tuning.light_column`` (see
    :meth:`from_profile`).
    """

    # Per-port minimum gap between RS485 frames. Sourced from the device file;
    # going below ~2 ms at 921600 baud corrupts colors on the wire.
    inter_command_delay_s: float = 0.002
    # Daydreaming breathing: one full dim->bright->dim cycle, and the lowest
    # brightness reached (1.0 = full color).
    breathing_period_s: float = 3.0
    breathing_min_brightness: float = 0.15
    # Play stage: how long every strip stays dark at the very start of play.
    play_startup_blackout_s: float = 0.5
    # Play stage end-game white flash: active during the final
    # `endgame_flash_window_s` (driven by published countdown_s); within each
    # `endgame_flash_period_s` the strips flash solid white for
    # `endgame_flash_on_s`.
    endgame_flash_window_s: float = 5.0
    endgame_flash_period_s: float = 1.0
    endgame_flash_on_s: float = 0.2
    # Conclusion: hold the finished score bars this long after the count
    # finishes before flipping the losing team to the winner's color.
    conclusion_post_count_hold_s: float = 1.0
    # Per-team solid color used for team strips, indicators, and bars.
    team_colors: dict[str, Color] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        """Fill in default team colors if none were supplied."""

        if self.team_colors is None:
            self.team_colors = dict(_DEFAULT_TEAM_COLORS)

    @classmethod
    def from_profile(cls, profile: Any) -> "LightColumnConfig":
        """Build a config from a loaded profile's ``tuning`` block.

        Reads ``tuning.light_column`` for overrides. Tutorial progress bars are
        driven by the per-player progress published in state.full, so no
        tutorial scroll scale is needed here.
        """

        tuning = getattr(profile, "tuning", {}) or {}
        light = tuning.get("light_column") if isinstance(tuning, dict) else {}
        light = light if isinstance(light, dict) else {}

        config = cls()
        config.breathing_period_s = _pos_float(
            light.get("breathing_period_s"), config.breathing_period_s
        )
        config.breathing_min_brightness = _clamp01(
            light.get("breathing_min_brightness"), config.breathing_min_brightness
        )
        config.play_startup_blackout_s = _non_neg_float(
            light.get("play_startup_blackout_s"), config.play_startup_blackout_s
        )
        config.endgame_flash_window_s = _non_neg_float(
            light.get("endgame_flash_window_s"), config.endgame_flash_window_s
        )
        config.endgame_flash_period_s = _pos_float(
            light.get("endgame_flash_period_s"), config.endgame_flash_period_s
        )
        config.endgame_flash_on_s = _non_neg_float(
            light.get("endgame_flash_on_s"), config.endgame_flash_on_s
        )
        config.conclusion_post_count_hold_s = _non_neg_float(
            light.get("conclusion_post_count_hold_s"),
            config.conclusion_post_count_hold_s,
        )
        team_colors = light.get("team_colors")
        if isinstance(team_colors, dict):
            for team in ("a", "b"):
                parsed = _parse_color(team_colors.get(team))
                if parsed is not None:
                    config.team_colors[team] = parsed
        return config


class LedColumnController:
    """Own the desired LED state and drive the per-port send scheduler.

    Args:
        transport: Open (or openable) low-level serial transport.
        layout: Strip groupings and counts.
        config: Timing/appearance tunables.
    """

    def __init__(
        self,
        transport: LedTransport,
        layout: LightColumnLayout,
        config: LightColumnConfig,
    ) -> None:
        self._transport = transport
        self._layout = layout
        self._config = config
        self._n = layout.leds_per_strip

        # Desired color frame per strip; starts dark for every team strip.
        self._strip_colors: dict[int, list[Color]] = {
            strip_id: solid(OFF, self._n) for strip_id in layout.all_strips
        }
        # Per-port send pacing: last transmit time and round-robin cursor.
        self._last_sent_mono: dict[str, float] = {}
        self._send_cursor: dict[str, int] = {}

        # Latest game snapshot (set by the process loop via set_state).
        self._state: dict[str, Any] = {}

        # Local stage tracking (cross-process monotonic clocks are not
        # comparable, so we time stage entry against our own clock).
        self._stage: str | None = None
        self._stage_entered_mono: float = 0.0
        # Conclusion: local timestamp when both teams finished counting.
        self._conclusion_done_mono: float | None = None

        # Stage -> render function dispatch table.
        self._renderers: dict[str, Callable[[dict[str, Any], float, float], None]] = {
            "daydreaming": self._render_daydreaming,
            "idle": self._render_idle,
            "tutorial": self._render_tutorial,
            "play": self._render_play,
            "reset": self._render_reset,
            "conclusion": self._render_conclusion,
        }

    # ---- inputs ---------------------------------------------------------

    def set_state(self, state: dict[str, Any]) -> None:
        """Store the most recent ``state.full`` body for the next update."""

        if isinstance(state, dict):
            self._state = state

    # ---- animation (memory only) ---------------------------------------

    def update(self, now_mono: float, now_wall: float) -> None:
        """Recompute desired strip colors for the current game stage.

        Writes only to internal memory. ``now_mono`` is a process-local
        monotonic clock (``time.perf_counter``); ``now_wall`` is wall-clock
        (``time.time``) used for the breathing phase.
        """

        state = self._state
        # Use active_stage so the LEDs reflect the real stage even while the
        # controller reports a "paused" display stage.
        stage = state.get("active_stage") or state.get("stage")
        if not stage:
            # No game state yet: keep everything dark until the game appears.
            self._fill_all(OFF)
            return

        if stage != self._stage:
            self._stage = stage
            self._stage_entered_mono = now_mono
            self._conclusion_done_mono = None

        renderer = self._renderers.get(stage, self._render_idle)
        renderer(state, now_mono, now_wall)

    # ---- transmission (paced serial sends) -----------------------------

    def pump(self, now_mono: float) -> None:
        """Send at most one frame per open COM port, respecting line spacing.

        Round-robins over each port's strips so all of them refresh over time.
        """

        spacing = self._config.inter_command_delay_s
        for port in self._transport.ports():
            last = self._last_sent_mono.get(port, -1e9)
            if now_mono - last < spacing:
                continue
            strips = self._transport.strips_for_port(port)
            if not strips:
                continue
            cursor = self._send_cursor.get(port, 0) % len(strips)
            strip_id = strips[cursor]
            self._send_cursor[port] = (cursor + 1) % len(strips)
            colors = self._strip_colors.get(strip_id)
            if colors is None:
                continue
            self._transport.write(port, build_strip_frame(strip_id, colors))
            self._last_sent_mono[port] = now_mono

    # ---- per-stage renderers -------------------------------------------

    def _render_daydreaming(
        self, state: dict[str, Any], now_mono: float, now_wall: float
    ) -> None:
        """Attract mode: each team's strips solid team color, breathing slowly."""

        period = self._config.breathing_period_s
        low = self._config.breathing_min_brightness
        # 0..1 sine that spends equal time bright and dim across the period.
        phase = (now_wall % period) / period if period > 0 else 0.0
        wave = 0.5 - 0.5 * math.cos(phase * 2.0 * math.pi)
        brightness = low + (1.0 - low) * wave
        for team in ("a", "b"):
            color = scale(self._config.team_colors[team], brightness)
            self._set_team(team, solid(color, self._n))

    def _render_idle(
        self, state: dict[str, Any], now_mono: float, now_wall: float
    ) -> None:
        """Idle: steady solid team colors, no animation."""

        for team in ("a", "b"):
            self._set_team(team, solid(self._config.team_colors[team], self._n))

    def _render_tutorial(
        self, state: dict[str, Any], now_mono: float, now_wall: float
    ) -> None:
        """Tutorial: team-color indicators + per-player viewing-progress bars."""

        for team in ("a", "b"):
            team_color = self._config.team_colors[team]
            # Indicator strips show the flat team color.
            for strip_id in self._layout.team_indicator_strips[team]:
                self._strip_colors[strip_id] = solid(team_color, self._n)
            # Each player's strip is a progress bar from their published
            # tutorial scroll progress (0..100%, computed by the game
            # controller from the dial position).
            progress_pct = _team_tutorial_progress_pct(state, team)
            for player, strip_id in self._layout.tutorial_player_strips[team].items():
                index = _player_dial_index(player)
                value = progress_pct[index] if 0 <= index < len(progress_pct) else 0.0
                progress = _clamp01(value / 100.0, 0.0)
                self._strip_colors[strip_id] = two_color_split(
                    team_color, OFF, progress, self._n
                )

    def _render_play(
        self, state: dict[str, Any], now_mono: float, now_wall: float
    ) -> None:
        """Play: 0.5 s blackout, then team-color speed bars, end-game flashes."""

        elapsed = now_mono - self._stage_entered_mono
        if elapsed < self._config.play_startup_blackout_s:
            self._fill_all(OFF)
            return

        # End-game white flash overrides everything during the final window.
        countdown = _maybe_float(state.get("countdown_s"))
        if (
            countdown is not None
            and countdown <= self._config.endgame_flash_window_s
            and self._flash_on(now_mono)
        ):
            self._fill_all(WHITE)
            return

        # Normal play: each team's strips show its speed scale as a fill bar.
        for team in ("a", "b"):
            scalar = _team_final_scalar(state, team)
            bar = two_color_split(self._config.team_colors[team], OFF, scalar, self._n)
            self._set_team(team, bar)

    def _render_reset(
        self, state: dict[str, Any], now_mono: float, now_wall: float
    ) -> None:
        """Reset: every strip solid white while the robots return to start."""

        self._fill_all(WHITE)

    def _render_conclusion(
        self, state: dict[str, Any], now_mono: float, now_wall: float
    ) -> None:
        """Conclusion: follow published score countdown, then crown the winner.

        While the game counts buckets down, each team's accumulated
        ``summed_score`` fills its column (normalized so the higher team's total
        reaches the full height). Once both teams finish, the bars hold for
        ``conclusion_post_count_hold_s`` and then the losing team's strips switch
        to the winner's color, which is held through the rewind.
        """

        totals = {team: _team_score_total(state, team) for team in ("a", "b")}
        filled = {team: _team_summed_score(state, team) for team in ("a", "b")}
        scale_max = max(1.0, totals["a"], totals["b"])

        both_done = all(_team_conclusion_done(state, team) for team in ("a", "b"))
        if both_done and self._conclusion_done_mono is None:
            self._conclusion_done_mono = now_mono

        winner = "a" if totals["a"] >= totals["b"] else "b"
        loser = "b" if winner == "a" else "a"

        if (
            self._conclusion_done_mono is not None
            and now_mono - self._conclusion_done_mono
            >= self._config.conclusion_post_count_hold_s
        ):
            # Crown phase: every strip becomes the winner's color.
            winner_color = self._config.team_colors[winner]
            self._set_team(winner, solid(winner_color, self._n))
            self._set_team(loser, solid(winner_color, self._n))
            return

        # Counting / hold phase: render each team's normalized fill bar.
        for team in ("a", "b"):
            progress = filled[team] / scale_max
            bar = two_color_split(self._config.team_colors[team], OFF, progress, self._n)
            self._set_team(team, bar)

    # ---- helpers --------------------------------------------------------

    def _flash_on(self, now_mono: float) -> bool:
        """Return True during the 'on' slice of the end-game flash duty cycle."""

        period = self._config.endgame_flash_period_s
        if period <= 0:
            return False
        return (now_mono % period) < self._config.endgame_flash_on_s

    def _set_team(self, team: str, colors: list[Color]) -> None:
        """Write the same frame to every strip owned by ``team``."""

        for strip_id in self._layout.team_strips[team]:
            self._strip_colors[strip_id] = list(colors)

    def _fill_all(self, color: Color) -> None:
        """Set every team strip to a single solid color."""

        frame = solid(color, self._n)
        for strip_id in self._strip_colors:
            self._strip_colors[strip_id] = list(frame)

    # ---- test/inspection accessors -------------------------------------

    def strip_colors(self, strip_id: int) -> list[Color]:
        """Return the current desired frame for one strip (defensive copy)."""

        return list(self._strip_colors.get(strip_id, ()))


# ---- module-level state extraction + coercion helpers -------------------


def _team_block(state: dict[str, Any], team: str) -> dict[str, Any]:
    """Return ``state.full.teams.<team>`` as a dict (empty if absent)."""

    teams = state.get("teams")
    if not isinstance(teams, dict):
        return {}
    block = teams.get(team)
    return block if isinstance(block, dict) else {}


def _team_final_scalar(state: dict[str, Any], team: str) -> float:
    """Return the team's speed scale (1.0 = full speed) clamped to [0, 1]."""

    collision = _team_block(state, team).get("collision")
    collision = collision if isinstance(collision, dict) else {}
    return _clamp01(collision.get("final_scalar"), 1.0)


def _team_tutorial_progress_pct(state: dict[str, Any], team: str) -> list[float]:
    """Return the team's 6 per-player tutorial progress values (0..100%)."""

    haptic = _team_block(state, team).get("haptic")
    haptic = haptic if isinstance(haptic, dict) else {}
    values = haptic.get("tutorial_progress_pct")
    if isinstance(values, list):
        return [_maybe_float(v) or 0.0 for v in values]
    return [0.0] * 6


def _team_summed_score(state: dict[str, Any], team: str) -> float:
    """Return the team's accumulated countdown score so far."""

    return _maybe_float(_team_block(state, team).get("summed_score")) or 0.0


def _team_score_total(state: dict[str, Any], team: str) -> float:
    """Return the team's full score total (accumulated + remaining buckets)."""

    block = _team_block(state, team)
    summed = _maybe_float(block.get("summed_score")) or 0.0
    buckets = block.get("buckets")
    remaining = (
        sum((_maybe_float(v) or 0.0) for v in buckets)
        if isinstance(buckets, list)
        else 0.0
    )
    return summed + remaining


def _team_conclusion_done(state: dict[str, Any], team: str) -> bool:
    """Return whether the team's conclusion countdown has finished."""

    conclusion = _team_block(state, team).get("conclusion")
    conclusion = conclusion if isinstance(conclusion, dict) else {}
    return bool(conclusion.get("done", False))


def _player_dial_index(player: str) -> int:
    """Map a tutorial player label (e.g. ``A3``) to a 0-based dial index."""

    digits = "".join(ch for ch in str(player) if ch.isdigit())
    if not digits:
        return -1
    return int(digits) - 1


def _maybe_float(value: Any) -> float | None:
    """Coerce to float, returning None on failure."""

    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _clamp01(value: Any, default: float = 0.0) -> float:
    """Coerce to float and clamp into [0, 1], using ``default`` on failure."""

    parsed = _maybe_float(value)
    if parsed is None:
        parsed = default
    if parsed < 0.0:
        return 0.0
    if parsed > 1.0:
        return 1.0
    return parsed


def _pos_float(value: Any, default: float) -> float:
    """Coerce to a strictly-positive float, else return ``default``."""

    parsed = _maybe_float(value)
    return parsed if parsed is not None and parsed > 0.0 else default


def _non_neg_float(value: Any, default: float) -> float:
    """Coerce to a non-negative float, else return ``default``."""

    parsed = _maybe_float(value)
    return parsed if parsed is not None and parsed >= 0.0 else default


def _parse_color(value: Any) -> Color | None:
    """Parse a ``[r, g, b]`` list or ``#RRGGBB`` string into a Color."""

    if isinstance(value, (list, tuple)) and len(value) == 3:
        return make_color(*(float(c) for c in value))
    if isinstance(value, str):
        text = value.strip().lstrip("#")
        if len(text) == 6:
            try:
                return make_color(
                    int(text[0:2], 16), int(text[2:4], 16), int(text[4:6], 16)
                )
            except ValueError:
                return None
    return None
