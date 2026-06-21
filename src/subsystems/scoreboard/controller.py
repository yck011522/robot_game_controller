"""High-level, stateful scoreboard controller.

This is the "brain" of the per-bucket LED scoreboard. It runs inside the process
loop and does two clearly separated jobs each tick:

1. :meth:`ScoreboardController.update` reads the latest ``state.full`` game
   snapshot, dispatches to the per-stage render function, and writes the desired
   per-panel text **only into internal memory**. It diffs that against what has
   already been queued and enqueues only the changed command lines.
2. :meth:`ScoreboardController.pump` drains the command queue to the serial
   transport, optionally pacing one line per ``inter_command_delay_s``.

Keeping ``update`` (content) and ``pump`` (transmission) separate makes the
stage logic deterministic and unit-testable, and keeps serial timing isolated.

Per-stage behaviour (matches docs/GAME_MECHANICS.md and the bring-up spec):

* daydreaming - every panel blank (text layer disabled).
* idle        - every active bucket panel shows ``0000`` (static).
* tutorial    - bucket 1/2/3 of each team show ``HOW`` / ``TO`` / ``PLAY``.
* play        - each panel shows its bucket weight, 4-digit zero-padded.
* reset       - every panel scrolls ``GAME,OVER`` upward (mode 1).
* conclusion  - while the score counts down: bucket weights (as in play, now
                decrementing). Once the show moves from the announcement pose to
                the win/lose pose: the winning team's panels show ``WIN``, the
                losing team's ``LOSE``, or every panel ``TIE`` on an integer tie.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable

from subsystems.scoreboard.layout import ScoreboardLayout
from subsystems.scoreboard.transport import (
    MODE_SCROLL_UP,
    MODE_STATIC,
    ScoreboardTransport,
    cmd_brightness,
    cmd_color,
    cmd_enable,
    cmd_mode,
    cmd_text,
    cmd_text_brightness,
)


# Pure white, used for every readable stage except the tutorial (which uses the
# per-team colour) so live scores / results are maximally legible.
_WHITE = (255, 255, 255)



# Conclusion-show phases (published under ``teams.<t>.conclusion.phase``) that
# mean "the robot has left the announcement pose and is revealing the result".
# While the controller is in one of these phases - or once ``conclusion.done``
# is set - the panels switch from the counting numbers to WIN/LOSE/TIE words.
_REVEAL_PHASES = frozenset(
    {"move_to_winner_pose", "winner_pose_hold", "move_to_begin"}
)

_TEAMS = ("a", "b")


@dataclass(frozen=True)
class DisplayState:
    """Desired state of one LED panel.

    A disabled panel is always normalized to ``mode=MODE_STATIC``, empty text and
    no colour so two "blank" states compare equal and never trigger a spurious
    resend.

    Attributes:
        enable: Whether the text layer is on (False blanks the panel).
        mode: ``MODE_STATIC`` (0) or ``MODE_SCROLL_UP`` (1).
        text: The string to show (sent verbatim; commas split lines in scroll).
        color: Optional ``(r, g, b)`` text colour (0..255 each). ``None`` leaves
            the panel's current colour untouched (no colour command emitted).
    """

    enable: bool
    mode: int = MODE_STATIC
    text: str = ""
    color: tuple[int, int, int] | None = None


# Canonical blank panel (text layer off). Reused so equality checks are cheap.
# ``initialize()`` also drives the hardware to this exact state at startup so the
# diff baseline and the physical panels agree.
_BLANK = DisplayState(enable=False, mode=MODE_STATIC, text="", color=None)



@dataclass
class ScoreboardConfig:
    """Tunable text + timing parameters for the scoreboard.

    Defaults are production-ready; override per-profile under
    ``tuning.scoreboard`` (see :meth:`from_profile`).
    """

    # Minimum gap between serial command lines. 0.0 flushes the whole pending
    # queue every tick (the panel firmware accepts back-to-back lines); raise it
    # only if the RS485 link needs breathing room (then one line is sent per
    # this many seconds).
    inter_command_delay_s: float = 0.0
    # Minimum time between successive enqueues for the *same* panel. This caps
    # how fast a rapidly-changing number (the conclusion count-down) is pushed
    # to a panel so the bus is not flooded; 0.05 s == up to 20 updates/s/panel.
    min_refresh_interval_s: float = 0.05
    # Number of zero-padded digits used for bucket-weight readouts. Values are
    # clamped to this width (e.g. 4 -> 0..9999) so a panel never overflows.
    score_digits: int = 4
    # Text shown per stage / outcome. Tweak casing/wording here if the panel
    # font or the desired copy changes.
    idle_text: str = "0000"
    tutorial_words: tuple[str, str, str] = ("HOW", "TO", "PLAY")
    game_over_text: str = "GAME,OVER"
    win_text: str = "WIN"
    lose_text: str = "LOSE"
    tie_text: str = "TIE"

    # Per-team text colour used **only** during the tutorial (HOW/TO/PLAY) so the
    # boards visually match each team's light column. Keys are team ids ("a"/"b")
    # and values are (r, g, b) 0..255 tuples. Override under
    # ``tuning.scoreboard.team_colors`` (accepts "#RRGGBB" or [r, g, b]); if that
    # is absent the loader falls back to ``tuning.light_column.team_colors`` so
    # the two subsystems stay in sync from a single source. Defaults: A blue,
    # B red (see __post_init__).
    team_colors: dict[str, tuple[int, int, int]] = field(default_factory=dict)
    # Text colour for every readable, non-tutorial stage (idle, play, reset,
    # conclusion counting + WIN/LOSE/TIE). White is the most legible default.
    play_color: tuple[int, int, int] = _WHITE
    # Whole-display + text-layer brightness pushed to every panel by
    # initialize() at startup (0..255). Max by default so a panel left dim by a
    # previous run is forced bright.
    init_brightness: int = 255
    # Winner-blink cadence on the conclusion WIN reveal. The winning team's
    # panels toggle their text on/off; period is one full on+off cycle (seconds)
    # and on_fraction is the duty cycle (0.5 == equal on/off). Loser (LOSE) and
    # tie (TIE) panels stay steady.
    blink_period_s: float = 0.6
    blink_on_fraction: float = 0.5

    def __post_init__(self) -> None:
        """Seed default team colours (A=blue, B=red) when none were provided."""

        if not self.team_colors:
            self.team_colors = {"a": (0, 0, 255), "b": (255, 0, 0)}

    @classmethod
    def from_profile(cls, profile: Any) -> "ScoreboardConfig":
        """Build a config from a loaded profile's ``tuning.scoreboard`` block."""

        tuning = getattr(profile, "tuning", {}) or {}
        node = tuning.get("scoreboard") if isinstance(tuning, dict) else {}
        node = node if isinstance(node, dict) else {}

        config = cls()
        config.inter_command_delay_s = _non_neg_float(
            node.get("inter_command_delay_s"), config.inter_command_delay_s
        )
        config.min_refresh_interval_s = _non_neg_float(
            node.get("min_refresh_interval_s"), config.min_refresh_interval_s
        )
        digits = node.get("score_digits")
        if isinstance(digits, int) and digits >= 1:
            config.score_digits = digits
        words = node.get("tutorial_words")
        if isinstance(words, list) and len(words) >= 3:
            config.tutorial_words = (str(words[0]), str(words[1]), str(words[2]))
        for attr in ("idle_text", "game_over_text", "win_text", "lose_text", "tie_text"):
            value = node.get(attr)
            if isinstance(value, str) and value:
                setattr(config, attr, value)

        # Brightness + blink tunables.
        brightness = _maybe_int(node.get("init_brightness"))
        if brightness is not None:
            config.init_brightness = max(0, min(255, brightness))
        config.blink_period_s = _non_neg_float(
            node.get("blink_period_s"), config.blink_period_s
        )
        config.blink_on_fraction = _non_neg_float(
            node.get("blink_on_fraction"), config.blink_on_fraction
        )

        # Colours: scoreboard.play_color (white default) and per-team colours,
        # falling back to the light column's team_colors so a single config edit
        # keeps both subsystems aligned.
        play = _parse_color(node.get("play_color"))
        if play is not None:
            config.play_color = play
        team_colors = dict(config.team_colors)
        fallback = tuning.get("light_column") if isinstance(tuning, dict) else None
        fallback_colors = (
            fallback.get("team_colors") if isinstance(fallback, dict) else None
        )
        for source in (fallback_colors, node.get("team_colors")):
            if isinstance(source, dict):
                for team, raw in source.items():
                    parsed = _parse_color(raw)
                    if parsed is not None:
                        team_colors[str(team).lower()] = parsed
        config.team_colors = team_colors
        return config



class ScoreboardController:
    """Own the desired per-panel text and drive the serial send queue.

    Args:
        transport: Open (or openable) serial transport for the panel string.
        layout: Bucket->panel routing and the list of panel indices.
        config: Text + timing tunables.
    """

    def __init__(
        self,
        transport: ScoreboardTransport,
        layout: ScoreboardLayout,
        config: ScoreboardConfig,
    ) -> None:
        self._transport = transport
        self._layout = layout
        self._config = config
        self._displays = layout.all_displays

        # Desired panel state for this tick (recomputed every update()).
        self._desired: dict[int, DisplayState] = {n: _BLANK for n in self._displays}
        # Panel state we have already queued commands for. ``None`` == unknown,
        # which forces a full (enable+mode+text) send the first time.
        self._queued: dict[int, DisplayState | None] = {n: None for n in self._displays}
        # Last monotonic time we enqueued a change for each panel (refresh gate).
        self._last_enqueue_mono: dict[int, float] = {n: -1e9 for n in self._displays}
        # FIFO of command-line bytes waiting to go out on the wire.
        self._send_queue: deque[bytes] = deque()
        # Last monotonic time a line was actually written (send pacing).
        self._last_send_mono: float = -1e9
        # Latest monotonic time handed to update(), reused by the winner-blink
        # phase so the on/off toggle is driven off the render clock.
        self._now_mono: float = -1e9

        # Latest game snapshot (set by the process loop via set_state).
        self._state: dict[str, Any] = {}

        # Stage -> render function dispatch table.
        self._renderers: dict[str, Callable[[dict[str, Any]], None]] = {
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

    def initialize(self) -> None:
        """Force every panel into a known startup baseline.

        The panels are NVS-backed, so a previous run can leave a board scrolling
        or dimmed. This pushes, for **every** mapped panel: max brightness
        (display + text layer), static mode, and a blanked text layer. It also
        primes the diff baseline (``_queued``) to ``_BLANK`` so the hardware and
        the controller's model agree - the first real content then only emits the
        fields that differ from blank/static.

        Call once from the process ``setup`` (after the transport opens); the
        queued lines go out on the first :meth:`pump`.
        """

        level = self._config.init_brightness
        for n in self._displays:
            self._send_queue.append(cmd_brightness(n, level))
            self._send_queue.append(cmd_text_brightness(n, level))
            self._send_queue.append(cmd_mode(n, MODE_STATIC))
            self._send_queue.append(cmd_enable(n, False))
            self._queued[n] = _BLANK

    # ---- content (memory only) -----------------------------------------

    def update(self, now_mono: float) -> None:
        """Recompute desired panel text for the current stage and queue diffs.

        ``now_mono`` is a process-local monotonic clock (``time.perf_counter``).
        """

        # Remember the render clock for the winner-blink toggle.
        self._now_mono = now_mono

        # Start from "all blank" each tick; renderers light up only the panels
        # that should show something for the current stage / active teams.
        for n in self._displays:
            self._desired[n] = _BLANK

        state = self._state
        # active_stage reflects the real stage even while the controller reports
        # a "paused" display stage, matching the light columns.
        stage = state.get("active_stage") or state.get("stage")
        if stage:
            renderer = self._renderers.get(stage)
            if renderer is not None:
                renderer(state)

        for n in self._displays:
            self._enqueue_if_changed(n, now_mono)

    # ---- transmission ---------------------------------------------------

    def pump(self, now_mono: float) -> None:
        """Drain queued command lines to the serial port.

        With ``inter_command_delay_s == 0`` the whole pending queue is flushed
        this tick. With a positive delay, at most one line is sent per delay
        window (spread across ticks) to throttle the RS485 link.
        """

        delay = self._config.inter_command_delay_s
        if delay <= 0.0:
            while self._send_queue:
                self._transport.write(self._send_queue.popleft())
            self._last_send_mono = now_mono
            return
        if self._send_queue and (now_mono - self._last_send_mono) >= delay:
            self._transport.write(self._send_queue.popleft())
            self._last_send_mono = now_mono

    # ---- per-stage renderers -------------------------------------------

    def _render_daydreaming(self, state: dict[str, Any]) -> None:
        """Attract mode: keep every panel blank (default), nothing to draw."""

        # All panels already default to _BLANK in update(); nothing to do.

    def _render_idle(self, state: dict[str, Any]) -> None:
        """Idle: every active bucket panel shows the idle placeholder (0000)."""

        ds = DisplayState(True, MODE_STATIC, self._config.idle_text, self._config.play_color)
        for team in self._active_teams(state):
            for index in range(3):
                self._set_bucket(team, index, ds)

    def _render_tutorial(self, state: dict[str, Any]) -> None:
        """Tutorial: bucket 1/2/3 of each active team show HOW / TO / PLAY.

        Tutorial text is drawn in each team's colour so the boards match the
        team light columns; all other stages use white (see ``play_color``).
        """

        words = self._config.tutorial_words
        for team in self._active_teams(state):
            color = self._team_color(team)
            for index in range(3):
                self._set_bucket(
                    team, index, DisplayState(True, MODE_STATIC, words[index], color)
                )


    def _render_play(self, state: dict[str, Any]) -> None:
        """Play: each panel shows its live bucket weight, 4-digit zero-padded."""

        self._render_bucket_weights(state)

    def _render_reset(self, state: dict[str, Any]) -> None:
        """Reset: every active panel scrolls GAME,OVER upward (mode 1)."""

        ds = DisplayState(
            True, MODE_SCROLL_UP, self._config.game_over_text, self._config.play_color
        )
        for team in self._active_teams(state):
            for index in range(3):
                self._set_bucket(team, index, ds)

    def _render_conclusion(self, state: dict[str, Any]) -> None:
        """Conclusion: count down bucket weights, then reveal WIN/LOSE/TIE.

        While counting, panels stay white (like play). On the reveal the winning
        team's panels blink ``WIN`` (text toggles on/off at ``blink_period_s``);
        the losing team shows a steady ``LOSE`` and an integer tie shows a steady
        ``TIE`` on every panel. All reveal text is white.
        """

        active = self._active_teams(state)
        if not self._conclusion_revealing(state, active):
            # Still counting: keep showing the (now decrementing) bucket weights.
            self._render_bucket_weights(state)
            return

        winner = self._winner_team(state, active)
        blink_on = self._blink_on(self._now_mono)
        color = self._config.play_color
        for team in active:
            if winner == "tie":
                text = self._config.tie_text
            elif team == winner:
                # Blink: blank the text on the "off" half of the cycle.
                text = self._config.win_text if blink_on else ""
            else:
                text = self._config.lose_text
            for index in range(3):
                self._set_bucket(team, index, DisplayState(True, MODE_STATIC, text, color))


    # ---- shared rendering helpers --------------------------------------

    def _render_bucket_weights(self, state: dict[str, Any]) -> None:
        """Fill each active panel with its bucket weight as zero-padded digits."""

        color = self._config.play_color
        for team in self._active_teams(state):
            buckets = _team_block(state, team).get("buckets")
            buckets = buckets if isinstance(buckets, list) else []
            for index in range(3):
                value = buckets[index] if index < len(buckets) else 0
                text = self._format_weight(value)
                self._set_bucket(team, index, DisplayState(True, MODE_STATIC, text, color))

    def _team_color(self, team: str) -> tuple[int, int, int]:
        """Return the configured tutorial colour for ``team`` (white fallback)."""

        return self._config.team_colors.get(str(team).lower(), _WHITE)

    def _blink_on(self, now_mono: float) -> bool:
        """Return True during the "on" half of the winner-blink cycle.

        ``blink_period_s`` is one full on+off cycle; ``blink_on_fraction`` is the
        on duty cycle. A non-positive period disables blinking (always on).
        """

        period = self._config.blink_period_s
        if period <= 0.0:
            return True
        phase = now_mono % period
        return phase < (period * self._config.blink_on_fraction)


    def _set_bucket(self, team: str, bucket_index: int, ds: DisplayState) -> None:
        """Set the desired state for one team bucket's panel (if it is mapped)."""

        display = self._layout.display_for(team, bucket_index)
        if display is not None:
            self._desired[display] = ds

    def _format_weight(self, value: Any) -> str:
        """Clamp ``value`` to the configured digit width and zero-pad it."""

        number = _maybe_int(value) or 0
        max_value = 10 ** self._config.score_digits - 1
        number = max(0, min(number, max_value))
        return f"{number:0{self._config.score_digits}d}"

    def _active_teams(self, state: dict[str, Any]) -> list[str]:
        """Return the teams that have a published block in ``state.full``."""

        teams = state.get("teams")
        if not isinstance(teams, dict):
            return []
        return [team for team in _TEAMS if isinstance(teams.get(team), dict)]

    def _conclusion_revealing(self, state: dict[str, Any], active: list[str]) -> bool:
        """Return True once any active team has reached the win/lose reveal."""

        for team in active:
            conclusion = _team_block(state, team).get("conclusion")
            conclusion = conclusion if isinstance(conclusion, dict) else {}
            if conclusion.get("phase") in _REVEAL_PHASES or conclusion.get("done"):
                return True
        return False

    def _winner_team(self, state: dict[str, Any], active: list[str]) -> str:
        """Return ``"a"``/``"b"`` for the higher total, or ``"tie"`` if equal.

        Totals come from the published per-team ``summed_score`` (which holds the
        whole team score once the count-down has finished). Mirrors the game
        controller's own ``_winner_team`` tie rule (integer-level equality).
        """

        totals = [
            (team, _maybe_int(_team_block(state, team).get("summed_score")) or 0)
            for team in active
        ]
        if not totals:
            return "tie"
        totals.sort(key=lambda item: item[1], reverse=True)
        if len(totals) >= 2 and totals[0][1] == totals[1][1]:
            return "tie"
        return totals[0][0]

    # ---- diff + enqueue -------------------------------------------------

    def _enqueue_if_changed(self, display: int, now_mono: float) -> None:
        """Queue command lines for ``display`` if its desired state changed.

        Respects ``min_refresh_interval_s`` so a fast-changing number does not
        flood the bus: if the panel changed too recently, the new value waits
        for the next eligible tick (always coalescing to the latest desired).
        """

        desired = self._desired[display]
        prev = self._queued[display]
        if prev is not None and desired == prev:
            return
        if prev is not None and (
            now_mono - self._last_enqueue_mono[display]
        ) < self._config.min_refresh_interval_s:
            return

        lines = self._diff_lines(display, prev, desired)
        self._queued[display] = desired
        if not lines:
            return
        self._send_queue.extend(lines)
        self._last_enqueue_mono[display] = now_mono

    def _diff_lines(
        self, display: int, prev: DisplayState | None, desired: DisplayState
    ) -> list[bytes]:
        """Return the minimal command lines to move ``prev`` -> ``desired``.

        Only the fields that actually changed are emitted. The first send for a
        panel (``prev is None``) emits the full enable+mode+colour+text sequence.
        Colour is only emitted when set (``color is not None``) and different from
        the previous colour.
        """

        if not desired.enable:
            # Blanking: only send the disable line if it was on (or unknown).
            if prev is None or prev.enable:
                return [cmd_enable(display, False)]
            return []

        lines: list[bytes] = []
        if prev is None or not prev.enable:
            lines.append(cmd_enable(display, True))
        if prev is None or prev.mode != desired.mode:
            lines.append(cmd_mode(display, desired.mode))
        if desired.color is not None and (prev is None or prev.color != desired.color):
            lines.append(cmd_color(display, *desired.color))
        if prev is None or prev.text != desired.text:
            lines.append(cmd_text(display, desired.text))
        return lines


    # ---- test/inspection accessors -------------------------------------

    def desired_state(self, display: int) -> DisplayState:
        """Return the current desired state for one panel (for tests/inspection)."""

        return self._desired.get(display, _BLANK)


# ---- module-level state extraction + coercion helpers -------------------


def _team_block(state: dict[str, Any], team: str) -> dict[str, Any]:
    """Return ``state.full.teams.<team>`` as a dict (empty if absent)."""

    teams = state.get("teams")
    if not isinstance(teams, dict):
        return {}
    block = teams.get(team)
    return block if isinstance(block, dict) else {}


def _maybe_int(value: Any) -> int | None:
    """Coerce to a rounded int, returning None on failure."""

    try:
        return int(round(float(value)))
    except (TypeError, ValueError):
        return None


def _non_neg_float(value: Any, default: float) -> float:
    """Coerce to a non-negative float, else return ``default``."""

    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed >= 0.0 else default


def _parse_color(value: Any) -> tuple[int, int, int] | None:
    """Parse a colour as ``"#RRGGBB"`` or ``[r, g, b]`` into a 0..255 tuple.

    Returns ``None`` when the value is missing or malformed so callers can keep
    their existing default. Channels are clamped to 0..255.
    """

    if isinstance(value, str):
        text = value.strip().lstrip("#")
        if len(text) == 6:
            try:
                r = int(text[0:2], 16)
                g = int(text[2:4], 16)
                b = int(text[4:6], 16)
            except ValueError:
                return None
            return (r, g, b)
        return None
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        out: list[int] = []
        for channel in value[:3]:
            try:
                out.append(max(0, min(255, int(round(float(channel))))))
            except (TypeError, ValueError):
                return None
        return (out[0], out[1], out[2])
    return None

