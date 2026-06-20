"""display_viewer - local mock of a Raspberry Pi player display.

Receives the UDP game-state broadcast emitted by ``apps.state_broadcaster`` and
renders one or two player panels for the current game stage. This is the same
client logic intended to run on each Raspberry Pi: on a real Pi it learns its
identity from the machine hostname (mapped to two player ids in
``config/device_ports_and_addr.yaml`` -> ``display_broadcast.hosts``); on a dev
machine you select the panel(s) explicitly with --player / --host.

A player id is the team letter (a/b) + 1-based player number; player N renders
robot joint index N-1 and haptic dial N-1 for that team.

Per-stage screens
-----------------
* idle         : "Move the dial to begin" prompt.
* daydreaming  : live gauge of this player's joint + dial position.
* tutorial     : long scrollable page driven by this player's progress (0-100%).
* play         : robot joint + dial + proximity collision + speed override + timer.
* reset        : "Game over - robot returning to start" + rewind progress.
* conclusion   : bucket countdown into the team total, team-vs-opponent, thanks.

Run (local testing; the broadcaster must also be running):
    $env:PYTHONPATH = "src"
    # Single player panel:
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.display_viewer \
        --player a1 --bind 127.0.0.1 --port 49200
    # Emulate a Pi (its two screens side by side):
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.display_viewer \
        --host rpi5-11 --bind 127.0.0.1 --port 49200
    # On a real Pi (identity from hostname, listen on all interfaces):
    & python -m apps.display_viewer

Notes
-----
* --bind defaults to 0.0.0.0 (all interfaces) so it works on a Pi unchanged;
  use 127.0.0.1 when testing a localhost broadcaster (--dest 127.0.0.1).
* Press ESC or close the window to quit.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import pygame  # noqa: E402

from core.device_connection import load_display_broadcast, resolve_display_players  # noqa: E402
from core.display_protocol import decode_datagram  # noqa: E402

# ----------------------------------------------------------------------------
# Tunables (visual only; safe to adjust for different monitors).
# ----------------------------------------------------------------------------
DEFAULT_WINDOW_W = 1280  # window width in pixels (per run; not per Pi screen)
DEFAULT_WINDOW_H = 720  # window height in pixels
TARGET_FPS = 60  # render loop cap (Hz); independent of the network rate
SIGNAL_TIMEOUT_S = 1.0  # show "SIGNAL LOST" if no datagram within this window
RX_BUFFER_BYTES = 1 << 16  # UDP recv buffer per call (>= max datagram size)
TUTORIAL_PAGES = 8  # number of placeholder tutorial "pages" to scroll through
TUTORIAL_FALLBACK_PLAYERS = ("a1", "a2")  # used when hostname is not in the map

# Team accent colors. Team A = blue, Team B = red (matches the dashboard/LEDs).
TEAM_COLORS = {
    "a": (60, 140, 255),
    "b": (235, 70, 70),
}
BG = (16, 18, 24)  # panel background
FG = (235, 238, 245)  # primary text
MUTED = (120, 128, 140)  # secondary text
PANEL_GAP = 8  # pixels between side-by-side panels


# ----------------------------------------------------------------------------
# Receiver
# ----------------------------------------------------------------------------
class UdpStateReceiver:
    """Non-blocking UDP socket that keeps only the most recent game state.

    The socket binds ``(bind_addr, port)`` and is drained to the latest valid
    datagram each frame. Older / out-of-order packets are discarded so the
    display always reflects the freshest snapshot. ``state``/``ts_wall_ns`` hold
    the last good payload; ``last_rx_mono`` powers the staleness indicator.
    """

    def __init__(self, bind_addr: str, port: int) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_addr, port))
        self._sock.setblocking(False)
        self.state: dict | None = None  # latest state.full body, or None
        self.seq: int = -1  # last accepted datagram seq (reorder guard)
        self.ts_wall_ns: int = 0  # broadcaster wall clock of the latest frame
        self.last_rx_mono: float | None = None  # perf_counter of last accept

    def poll(self) -> None:
        """Drain the socket, accepting only the newest in-order datagram."""

        newest: dict | None = None
        while True:
            try:
                raw, _addr = self._sock.recvfrom(RX_BUFFER_BYTES)
            except BlockingIOError:
                break
            except OSError:
                break
            message = decode_datagram(raw)
            if message is None:
                continue
            seq = int(message.get("seq", 0))
            # Keep the highest seq seen this poll; also reject stale seq vs the
            # last accepted one to ignore reordered fragments across polls.
            if newest is None or seq >= int(newest.get("seq", 0)):
                newest = message
        if newest is not None and int(newest.get("seq", 0)) >= self.seq:
            self.state = newest.get("state")
            self.seq = int(newest.get("seq", 0))
            self.ts_wall_ns = int(newest.get("ts_wall_ns", 0))
            self.last_rx_mono = time.perf_counter()

    def is_stale(self) -> bool:
        """Return True if no fresh datagram arrived within SIGNAL_TIMEOUT_S."""

        if self.last_rx_mono is None:
            return True
        return (time.perf_counter() - self.last_rx_mono) > SIGNAL_TIMEOUT_S

    def close(self) -> None:
        """Close the underlying UDP socket."""

        self._sock.close()


# ----------------------------------------------------------------------------
# Small drawing helpers
# ----------------------------------------------------------------------------
def _player_team_and_joint(player: str) -> tuple[str, int]:
    """Split a player id like ``"a3"`` into (team_letter, joint_index 0-5)."""

    team = player[:1].lower()
    try:
        number = int(player[1:])
    except ValueError:
        number = 1
    joint = max(0, min(5, number - 1))
    return team, joint


def _fonts(panel_h: int) -> dict[str, "pygame.font.Font"]:
    """Build a small set of fonts scaled to the panel height."""

    base = max(12, panel_h // 22)
    return {
        "big": pygame.font.SysFont("Arial", base * 3, bold=True),
        "title": pygame.font.SysFont("Arial", base * 2, bold=True),
        "body": pygame.font.SysFont("Arial", base),
        "small": pygame.font.SysFont("Arial", max(10, base * 3 // 4)),
    }


def _text(
    surface: "pygame.Surface",
    font: "pygame.font.Font",
    text: str,
    color: tuple[int, int, int],
    center: tuple[int, int],
) -> None:
    """Blit center-anchored text."""

    img = font.render(text, True, color)
    surface.blit(img, img.get_rect(center=center))


def _bar(
    surface: "pygame.Surface",
    rect: "pygame.Rect",
    frac: float,
    color: tuple[int, int, int],
) -> None:
    """Draw a left-to-right fill bar with ``frac`` in [0, 1]."""

    frac = max(0.0, min(1.0, frac))
    pygame.draw.rect(surface, (40, 44, 54), rect, border_radius=4)
    inner = rect.copy()
    inner.width = int(rect.width * frac)
    if inner.width > 0:
        pygame.draw.rect(surface, color, inner, border_radius=4)
    pygame.draw.rect(surface, (70, 76, 90), rect, width=2, border_radius=4)


def _angle_gauge(
    surface: "pygame.Surface",
    center: tuple[int, int],
    radius: int,
    angle_rad: float,
    color: tuple[int, int, int],
    *,
    span_rad: float = 6.2831853,
) -> None:
    """Draw a circular gauge with a needle at ``angle_rad`` (wrapped to span)."""

    import math

    pygame.draw.circle(surface, (40, 44, 54), center, radius)
    pygame.draw.circle(surface, (70, 76, 90), center, radius, width=3)
    # Map angle onto the gauge: 0 rad points up, increasing clockwise.
    theta = (angle_rad % span_rad) / span_rad * 2.0 * math.pi
    tip = (
        center[0] + int(radius * 0.85 * math.sin(theta)),
        center[1] - int(radius * 0.85 * math.cos(theta)),
    )
    pygame.draw.line(surface, color, center, tip, 5)
    pygame.draw.circle(surface, color, center, max(4, radius // 12))


# ----------------------------------------------------------------------------
# Per-stage panel renderers. Each draws into ``rect`` for one player.
# ----------------------------------------------------------------------------
def _team_block(state: dict, team: str) -> dict:
    """Return the per-team block from state.full, or an empty dict."""

    teams = state.get("teams") if isinstance(state, dict) else None
    if isinstance(teams, dict) and isinstance(teams.get(team), dict):
        return teams[team]
    return {}


def _render_idle(surface, rect, fonts, accent, player) -> None:
    """Idle screen: invite the player to move the dial."""

    _text(surface, fonts["title"], f"Player {player.upper()}", accent,
          (rect.centerx, rect.top + rect.height // 4))
    _text(surface, fonts["big"], "Move the dial", FG,
          (rect.centerx, rect.centery))
    _text(surface, fonts["body"], "to begin the game", MUTED,
          (rect.centerx, rect.centery + rect.height // 6))


def _render_daydreaming(surface, rect, fonts, accent, player, team_block) -> None:
    """Daydreaming screen: live gauge of this player's joint + dial."""

    import math

    _, joint = _player_team_and_joint(player)
    q_rad = (team_block.get("robot") or {}).get("q_rad") or [0.0] * 6
    dial_rad = (team_block.get("haptic") or {}).get("dial_pos_rad") or [0.0] * 6
    q = float(q_rad[joint]) if joint < len(q_rad) else 0.0
    dial = float(dial_rad[joint]) if joint < len(dial_rad) else 0.0

    _text(surface, fonts["title"], f"Axis {joint + 1}", accent,
          (rect.centerx, rect.top + rect.height // 8))
    radius = min(rect.width, rect.height) // 4
    _angle_gauge(surface, (rect.centerx, rect.centery), radius, q, accent)
    _text(surface, fonts["body"], f"robot {math.degrees(q):+.0f} deg", FG,
          (rect.centerx, rect.bottom - rect.height // 4))
    _text(surface, fonts["small"], f"dial {math.degrees(dial):+.0f} deg", MUTED,
          (rect.centerx, rect.bottom - rect.height // 7))


def _render_tutorial(surface, rect, fonts, accent, player, team_block) -> None:
    """Tutorial screen: a long page scrolled by this player's progress (0-100%)."""

    _, joint = _player_team_and_joint(player)
    progress = (team_block.get("haptic") or {}).get("tutorial_progress_pct") or [0.0] * 6
    pct = float(progress[joint]) if joint < len(progress) else 0.0
    frac = max(0.0, min(1.0, pct / 100.0))

    # Virtual page taller than the view; scroll its content by progress.
    view = rect.inflate(-rect.width // 12, -rect.height // 8)
    content_h = view.height * 3  # the "long" page is 3x the visible height
    scroll = int(frac * (content_h - view.height))
    prev_clip = surface.get_clip()
    surface.set_clip(view)
    section_h = content_h // TUTORIAL_PAGES
    for i in range(TUTORIAL_PAGES):
        y = view.top + i * section_h - scroll
        if y > view.bottom or y + section_h < view.top:
            continue
        block = pygame.Rect(view.left, y + 6, view.width, section_h - 12)
        pygame.draw.rect(surface, (30, 34, 44), block, border_radius=6)
        _text(surface, fonts["title"], f"Tutorial Page {i + 1}/{TUTORIAL_PAGES}",
              accent, (block.centerx, block.top + section_h // 3))
        _text(surface, fonts["small"], "Scroll the dial to continue", MUTED,
              (block.centerx, block.top + section_h // 2))
    surface.set_clip(prev_clip)

    # Scroll thumb on the right edge + a progress readout.
    track = pygame.Rect(rect.right - 18, view.top, 8, view.height)
    pygame.draw.rect(surface, (40, 44, 54), track, border_radius=4)
    thumb_h = max(20, int(view.height / TUTORIAL_PAGES))
    thumb_y = view.top + int(frac * (view.height - thumb_h))
    pygame.draw.rect(surface, accent,
                     pygame.Rect(track.left, thumb_y, track.width, thumb_h),
                     border_radius=4)
    _text(surface, fonts["small"], f"{pct:.0f}%", FG,
          (rect.centerx, rect.bottom - rect.height // 14))


def _render_play(surface, rect, fonts, accent, player, team_block, state) -> None:
    """Play screen: robot + dial + proximity collision + override + timer."""

    import math

    _, joint = _player_team_and_joint(player)
    robot = team_block.get("robot") or {}
    haptic = team_block.get("haptic") or {}
    collision = team_block.get("collision") or {}
    q_rad = robot.get("q_rad") or [0.0] * 6
    dial_rad = haptic.get("dial_pos_rad") or [0.0] * 6
    q = float(q_rad[joint]) if joint < len(q_rad) else 0.0
    dial = float(dial_rad[joint]) if joint < len(dial_rad) else 0.0

    in_collision = bool(collision.get("in_collision"))
    final_scalar = float(collision.get("final_scalar") or 1.0)  # 1 = full speed
    prox_scalar = float(collision.get("prox_scalar") or 1.0)
    countdown = state.get("countdown_s")

    # Timer (top).
    timer_text = f"{float(countdown):.0f}s" if countdown is not None else "--"
    _text(surface, fonts["title"], timer_text, FG,
          (rect.centerx, rect.top + rect.height // 12))
    _text(surface, fonts["small"], f"Axis {joint + 1}", accent,
          (rect.left + rect.width // 6, rect.top + rect.height // 12))

    # Robot + dial readouts.
    pad = rect.width // 10
    _text(surface, fonts["body"], f"robot {math.degrees(q):+.0f} deg", FG,
          (rect.centerx, rect.top + rect.height // 4))
    _text(surface, fonts["body"], f"dial  {math.degrees(dial):+.0f} deg", MUTED,
          (rect.centerx, rect.top + rect.height // 4 + rect.height // 12))

    # Speed override bar (proximity slowdown). Lower = more slowdown.
    bar_w = rect.width - 2 * pad
    ov_rect = pygame.Rect(rect.left + pad, rect.centery + rect.height // 12, bar_w,
                          max(14, rect.height // 22))
    ov_color = (90, 200, 120) if final_scalar > 0.66 else (
        (230, 190, 60) if final_scalar > 0.33 else (235, 90, 70))
    _bar(surface, ov_rect, final_scalar, ov_color)
    _text(surface, fonts["small"], f"speed override {final_scalar * 100:.0f}%",
          FG, (rect.centerx, ov_rect.top - rect.height // 22))

    # Proximity bar.
    px_rect = pygame.Rect(rect.left + pad, ov_rect.bottom + rect.height // 14,
                          bar_w, max(10, rect.height // 30))
    _bar(surface, px_rect, prox_scalar, accent)
    _text(surface, fonts["small"], "proximity", MUTED,
          (rect.left + pad + 40, px_rect.top - rect.height // 30))

    # Imminent-collision banner.
    if in_collision:
        warn = pygame.Rect(rect.left + pad, rect.bottom - rect.height // 6,
                           bar_w, rect.height // 10)
        pygame.draw.rect(surface, (235, 70, 70), warn, border_radius=6)
        _text(surface, fonts["title"], "COLLISION", (16, 16, 20), warn.center)


def _render_reset(surface, rect, fonts, accent, player, team_block) -> None:
    """Reset/rewind screen: game over, robot returning to start."""

    rewind = team_block.get("rewind") or {}
    progress = float(rewind.get("progress") or 0.0)  # 0..1
    _text(surface, fonts["title"], "Game over", accent,
          (rect.centerx, rect.top + rect.height // 4))
    _text(surface, fonts["body"], "Robot returning to start", FG,
          (rect.centerx, rect.centery - rect.height // 12))
    _text(surface, fonts["small"], "to begin score counting", MUTED,
          (rect.centerx, rect.centery + rect.height // 20))
    bar = pygame.Rect(rect.left + rect.width // 8,
                      rect.bottom - rect.height // 4,
                      rect.width * 3 // 4, max(14, rect.height // 22))
    _bar(surface, bar, progress, accent)


def _render_conclusion(surface, rect, fonts, accent, player, team_block, state) -> None:
    """Conclusion screen: bucket countdown -> team total -> vs opponent -> thanks."""

    team, _joint = _player_team_and_joint(player)
    conclusion = team_block.get("conclusion") or {}
    done = bool(conclusion.get("done"))
    buckets = team_block.get("buckets") or []
    summed = team_block.get("summed_score")
    labels = team_block.get("bucket_labels") or [f"B{i + 1}" for i in range(len(buckets))]

    if done:
        # Final card: this team vs the opponent, then thanks.
        opponent = "b" if team == "a" else "a"
        opp_block = _team_block(state, opponent)
        opp_summed = opp_block.get("summed_score")
        _text(surface, fonts["title"], "Final score", accent,
              (rect.centerx, rect.top + rect.height // 6))
        _text(surface, fonts["big"],
              f"{_num(summed)}  vs  {_num(opp_summed)}", FG,
              (rect.centerx, rect.centery - rect.height // 12))
        _text(surface, fonts["body"], "Thanks for playing!", MUTED,
              (rect.centerx, rect.bottom - rect.height // 5))
        return

    # Live count: three buckets decrementing into the rising team total.
    _text(surface, fonts["title"], "Score counting", accent,
          (rect.centerx, rect.top + rect.height // 8))
    active = conclusion.get("active_bucket_index")
    col_w = rect.width // max(1, len(buckets) or 1)
    for i, value in enumerate(buckets):
        cx = rect.left + col_w * i + col_w // 2
        color = accent if i == active else MUTED
        _text(surface, fonts["body"], str(labels[i]) if i < len(labels) else f"B{i+1}",
              MUTED, (cx, rect.centery - rect.height // 8))
        _text(surface, fonts["title"], _num(value), color,
              (cx, rect.centery))
    _text(surface, fonts["body"], f"Total {_num(summed)}", FG,
          (rect.centerx, rect.bottom - rect.height // 5))


def _num(value) -> str:
    """Format a numeric score value compactly, tolerating None."""

    if value is None:
        return "0"
    try:
        return f"{float(value):.0f}"
    except (TypeError, ValueError):
        return str(value)


# ----------------------------------------------------------------------------
# Panel dispatch
# ----------------------------------------------------------------------------
def _render_panel(surface, rect, fonts, player, state) -> None:
    """Render one player's panel for the current stage."""

    team, _joint = _player_team_and_joint(player)
    accent = TEAM_COLORS.get(team, FG)
    pygame.draw.rect(surface, BG, rect)
    pygame.draw.rect(surface, accent, rect, width=3)

    if not isinstance(state, dict):
        _text(surface, fonts["body"], "Waiting for broadcast...", MUTED, rect.center)
        return

    stage = str(state.get("active_stage") or state.get("stage") or "")
    team_block = _team_block(state, team)

    # Stage header strip.
    _text(surface, fonts["small"], f"{player.upper()}  |  {stage or '...'}",
          accent, (rect.centerx, rect.top + max(12, rect.height // 30)))

    if stage == "idle":
        _render_idle(surface, rect, fonts, accent, player)
    elif stage == "daydreaming":
        _render_daydreaming(surface, rect, fonts, accent, player, team_block)
    elif stage == "tutorial":
        _render_tutorial(surface, rect, fonts, accent, player, team_block)
    elif stage == "play":
        _render_play(surface, rect, fonts, accent, player, team_block, state)
    elif stage == "reset":
        _render_reset(surface, rect, fonts, accent, player, team_block)
    elif stage == "conclusion":
        _render_conclusion(surface, rect, fonts, accent, player, team_block, state)
    else:
        _text(surface, fonts["body"], f"stage: {stage or 'unknown'}", MUTED,
              rect.center)

    if bool(state.get("paused")):
        _text(surface, fonts["title"], "PAUSED", (235, 190, 60),
              (rect.centerx, rect.bottom - max(20, rect.height // 16)))


# ----------------------------------------------------------------------------
# Identity + CLI
# ----------------------------------------------------------------------------
def _resolve_panels(ns: argparse.Namespace) -> list[str]:
    """Decide which player panel(s) to render from CLI args / hostname.

    Priority: explicit --player, then --host map lookup, then this machine's
    hostname, then a safe fallback so the window always shows something.
    """

    if ns.player:
        return [str(ns.player).lower()]
    if ns.host:
        players = resolve_display_players(ns.host)
        if players:
            return list(players)
        print(f"[display_viewer] host {ns.host!r} not in display_broadcast.hosts; "
              f"falling back to {TUTORIAL_FALLBACK_PLAYERS}", flush=True)
        return list(TUTORIAL_FALLBACK_PLAYERS)
    hostname = socket.gethostname()
    players = resolve_display_players(hostname)
    if players:
        return list(players)
    print(f"[display_viewer] hostname {hostname!r} not in display_broadcast.hosts; "
          f"falling back to {TUTORIAL_FALLBACK_PLAYERS} (use --player/--host to override)",
          flush=True)
    return list(TUTORIAL_FALLBACK_PLAYERS)


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse the display-viewer CLI."""

    ap = argparse.ArgumentParser(description="Local Raspberry Pi display mock.")
    ap.add_argument("--player", default=None,
                    help="Render a single player panel, e.g. a1 (overrides --host).")
    ap.add_argument("--host", default=None,
                    help="Emulate a Pi hostname; renders its two player panels.")
    ap.add_argument("--bind", default="0.0.0.0",
                    help="Local interface to bind the UDP receiver (default 0.0.0.0).")
    ap.add_argument("--port", type=int, default=None,
                    help="UDP port to listen on (default from device_ports_and_addr.yaml).")
    ap.add_argument("--width", type=int, default=DEFAULT_WINDOW_W,
                    help="Window width in pixels.")
    ap.add_argument("--height", type=int, default=DEFAULT_WINDOW_H,
                    help="Window height in pixels.")
    ap.add_argument("--fullscreen", action="store_true",
                    help="Open the window fullscreen.")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run the UDP receiver + pygame render loop until the window closes."""

    ns = _parse_args(argv)
    port = ns.port if ns.port is not None else load_display_broadcast().port
    panels = _resolve_panels(ns)

    receiver = UdpStateReceiver(ns.bind, port)

    pygame.init()
    flags = pygame.FULLSCREEN if ns.fullscreen else 0
    screen = pygame.display.set_mode((ns.width, ns.height), flags)
    pygame.display.set_caption(f"display_viewer {panels} :{port}")
    clock = pygame.time.Clock()
    fonts = _fonts(screen.get_height())

    print(f"[display_viewer] listening on {ns.bind}:{port}; panels={panels}", flush=True)

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            receiver.poll()
            screen.fill((8, 9, 12))

            # Split the window into one column per panel.
            w = screen.get_width()
            h = screen.get_height()
            n = len(panels)
            panel_w = (w - PANEL_GAP * (n - 1)) // n
            for i, player in enumerate(panels):
                rect = pygame.Rect(i * (panel_w + PANEL_GAP), 0, panel_w, h)
                _render_panel(screen, rect, fonts, player, receiver.state)

            if receiver.is_stale():
                _text(screen, fonts["small"], "SIGNAL LOST", (235, 90, 70),
                      (w // 2, h - 14))

            pygame.display.flip()
            clock.tick(TARGET_FPS)
    finally:
        receiver.close()
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
