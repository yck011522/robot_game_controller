"""Standalone pygame observer dashboard mockup.

Renders every layout on a logical 3840x2160 canvas, then scales that
canvas down to fit the actual monitor. This lets us design against a
4k target even while developing on a smaller display.

Controls
--------
P   : play / resume
Space: soft e-stop
E   : end game
F   : toggle fullscreen fit-to-monitor
Esc : quit
"""

from __future__ import annotations

import argparse
from collections import deque
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pygame
import zmq


_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from core import bus
from core.config import ConfigError, default_runtime_setting, load as load_profile


REPO_ROOT = Path(__file__).resolve().parents[3]
LOGICAL_SIZE = (3840, 2160)
TEAM_A = "a"
TEAM_B = "b"
HEARTBEAT_HZ = 1.0
UI_COMMAND_ACK_TIMEOUT_S = 0.25
UI_COMMAND_ACK_BANNER_S = 1.5
UI_COMMAND_ERROR_BANNER_S = 4.0
DEFAULT_Q_MIN_RAD = [-math.pi] * 6
DEFAULT_Q_MAX_RAD = [math.pi] * 6
MAX_COLLISION_WORKER_SLOTS = 16
DEFAULT_HEARTBEAT_AGE_MAX_MS = 1100.0

COLORS = {
    "bg": (11, 16, 24),
    "bg_2": (20, 33, 46),
    "panel": (20, 28, 39),
    "panel_alt": (26, 37, 51),
    "panel_soft": (33, 46, 63),
    "outline": (69, 93, 118),
    "grid": (52, 68, 88),
    "text": (233, 239, 245),
    "muted": (139, 151, 165),
    "grey": (104, 114, 126),
    "warning": (235, 149, 53),
    "danger": (211, 84, 64),
    "success": (90, 197, 130),
    "blue": (45, 108, 223),
    "blue_soft": (96, 151, 255),
    "red": (217, 76, 58),
    "red_soft": (255, 142, 124),
    "cyan": (89, 206, 214),
    "black": (0, 0, 0),
}

LAYOUT_NAME = "Central Spine Broadcast Tech"

CONTROL_ACTION_LABELS = {
    "play_resume": "PLAY/RESUME",
    "end_game": "END GAME",
    "soft_estop": "E-STOP",
}

LAYOUT_STYLE = {
    "name": "Broadcast Tech",
    "panel_radius": 0,
    "lane_radius": 2,
    "outline_width": 3,
    "corner_brackets": True,
    "heavy_dividers": True,
    "worker_tile_radius": 0,
    "spine_inset": 22,
    "match_widget_inset": 40,
}


@dataclass
class TeamMock:
    name: str
    color: tuple[int, int, int]
    color_soft: tuple[int, int, int]
    active: bool
    score: float
    buckets: list[float]
    summed_score: int
    actual: list[float]
    target: list[float]
    actual_deg: list[float]
    connected: list[bool]
    loop_hz: list[float]
    in_collision: bool
    path_scalar: float
    final_scalar: float
    first_hit_deg: float
    prox_probe_offsets_deg: list[float]
    prox_hits: list[list[bool]]
    prox_age_ticks: list[int]


@dataclass
class ProcessMock:
    name: str
    proc_key: str
    hz: float
    age_ms: float
    checks_per_sec: float | None = None
    active: bool = True


class FontBook:
    def __init__(self) -> None:
        base = REPO_ROOT / "interface_design" / "Font_Roboto" / "static"
        self._regular_path = str(base / "Roboto-Regular.ttf")
        self._medium_path = str(base / "Roboto-Medium.ttf")
        self._bold_path = str(base / "Roboto-Bold.ttf")
        self._cache: dict[tuple[str, int], pygame.font.Font] = {}

    def regular(self, size: int) -> pygame.font.Font:
        return self._font(self._regular_path, size)

    def medium(self, size: int) -> pygame.font.Font:
        return self._font(self._medium_path, size)

    def bold(self, size: int) -> pygame.font.Font:
        return self._font(self._bold_path, size)

    def _font(self, path: str, size: int) -> pygame.font.Font:
        key = (path, size)
        font = self._cache.get(key)
        if font is None:
            font = pygame.font.Font(path, size)
            self._cache[key] = font
        return font


class DashboardMockup:
    def __init__(self, native_4k: bool, fit: float, profile_path: str | None = None) -> None:
        pygame.init()
        pygame.font.init()
        self._native_4k = native_4k
        self._fit = max(0.3, min(1.0, fit))
        self._fonts = FontBook()
        self._clock = pygame.time.Clock()
        self._fullscreen = False
        self._screenshot_index = 0
        self._screen = self._create_screen()
        self._logical = pygame.Surface(LOGICAL_SIZE, pygame.SRCALPHA).convert_alpha()
        self._background = self._build_background_surface()
        self._scaled_surface: pygame.Surface | None = None
        self._scaled_size: tuple[int, int] | None = None
        self._ctx = zmq.Context.instance()
        self._state_sub = bus.make_sub(self._ctx, topics=["state.full"])
        self._heartbeat_sub = bus.make_sub(self._ctx, topics=["heartbeat."])
        self._heartbeat_pub = bus.make_pub(self._ctx)
        self._ui_req: zmq.Socket | None = None
        self._ui_req_poller = zmq.Poller()
        self._reset_ui_req_socket()
        self._profile = _load_profile_if_available(profile_path)
        self._target_fps = _profile_fps_target(self._profile, "gamemaster_ui")
        self._q_min_rad, self._q_max_rad = _profile_joint_limits_rad(self._profile)
        self._collision_worker_count = _profile_collision_worker_count(self._profile)
        self._latest_state_full: dict[str, Any] | None = None
        self._latest_state_recv_mono_ns: int | None = None
        self._heartbeat_body_by_proc: dict[str, dict[str, Any]] = {}
        self._heartbeat_recv_mono_ns: dict[str, int] = {}
        self._heartbeat_window_by_proc: dict[str, deque[int]] = {}
        self._next_control_request_id = 1
        self._pending_control_request: dict[str, Any] | None = None
        self._last_control_ack: dict[str, Any] | None = None
        self._last_control_error: dict[str, Any] | None = None
        self._next_heartbeat_t = time.perf_counter() + (1.0 / HEARTBEAT_HZ)
        pygame.display.set_caption("Observer dashboard | P play/resume | Space soft e-stop | E end game")

    def run(self) -> int:
        try:
            while True:
                self._clock.tick(self._target_fps)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return 0
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            return 0
                        if event.key == pygame.K_f:
                            self._fullscreen = not self._fullscreen
                            self._screen = self._create_screen()
                        elif event.key == pygame.K_p:
                            self._request_control_action("play_resume", source="keyboard")
                        elif event.key == pygame.K_SPACE:
                            self._request_control_action("soft_estop", source="keyboard")
                        elif event.key == pygame.K_e:
                            self._request_control_action("end_game", source="keyboard")
                    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                        logical_pos = self._screen_to_logical(event.pos)
                        if logical_pos is not None:
                            action = self._control_action_at(logical_pos, self._render_state())
                            if action is not None:
                                self._request_control_action(action, source="mouse")
                    if event.type == pygame.VIDEORESIZE and not self._fullscreen and not self._native_4k:
                        self._screen = pygame.display.set_mode(event.size, pygame.RESIZABLE)

                self._poll_bus()
                self._poll_control_channel()
                self._publish_heartbeat_if_due()
                state = self._render_state()
                self._draw(state)
                self._present()
        finally:
            self.close()

    def close(self) -> None:
        self._state_sub.close(0)
        self._heartbeat_sub.close(0)
        self._heartbeat_pub.close(0)
        if self._ui_req is not None:
            self._ui_req.close(0)

    def _reset_ui_req_socket(self) -> None:
        # REQ sockets become unusable after a timeout until the missing
        # reply arrives, so the dashboard rebuilds the socket when a
        # command times out instead of stalling future operator input.
        self._ui_req_poller = zmq.Poller()
        if self._ui_req is not None:
            self._ui_req.close(0)
        self._ui_req = bus.make_req(self._ctx)
        self._ui_req_poller.register(self._ui_req, zmq.POLLIN)

    def _create_screen(self) -> pygame.Surface:
        if self._native_4k:
            return pygame.display.set_mode(LOGICAL_SIZE)

        info = pygame.display.Info()
        if self._fullscreen:
            return pygame.display.set_mode((info.current_w, info.current_h), pygame.FULLSCREEN)

        max_w = max(1280, int(info.current_w * self._fit))
        max_h = max(720, int(info.current_h * self._fit))
        win_w, win_h = _fit_size(LOGICAL_SIZE[0], LOGICAL_SIZE[1], max_w, max_h)
        return pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)

    def _present(self) -> None:
        if self._native_4k:
            self._screen.blit(self._logical, (0, 0))
            pygame.display.flip()
            return

        screen_w, screen_h = self._screen.get_size()
        target_w, target_h = _fit_size(LOGICAL_SIZE[0], LOGICAL_SIZE[1], screen_w, screen_h)
        target_size = (target_w, target_h)
        if self._scaled_surface is None or self._scaled_size != target_size:
            self._scaled_surface = pygame.Surface(target_size).convert()
            self._scaled_size = target_size
        pygame.transform.smoothscale(self._logical, target_size, self._scaled_surface)
        x = (screen_w - target_w) // 2
        y = (screen_h - target_h) // 2
        self._screen.fill(COLORS["black"])
        self._screen.blit(self._scaled_surface, (x, y))
        pygame.display.flip()

    def _export_frame(self) -> None:
        out_dir = REPO_ROOT / "tools" / "mockups"
        out_dir.mkdir(parents=True, exist_ok=True)
        self._screenshot_index += 1
        stem = LAYOUT_NAME.lower().replace(" ", "_")
        path = out_dir / f"observer_dashboard_{stem}_{self._screenshot_index:02d}.png"
        pygame.image.save(self._logical, str(path))
        print(f"[gamemaster_ui] exported {path}", flush=True)

    def _request_control_action(self, action: str, *, source: str) -> None:
        if self._pending_control_request is not None:
            pending_action = _control_action_label(self._pending_control_request.get("action"))
            self._last_control_error = {
                "message": f"Waiting for {pending_action} ack",
                "ts_mono_ns": time.perf_counter_ns(),
            }
            return
        if self._ui_req is None:
            self._last_control_error = {
                "message": "Control channel unavailable",
                "ts_mono_ns": time.perf_counter_ns(),
            }
            self._reset_ui_req_socket()
            return

        env = bus.make_envelope("gamemaster_ui", with_wall=True)
        env.update({
            "request_id": self._next_control_request_id,
            "action": action,
            "source": source,
        })
        self._next_control_request_id += 1
        bus.send_json(self._ui_req, env)
        self._pending_control_request = {
            "request_id": env["request_id"],
            "action": action,
            "deadline_t": time.perf_counter() + UI_COMMAND_ACK_TIMEOUT_S,
        }
        self._last_control_error = None

    def _poll_control_channel(self) -> None:
        pending = self._pending_control_request
        if pending is None or self._ui_req is None:
            return

        events = dict(self._ui_req_poller.poll(timeout=0))
        if events.get(self._ui_req) == zmq.POLLIN:
            reply = bus.recv_json(self._ui_req)
            action = reply.get("result", {}).get("action") if isinstance(reply.get("result"), dict) else None
            if bool(reply.get("ok", False)):
                self._last_control_ack = {
                    "action": action,
                    "ts_mono_ns": time.perf_counter_ns(),
                }
                self._last_control_error = None
            else:
                self._last_control_error = {
                    "message": str(reply.get("error") or "Command rejected"),
                    "ts_mono_ns": time.perf_counter_ns(),
                }
            self._pending_control_request = None
            return

        if time.perf_counter() >= float(pending.get("deadline_t", 0.0)):
            self._last_control_error = {
                "message": f"{_control_action_label(pending.get('action'))} timed out",
                "ts_mono_ns": time.perf_counter_ns(),
            }
            self._pending_control_request = None
            self._reset_ui_req_socket()

    def _screen_to_logical(self, screen_pos: tuple[int, int]) -> tuple[int, int] | None:
        if self._native_4k:
            return int(screen_pos[0]), int(screen_pos[1])
        screen_w, screen_h = self._screen.get_size()
        target_w, target_h = _fit_size(LOGICAL_SIZE[0], LOGICAL_SIZE[1], screen_w, screen_h)
        offset_x = (screen_w - target_w) // 2
        offset_y = (screen_h - target_h) // 2
        if not (offset_x <= screen_pos[0] < offset_x + target_w and offset_y <= screen_pos[1] < offset_y + target_h):
            return None
        logical_x = int((screen_pos[0] - offset_x) * LOGICAL_SIZE[0] / target_w)
        logical_y = int((screen_pos[1] - offset_y) * LOGICAL_SIZE[1] / target_h)
        return logical_x, logical_y

    def _poll_bus(self) -> None:
        while True:
            try:
                _, body = bus.recv(self._state_sub, flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            self._latest_state_full = body
            self._latest_state_recv_mono_ns = time.perf_counter_ns()

        while True:
            try:
                topic, body = bus.recv(self._heartbeat_sub, flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            proc = topic.split(".", 1)[1] if "." in topic else topic
            if proc == "gamemaster_ui":
                continue
            now_ns = time.perf_counter_ns()
            self._heartbeat_body_by_proc[proc] = body
            self._heartbeat_recv_mono_ns[proc] = now_ns
            window = self._heartbeat_window_by_proc.setdefault(proc, deque(maxlen=10))
            window.append(now_ns)

    def _publish_heartbeat_if_due(self) -> None:
        now = time.perf_counter()
        if now < self._next_heartbeat_t:
            return
        env = bus.make_envelope("gamemaster_ui", with_wall=True)
        env.update({
            "loop_hz": float(self._clock.get_fps()),
        })
        bus.publish(self._heartbeat_pub, "heartbeat.gamemaster_ui", env)
        self._next_heartbeat_t = now + (1.0 / HEARTBEAT_HZ)

    def _render_state(self) -> dict[str, Any]:
        body = self._latest_state_full
        if body is None:
            return self._placeholder_state("Waiting for state.full")

        teams = body.get("teams")
        if not isinstance(teams, dict):
            teams = {}

        team_b = self._team_from_state(TEAM_B, teams.get(TEAM_B))
        team_a = self._team_from_state(TEAM_A, teams.get(TEAM_A))
        stage = str(body.get("stage", "waiting"))
        active_stage = str(body.get("active_stage", stage))
        control_feedback = self._control_feedback_state()
        return {
            "stage": stage,
            "active_stage": active_stage,
            "paused": bool(body.get("paused", False)),
            "pause_reason": body.get("pause_reason"),
            "soft_estop": bool(body.get("soft_estop", False)),
            "control_pending_action": control_feedback["pending_action"],
            "control_last_ack_action": control_feedback["last_ack_action"],
            "control_error": control_feedback["error"],
            "timer_label": _format_timer_label(body.get("countdown_s")),
            "team_b": team_b,
            "team_a": team_a,
            "conclusion_winner": _local_conclusion_winner(active_stage, teams),
            "core_processes": self._core_processes_from_heartbeats(),
            "collision_workers": self._collision_workers_from_heartbeats(),
            "waiting_reason": None,
        }

    def _placeholder_state(self, reason: str) -> dict[str, Any]:
        return {
            "stage": "waiting",
            "active_stage": "waiting",
            "paused": False,
            "pause_reason": None,
            "soft_estop": False,
            "control_pending_action": None,
            "control_last_ack_action": None,
            "control_error": None,
            "timer_label": "--:--",
            "team_b": self._placeholder_team(TEAM_B),
            "team_a": self._placeholder_team(TEAM_A),
            "conclusion_winner": None,
            "core_processes": self._core_processes_from_heartbeats(),
            "collision_workers": self._collision_workers_from_heartbeats(),
            "waiting_reason": reason,
        }

    def _control_feedback_state(self) -> dict[str, Any]:
        now_ns = time.perf_counter_ns()
        error = None
        if isinstance(self._last_control_error, dict):
            age_s = (now_ns - int(self._last_control_error.get("ts_mono_ns") or now_ns)) / 1e9
            if age_s <= UI_COMMAND_ERROR_BANNER_S:
                error = self._last_control_error.get("message")

        last_ack_action = None
        if isinstance(self._last_control_ack, dict):
            age_s = (now_ns - int(self._last_control_ack.get("ts_mono_ns") or now_ns)) / 1e9
            if age_s <= UI_COMMAND_ACK_BANNER_S:
                last_ack_action = self._last_control_ack.get("action")

        pending_action = None
        if isinstance(self._pending_control_request, dict):
            pending_action = self._pending_control_request.get("action")

        return {
            "pending_action": pending_action,
            "last_ack_action": last_ack_action,
            "error": error,
        }

    def _placeholder_team(self, team: str) -> TeamMock:
        return TeamMock(
            name=team.upper(),
            color=COLORS["blue"] if team == TEAM_A else COLORS["red"],
            color_soft=COLORS["blue_soft"] if team == TEAM_A else COLORS["red_soft"],
            active=False,
            score=0,
            buckets=[0, 0, 0],
            summed_score=0,
            actual=[0.0] * 6,
            target=[0.0] * 6,
            actual_deg=[0.0] * 6,
            connected=[False] * 6,
            loop_hz=[0.0] * 6,
            in_collision=False,
            path_scalar=0.0,
            final_scalar=0.0,
            first_hit_deg=0.0,
            prox_probe_offsets_deg=[],
            prox_hits=[[] for _ in range(6)],
            prox_age_ticks=[9999] * 6,
        )

    def _team_from_state(self, team: str, team_body: Any) -> TeamMock:
        if not isinstance(team_body, dict):
            return self._placeholder_team(team)

        robot = team_body.get("robot") if isinstance(team_body.get("robot"), dict) else {}
        haptic = team_body.get("haptic") if isinstance(team_body.get("haptic"), dict) else {}
        collision = team_body.get("collision") if isinstance(team_body.get("collision"), dict) else {}
        q_actual = _coerce_float_list(robot.get("q_rad"), 6, 0.0)
        q_target = _coerce_float_list(robot.get("q_target_rad"), 6, 0.0)
        connected = _coerce_bool_list(haptic.get("connected"), 6, False)
        loop_hz = _coerce_float_list(haptic.get("board_loop_hz"), 6, 0.0)
        hit = collision.get("first_hit") if isinstance(collision.get("first_hit"), dict) else {}
        q_actual_deg = [math.degrees(v) for v in q_actual]
        prox_probe_offsets_deg = _coerce_float_list(collision.get("prox_probe_offsets_deg"), 20, 0.0)
        raw_prox_hits = collision.get("prox_hits") if isinstance(collision.get("prox_hits"), list) else []
        prox_hits = [
            [bool(v) for v in axis_hits] if isinstance(axis_hits, list) else []
            for axis_hits in raw_prox_hits[:6]
        ]
        while len(prox_hits) < 6:
            prox_hits.append([])
        prox_age_ticks = [int(v) for v in _coerce_float_list(collision.get("prox_age_ticks"), 6, 9999.0)]
        buckets = _coerce_float_list(team_body.get("buckets"), 3, 0.0)
        return TeamMock(
            name=team.upper(),
            color=COLORS["blue"] if team == TEAM_A else COLORS["red"],
            color_soft=COLORS["blue_soft"] if team == TEAM_A else COLORS["red_soft"],
            active=True,
            score=float(team_body.get("score", sum(buckets)) or 0.0),
            buckets=buckets,
            summed_score=_coerce_int(team_body.get("summed_score"), 0),
            actual=[self._normalize_joint(v, axis) for axis, v in enumerate(q_actual)],
            target=[self._normalize_joint(v, axis) for axis, v in enumerate(q_target)],
            actual_deg=q_actual_deg,
            connected=connected,
            loop_hz=loop_hz,
            in_collision=bool(collision.get("in_collision", False)),
            path_scalar=float(collision.get("path_scalar", 0.0) or 0.0),
            final_scalar=float(collision.get("final_scalar", 0.0) or 0.0),
            first_hit_deg=float(hit.get("distance_deg", 0.0) or 0.0),
            prox_probe_offsets_deg=prox_probe_offsets_deg,
            prox_hits=prox_hits,
            prox_age_ticks=prox_age_ticks,
        )

    def _normalize_joint(self, q_rad: float, axis: int) -> float:
        q_min = self._q_min_rad[axis]
        q_max = self._q_max_rad[axis]
        span = max(1e-6, q_max - q_min)
        clipped = max(q_min, min(q_max, q_rad))
        return ((clipped - q_min) / span) * 2.0 - 1.0

    def _normalize_joint_deg(self, q_deg: float, axis: int) -> float:
        return self._normalize_joint(math.radians(q_deg), axis)

    def _core_processes_from_heartbeats(self) -> list[ProcessMock]:
        preferred = [
            "gamemaster_ui",
            "game_controller",
            "collision_broker",
            "bus_broker",
            "robot_io.a",
            "robot_io.b",
            "haptic_io.a",
            "haptic_io.b",
        ]
        workers = {name for name in self._heartbeat_body_by_proc if name.startswith("collision_worker_")}
        procs = [name for name in self._heartbeat_body_by_proc if name not in workers]
        if "gamemaster_ui" not in procs:
            procs.append("gamemaster_ui")
        order = {name: i for i, name in enumerate(preferred)}
        procs.sort(key=lambda name: (order.get(name, 999), name))
        return [self._process_from_heartbeat(name) for name in procs]

    def _collision_workers_from_heartbeats(self) -> list[ProcessMock]:
        workers_by_index: dict[int, ProcessMock] = {}
        for name in self._heartbeat_body_by_proc:
            if not name.startswith("collision_worker_"):
                continue
            try:
                idx = int(name.rsplit("_", 1)[1])
            except (IndexError, ValueError):
                continue
            if 0 <= idx < MAX_COLLISION_WORKER_SLOTS:
                workers_by_index[idx] = self._process_from_heartbeat(name)

        workers: list[ProcessMock] = []
        for idx in range(MAX_COLLISION_WORKER_SLOTS):
            proc_key = f"collision_worker_{idx:02d}"
            if idx < self._collision_worker_count:
                workers.append(
                    workers_by_index.get(
                        idx,
                        ProcessMock(
                            name=f"worker_{idx:02d}",
                            proc_key=proc_key,
                            hz=0.0,
                            age_ms=0.0,
                            checks_per_sec=0.0,
                            active=True,
                        ),
                    )
                )
            else:
                workers.append(
                    ProcessMock(
                        name=f"worker_{idx:02d}",
                        proc_key=proc_key,
                        hz=0.0,
                        age_ms=0.0,
                        checks_per_sec=None,
                        active=False,
                    )
                )
        return workers

    def _process_from_heartbeat(self, name: str) -> ProcessMock:
        if name == "gamemaster_ui":
            return ProcessMock(
                name="gamemaster_ui",
                proc_key=name,
                hz=float(self._clock.get_fps()),
                age_ms=0.0,
            )
        body = self._heartbeat_body_by_proc.get(name, {})
        last_ns = self._heartbeat_recv_mono_ns.get(name)
        age_ms = 0.0 if last_ns is None else (time.perf_counter_ns() - last_ns) / 1e6
        checks = body.get("checks_per_sec")
        proc_name = name.replace("collision_worker_", "worker_")
        return ProcessMock(
            name=proc_name,
            proc_key=name,
            hz=float(body.get("loop_hz", 0.0) or 0.0),
            age_ms=age_ms,
            checks_per_sec=None if checks is None else float(checks),
        )

    def _draw(self, state: dict) -> None:
        surface = self._logical
        self._draw_background(surface)
        self._draw_chrome(surface, state)
        self._draw_central_spine(surface, state)

    def _build_background_surface(self) -> pygame.Surface:
        surface = pygame.Surface(LOGICAL_SIZE, pygame.SRCALPHA).convert_alpha()
        top = COLORS["bg_2"]
        bottom = COLORS["bg"]
        for y in range(LOGICAL_SIZE[1]):
            mix = y / LOGICAL_SIZE[1]
            color = (
                int(top[0] * (1.0 - mix) + bottom[0] * mix),
                int(top[1] * (1.0 - mix) + bottom[1] * mix),
                int(top[2] * (1.0 - mix) + bottom[2] * mix),
            )
            pygame.draw.line(surface, color, (0, y), (LOGICAL_SIZE[0], y))

        for i in range(18):
            alpha = 18 if i % 2 == 0 else 10
            stripe = pygame.Surface((LOGICAL_SIZE[0], 18), pygame.SRCALPHA)
            stripe.fill((255, 255, 255, alpha))
            y = 100 + i * 110
            surface.blit(stripe, (0, y))

        pygame.draw.circle(surface, (34, 57, 80), (620, 430), 340, width=2)
        pygame.draw.circle(surface, (45, 64, 85), (3240, 1680), 420, width=2)
        return surface

    def _draw_background(self, surface: pygame.Surface) -> None:
        surface.blit(self._background, (0, 0))

    def _draw_chrome(self, surface: pygame.Surface, state: dict) -> None:
        ui_proc = self._process_from_heartbeat("gamemaster_ui")
        ui_fps_min, _ = self._process_status_limits(ui_proc.proc_key)
        ui_fps_color = COLORS["warning"] if ui_fps_min is not None and ui_proc.hz < ui_fps_min else COLORS["success"]
        self._label(surface, LAYOUT_NAME, 60, 42, 38, COLORS["text"], bold=True)
        self._label(surface, f"{LAYOUT_STYLE['name']} | central spine family", 60, 88, 24, COLORS["cyan"], bold=True)
        self._label(surface, "Logical canvas 3840x2160, scaled to current window", 60, 118, 22, COLORS["muted"])
        self._label(surface, "Keys: P play/resume | Space soft e-stop | E end game | F fullscreen | Esc quit", 60, 148, 20, COLORS["muted"])
        self._label(surface, f"UI {ui_proc.hz:4.1f} Hz", LOGICAL_SIZE[0] - 60, 44, 28, ui_fps_color, bold=True, align="right")
        if ui_fps_min is None:
            detail = f"target {self._target_fps:.0f}"
        else:
            detail = f"target {self._target_fps:.0f} | warn < {ui_fps_min:.0f}"
        self._label(surface, detail, LOGICAL_SIZE[0] - 60, 84, 20, COLORS["muted"], align="right")

    def _draw_central_spine(self, surface: pygame.Surface, state: dict) -> None:
        style = self._current_style()
        left = pygame.Rect(70, 210, 1430, 1780)
        spine = pygame.Rect(1510, 210, 820, 1780)
        right = pygame.Rect(2340, 210, 1430, 1780)
        self._team_panel_compact(surface, left, state["team_b"], align="left", style=style, stage=state["active_stage"], conclusion_winner=state.get("conclusion_winner"))
        self._team_panel_compact(surface, right, state["team_a"], align="right", style=style, stage=state["active_stage"], conclusion_winner=state.get("conclusion_winner"))
        self._central_spine(surface, spine, state, style)

    def _panel(self, surface: pygame.Surface, rect: pygame.Rect, alt: bool = False) -> None:
        style = self._current_style()
        color = COLORS["panel_alt"] if alt else COLORS["panel"]
        radius = int(style["panel_radius"])
        outline_width = int(style["outline_width"])
        pygame.draw.rect(surface, color, rect, border_radius=radius)
        pygame.draw.rect(surface, COLORS["outline"], rect, outline_width, border_radius=radius)
        if style["corner_brackets"]:
            self._draw_corner_brackets(surface, rect, COLORS["outline"])

    def _lane_bank(self, surface: pygame.Surface, rect: pygame.Rect, team: TeamMock, title: str) -> None:
        self._panel(surface, rect, alt=True)
        self._label(surface, title, rect.x + 26, rect.y + 18, 34, COLORS["text"], bold=True)
        lane_gap = 18
        lane_w = (rect.w - 52 - lane_gap * 5) // 6
        lane_top = rect.y + 78
        lane_h = rect.h - 124
        for axis in range(6):
            lane_rect = pygame.Rect(rect.x + 26 + axis * (lane_w + lane_gap), lane_top, lane_w, lane_h)
            self._draw_lane(surface, lane_rect, axis, team)

    def _draw_lane(self, surface: pygame.Surface, rect: pygame.Rect, axis: int, team: TeamMock) -> None:
        style = self._current_style()
        lane_radius = int(style["lane_radius"])
        connected = team.connected[axis]
        border = team.color if connected else COLORS["grey"]
        fill = (24, 36, 48) if connected else (34, 36, 39)
        pygame.draw.rect(surface, fill, rect, border_radius=lane_radius)
        pygame.draw.rect(surface, border, rect, 2, border_radius=lane_radius)

        center_x = rect.centerx
        actual = team.actual[axis]
        target = team.target[axis]
        actual_y = _value_to_lane_y(actual, rect)
        target_y = _value_to_lane_y(target, rect)

        for step in range(1, 5):
            y = rect.y + step * rect.h // 5
            pygame.draw.line(surface, COLORS["grid"], (rect.x + 10, y), (rect.right - 10, y), 1)

        self._draw_lane_proximity(surface, rect, axis, team, connected)

        pygame.draw.line(surface, team.color_soft if connected else COLORS["grey"], (rect.x + 12, actual_y), (rect.right - 12, actual_y), 6)
        pygame.draw.line(surface, COLORS["text"] if connected else COLORS["grey"], (center_x, target_y), (center_x, actual_y), 3)
        direction = -18 if target_y < actual_y else 18
        arrow = [
            (center_x, target_y),
            (center_x - 12, target_y + direction),
            (center_x + 12, target_y + direction),
        ]
        pygame.draw.polygon(surface, COLORS["text"] if connected else COLORS["grey"], arrow)
        self._label(surface, f"J{axis + 1}", rect.centerx, rect.y + 14, 20, COLORS["muted"], align="center")
        self._label(surface, f"{team.actual_deg[axis]:+.0f}°", rect.centerx, rect.bottom - 44, 20, COLORS["text"] if connected else COLORS["grey"], align="center")
        hz_color = COLORS["warning"] if team.loop_hz[axis] < 190 else COLORS["muted"]
        self._label(surface, f"{team.loop_hz[axis]:.0f}Hz", rect.centerx, rect.bottom - 20, 18, hz_color, align="center")
        if not connected:
            self._label(surface, "OFF", rect.centerx, rect.centery - 18, 30, COLORS["grey"], bold=True, align="center")

    def _bucket_strip(self, surface: pygame.Surface, rect: pygame.Rect, team: TeamMock) -> None:
        self._panel(surface, rect, alt=True)
        labels = ("LEFT", "MID", "RIGHT")
        box_w = (rect.w - 48) // 3
        for i, label in enumerate(labels):
            box = pygame.Rect(rect.x + 16 + i * box_w, rect.y + 14, box_w - 8, rect.h - 28)
            pygame.draw.rect(surface, COLORS["panel_soft"], box, border_radius=16)
            pygame.draw.rect(surface, team.color, box, 2, border_radius=16)
            self._label(surface, label, box.centerx, box.y + 18, 20, COLORS["muted"], align="center")
            self._label(surface, _format_bucket_value(team.buckets[i]), box.centerx, box.y + 46, 44, COLORS["text"], bold=True, align="center")

    def _team_summary_strip(self, surface: pygame.Surface, rect: pygame.Rect, team: TeamMock, align: str, stage: str, conclusion_winner: str | None) -> None:
        self._panel(surface, rect, alt=True)
        if stage == "conclusion" and team.active:
            self._draw_conclusion_summary(surface, rect, team, align=align, conclusion_winner=conclusion_winner)
            return
        badge = "COLLISION" if team.in_collision else "FREE"
        badge_color = COLORS["danger"] if team.in_collision else COLORS["success"]
        left_x = rect.x + 26
        right_x = rect.right - 26
        final_reduced = team.final_scalar < 0.999
        final_color = COLORS["warning"] if final_reduced else COLORS["text"]
        self._label(surface, badge, left_x if align == "left" else right_x, rect.y + 18, 28, badge_color, bold=True, align=align)
        self._label(surface, f"Final {team.final_scalar * 100:.0f}%", left_x if align == "left" else right_x, rect.y + 58, 38, final_color, bold=True, align=align)
        self._label(surface, f"First hit {team.first_hit_deg:.0f}°", left_x if align == "left" else right_x, rect.y + 104, 22, COLORS["muted"], align=align)

        bar_rect = pygame.Rect(rect.x + 320, rect.y + 38, rect.w - 640, 38)
        pygame.draw.rect(surface, COLORS["warning"], bar_rect, border_radius=14)
        fill_w = int(bar_rect.w * max(0.0, min(1.0, team.final_scalar)))
        fill_rect = pygame.Rect(bar_rect.x, bar_rect.y, fill_w, bar_rect.h)
        pygame.draw.rect(surface, COLORS["text"], fill_rect, border_radius=14)
        self._label(surface, f"Speed scaler {team.final_scalar * 100:.0f}% | Path {team.path_scalar * 100:.0f}%", bar_rect.centerx, rect.y + 98, 22, COLORS["muted"], align="center")

    def _draw_conclusion_summary(self, surface: pygame.Surface, rect: pygame.Rect, team: TeamMock, align: str, conclusion_winner: str | None) -> None:
        left_x = rect.x + 26
        right_x = rect.right - 26
        anchor_x = left_x if align == "left" else right_x
        outcome_label, outcome_color = _team_conclusion_outcome(team.name.lower(), conclusion_winner)
        self._label(surface, "TOTAL SCORE", anchor_x, rect.y + 18, 28, team.color, bold=True, align=align)
        self._label(surface, str(team.summed_score), anchor_x, rect.y + 54, 56, COLORS["text"], bold=True, align=align)
        subtitle = "Summing live bucket values" if conclusion_winner is None else "All buckets empty"
        self._label(surface, subtitle, anchor_x, rect.y + 112, 22, COLORS["muted"], align=align)

        if outcome_label is not None:
            self._label(surface, outcome_label, right_x if align == "left" else left_x, rect.y + 18, 28, outcome_color, bold=True, align="right" if align == "left" else "left")

        bar_rect = pygame.Rect(rect.x + 320, rect.y + 38, rect.w - 640, 38)
        pygame.draw.rect(surface, COLORS["panel_soft"], bar_rect, border_radius=14)
        total_remaining = max(0, sum(int(round(max(0.0, value))) for value in team.buckets))
        total_capacity = max(1, team.summed_score + total_remaining)
        fill_w = int(bar_rect.w * max(0.0, min(1.0, team.summed_score / total_capacity)))
        fill_rect = pygame.Rect(bar_rect.x, bar_rect.y, fill_w, bar_rect.h)
        pygame.draw.rect(surface, team.color, fill_rect, border_radius=14)
        self._label(surface, f"Remaining {total_remaining} | Summed {team.summed_score}", bar_rect.centerx, rect.y + 98, 22, COLORS["muted"], align="center")

    def _match_core_widget(self, surface: pygame.Surface, rect: pygame.Rect, state: dict) -> None:
        self._panel(surface, rect, alt=True)
        self._label(surface, str(state.get("timer_label", "--:--")), rect.centerx, rect.y + 28, 104, COLORS["text"], bold=True, align="center")
        self._label(surface, state["stage"].upper(), rect.centerx, rect.y + 148, 34, COLORS["cyan"], bold=True, align="center")
        waiting_reason = state.get("waiting_reason")
        if isinstance(waiting_reason, str) and waiting_reason:
            self._label(surface, waiting_reason, rect.centerx, rect.y + 202, 28, COLORS["warning"], align="center")
            self._label(surface, "Connect launcher and publishers to replace this placeholder with live state.", rect.centerx, rect.y + 244, 22, COLORS["muted"], align="center")
        else:
            self._label(surface, "Total score stays hidden until play ends", rect.centerx, rect.y + 202, 28, COLORS["text"], align="center")
            self._label(surface, "Post-play sum-up animation will total the three bucket values.", rect.centerx, rect.y + 244, 22, COLORS["muted"], align="center")

    def _team_panel_compact(self, surface: pygame.Surface, rect: pygame.Rect, team: TeamMock, align: str, style: dict, stage: str, conclusion_winner: str | None) -> None:
        self._panel(surface, rect)
        self._team_spine_header(surface, pygame.Rect(rect.x + 28, rect.y + 28, rect.w - 56, 145), team, align=align)
        self._bucket_strip(surface, pygame.Rect(rect.x + 28, rect.y + 195, rect.w - 56, 128), team)
        self._team_summary_strip(surface, pygame.Rect(rect.x + 28, rect.y + 345, rect.w - 56, 142), team, align=align, stage=stage, conclusion_winner=conclusion_winner)
        self._lane_bank(surface, pygame.Rect(rect.x + 28, rect.y + 515, rect.w - 56, rect.h - 545), team, title=f"TEAM {team.name}")

    def _central_spine(self, surface: pygame.Surface, rect: pygame.Rect, state: dict, style: dict) -> None:
        self._panel(surface, rect)
        inset = int(style["spine_inset"])
        match_inset = int(style["match_widget_inset"])
        self._match_core_widget(surface, pygame.Rect(rect.x + match_inset, rect.y + 28, rect.w - match_inset * 2, 270), state)
        self._core_process_table(surface, pygame.Rect(rect.x + inset, rect.y + 330, rect.w - inset * 2, 320), state["core_processes"], style)
        self._collision_worker_grid(surface, pygame.Rect(rect.x + inset, rect.y + 680, rect.w - inset * 2, 650), state["collision_workers"], style)
        self._control_widget(surface, self._control_widget_rect(), state)

    def _control_widget_rect(self) -> pygame.Rect:
        inset = int(self._current_style()["spine_inset"])
        return pygame.Rect(1510 + inset, 1570, 820 - inset * 2, 380)

    def _control_widget(self, surface: pygame.Surface, rect: pygame.Rect, state: dict) -> None:
        self._panel(surface, rect, alt=True)
        self._label(surface, "MATCH CONTROL", rect.x + 22, rect.y + 16, 28, COLORS["text"], bold=True)
        status_text, status_color = _control_status_text(state)
        self._label(surface, status_text, rect.right - 22, rect.y + 18, 20, status_color, bold=True, align="right")
        self._label(surface, "Click a button or use the keyboard shortcuts shown inside each button.", rect.x + 22, rect.y + 56, 20, COLORS["muted"])

        for spec in self._control_button_specs(rect, state):
            button_rect = spec["rect"]
            fill_color = spec["fill"]
            outline_color = spec["outline"]
            pygame.draw.rect(surface, fill_color, button_rect, border_radius=18)
            pygame.draw.rect(surface, outline_color, button_rect, 3, border_radius=18)
            self._label(surface, spec["label"], button_rect.centerx, button_rect.y + 26, 30, COLORS["text"], bold=True, align="center")
            self._label(surface, spec["detail"], button_rect.centerx, button_rect.y + 72, 18, COLORS["muted"], align="center")
            shortcut_rect = pygame.Rect(button_rect.x + 18, button_rect.bottom - 42, 88, 24)
            pygame.draw.rect(surface, COLORS["panel_soft"], shortcut_rect, border_radius=10)
            self._label(surface, spec["shortcut"], shortcut_rect.centerx, shortcut_rect.y + 2, 16, COLORS["text"], bold=True, align="center")

    def _control_button_specs(self, rect: pygame.Rect, state: dict) -> list[dict[str, Any]]:
        gap = 18
        button_w = (rect.w - 44 - gap * 2) // 3
        button_h = 214
        top = rect.y + 110
        left = rect.x + 22
        paused = bool(state.get("paused", False))
        soft_estop = bool(state.get("soft_estop", False)) or state.get("pause_reason") == "soft_estop"
        return [
            {
                "action": "play_resume",
                "label": "PLAY / RESUME",
                "detail": "Continue the current stage after a pause.",
                "shortcut": "P",
                "rect": pygame.Rect(left, top, button_w, button_h),
                "fill": (28, 66, 52) if paused else COLORS["panel_soft"],
                "outline": COLORS["success"],
            },
            {
                "action": "end_game",
                "label": "END GAME",
                "detail": "Jump immediately to conclusion scoring.",
                "shortcut": "E",
                "rect": pygame.Rect(left + button_w + gap, top, button_w, button_h),
                "fill": COLORS["panel_soft"],
                "outline": COLORS["warning"],
            },
            {
                "action": "soft_estop",
                "label": "E-STOP",
                "detail": "Pause robot response and freeze the timer.",
                "shortcut": "SPACE",
                "rect": pygame.Rect(left + (button_w + gap) * 2, top, button_w, button_h),
                "fill": (88, 34, 34) if soft_estop else COLORS["panel_soft"],
                "outline": COLORS["danger"],
            },
        ]

    def _control_action_at(self, logical_pos: tuple[int, int], state: dict[str, Any]) -> str | None:
        rect = self._control_widget_rect()
        for spec in self._control_button_specs(rect, state):
            if spec["rect"].collidepoint(logical_pos):
                return str(spec["action"])
        return None

    def _team_spine_header(self, surface: pygame.Surface, rect: pygame.Rect, team: TeamMock, align: str) -> None:
        self._panel(surface, rect, alt=True)
        title_x = rect.x + 30 if align == "left" else rect.right - 30
        hint_x = rect.right - 30 if align == "left" else rect.x + 30
        self._label(surface, f"TEAM {team.name}", title_x, rect.y + 18, 44, team.color, bold=True, align=align)
        self._label(surface, "Final total hidden until post-play bucket sum-up", title_x, rect.y + 74, 24, COLORS["muted"], align=align)
        self._label(surface, "3 live bucket values only", hint_x, rect.y + 18, 22, COLORS["grey"], align="right" if align == "left" else "left")
        badge = "COLLISION" if team.in_collision else "FREE"
        badge_color = COLORS["danger"] if team.in_collision else COLORS["success"]
        self._label(surface, badge, hint_x, rect.y + 56, 30, badge_color, bold=True, align="right" if align == "left" else "left")

    def _draw_lane_proximity(self, surface: pygame.Surface, rect: pygame.Rect, axis: int, team: TeamMock, connected: bool) -> None:
        rail_gap = 8
        rail_w = 10
        left_rail = pygame.Rect(rect.x + 8, rect.y + 10, rail_w, rect.h - 20)
        right_rail = pygame.Rect(rect.right - 8 - rail_w, rect.y + 10, rail_w, rect.h - 20)
        for rail in (left_rail, right_rail):
            pygame.draw.rect(surface, COLORS["grid"], rail)

        if not connected:
            off_overlay = pygame.Surface((rect.w - 20, rect.h - 20), pygame.SRCALPHA)
            off_overlay.fill((90, 90, 90, 28))
            surface.blit(off_overlay, (rect.x + 10, rect.y + 10))
            return

        probe_offsets = team.prox_probe_offsets_deg
        probe_hits = team.prox_hits[axis] if axis < len(team.prox_hits) else []
        probe_age = team.prox_age_ticks[axis] if axis < len(team.prox_age_ticks) else 9999
        if not probe_offsets or not probe_hits or probe_age > 12:
            muted = pygame.Surface((rect.w - 20, rect.h - 20), pygame.SRCALPHA)
            muted.fill((120, 120, 120, 20))
            surface.blit(muted, (rect.x + 10, rect.y + 10))
            return

        step_deg = 1.0
        if len(probe_offsets) >= 2:
            diffs = [abs(probe_offsets[i + 1] - probe_offsets[i]) for i in range(len(probe_offsets) - 1) if abs(probe_offsets[i + 1] - probe_offsets[i]) > 1e-6]
            if diffs:
                step_deg = min(diffs)

        red_ranges: list[tuple[int, int]] = []
        for offset_deg, is_hit in zip(probe_offsets, probe_hits):
            center_deg = team.actual_deg[axis] + float(offset_deg)
            lo = self._normalize_joint_deg(center_deg - step_deg * 0.5, axis)
            hi = self._normalize_joint_deg(center_deg + step_deg * 0.5, axis)
            top = min(_value_to_lane_y(lo, rect), _value_to_lane_y(hi, rect))
            bottom = max(_value_to_lane_y(lo, rect), _value_to_lane_y(hi, rect))
            seg_h = max(8, bottom - top)
            left_seg = pygame.Rect(left_rail.x, top, rail_w, seg_h)
            right_seg = pygame.Rect(right_rail.x, top, rail_w, seg_h)
            seg_color = COLORS["danger"] if is_hit else COLORS["success"]
            pygame.draw.rect(surface, seg_color, left_seg)
            pygame.draw.rect(surface, seg_color, right_seg)
            if is_hit:
                red_ranges.append((top, top + seg_h))

        if red_ranges:
            hatch_top = min(r[0] for r in red_ranges)
            hatch_bottom = max(r[1] for r in red_ranges)
            hatch_h = max(14, hatch_bottom - hatch_top)
            hatch = pygame.Surface((rect.w - 42, hatch_h), pygame.SRCALPHA)
            for x in range(-hatch_h, hatch.get_width(), 18):
                pygame.draw.line(hatch, (164, 27, 27, 110), (x, hatch_h), (x + hatch_h, 0), 4)
            surface.blit(hatch, (rect.x + 21, hatch_top))

    def _core_process_table(self, surface: pygame.Surface, rect: pygame.Rect, processes: list[ProcessMock], style: dict) -> None:
        self._panel(surface, rect, alt=True)
        self._label(surface, "CORE PROCESS HEALTH", rect.x + 22, rect.y + 16, 28, COLORS["text"], bold=True)
        self._label(surface, "proc", rect.x + 24, rect.y + 62, 18, COLORS["muted"])
        self._label(surface, "hz", rect.x + 370, rect.y + 62, 18, COLORS["muted"])
        self._label(surface, "age", rect.x + 490, rect.y + 62, 18, COLORS["muted"])
        row_y = rect.y + 96
        for proc in processes:
            fps_min, heartbeat_age_max_ms = self._process_status_limits(proc.proc_key)
            hz_color = COLORS["warning"] if fps_min is not None and proc.hz < fps_min else COLORS["text"]
            age_color = COLORS["warning"] if proc.age_ms > heartbeat_age_max_ms else COLORS["text"]
            self._label(surface, proc.name, rect.x + 24, row_y, 23, COLORS["text"])
            self._label(surface, f"{proc.hz:5.1f}", rect.x + 370, row_y, 23, hz_color)
            self._label(surface, f"{proc.age_ms:4.0f} ms", rect.x + 490, row_y, 23, age_color)
            line_width = 2 if style["heavy_dividers"] else 1
            pygame.draw.line(surface, COLORS["grid"], (rect.x + 20, row_y + 34), (rect.right - 20, row_y + 34), line_width)
            row_y += 42

    def _collision_worker_grid(self, surface: pygame.Surface, rect: pygame.Rect, workers: list[ProcessMock], style: dict) -> None:
        self._panel(surface, rect, alt=True)
        self._label(surface, "COLLISION WORKERS", rect.x + 22, rect.y + 16, 28, COLORS["text"], bold=True)
        self._label(surface, f"Configured {self._collision_worker_count} / {MAX_COLLISION_WORKER_SLOTS} slots", rect.right - 22, rect.y + 20, 18, COLORS["muted"], align="right")
        cols = 4
        gap = 16
        tile_w = (rect.w - 44 - gap * (cols - 1)) // cols
        tile_h = 108 if style["heavy_dividers"] else 126
        start_x = rect.x + 22
        start_y = rect.y + 58
        for i, worker in enumerate(workers[:MAX_COLLISION_WORKER_SLOTS]):
            col = i % cols
            row = i // cols
            tile = pygame.Rect(start_x + col * (tile_w + gap), start_y + row * (tile_h + gap), tile_w, tile_h)
            if worker.active:
                pygame.draw.rect(surface, COLORS["panel_soft"], tile, border_radius=int(style["worker_tile_radius"]))
                pygame.draw.rect(surface, COLORS["outline"], tile, 1 if not style["heavy_dividers"] else 2, border_radius=int(style["worker_tile_radius"]))
                fps_min, heartbeat_age_max_ms = self._process_status_limits(worker.proc_key)
                hz_color = COLORS["warning"] if fps_min is not None and worker.hz < fps_min else COLORS["text"]
                age_color = COLORS["warning"] if worker.age_ms > heartbeat_age_max_ms else COLORS["muted"]
                self._label(surface, worker.name, tile.x + 14, tile.y + 12, 20, COLORS["muted"])
                self._label(surface, f"{(worker.checks_per_sec or 0.0):.0f}/s", tile.x + 14, tile.y + 46, 36, hz_color, bold=True)
                self._label(surface, f"age {worker.age_ms:.0f} ms", tile.x + 14, tile.y + 86, 18, age_color)
            else:
                pygame.draw.rect(surface, (40, 48, 58), tile, border_radius=int(style["worker_tile_radius"]))
                pygame.draw.rect(surface, COLORS["grid"], tile, 1, border_radius=int(style["worker_tile_radius"]))
                self._label(surface, worker.name, tile.x + 14, tile.y + 12, 20, COLORS["grey"])
                self._label(surface, "inactive", tile.x + 14, tile.y + 54, 22, COLORS["grey"], bold=True)

    def _lane_legend(self, surface: pygame.Surface, rect: pygame.Rect, style: dict) -> None:
        self._panel(surface, rect, alt=True)
        self._label(surface, "LANE LEGEND", rect.x + 22, rect.y + 16, 28, COLORS["text"], bold=True)
        self._label(surface, f"Style study: {style['name']}", rect.right - 22, rect.y + 18, 20, COLORS["cyan"], align="right")
        rail_x = rect.x + 32
        rail_y = rect.y + 70
        rail_h = rect.h - 110
        rail = pygame.Rect(rail_x, rail_y, 12, rail_h)
        pygame.draw.rect(surface, COLORS["grid"], rail)
        green = pygame.Rect(rail_x, rail_y + 28, 12, 74)
        red = pygame.Rect(rail_x, rail_y + 102, 12, 52)
        pygame.draw.rect(surface, COLORS["success"], green)
        pygame.draw.rect(surface, COLORS["danger"], red)
        self._label(surface, "Green rail segment = locally allowable motion near the current pose", rect.x + 72, rect.y + 76, 22, COLORS["text"])
        self._label(surface, "Red rail segment + hatch = nearby collision-limited region", rect.x + 72, rect.y + 112, 22, COLORS["text"])
        self._label(surface, "Everything outside those segments stays subdued grey instead of glowing the full lane.", rect.x + 72, rect.y + 148, 22, COLORS["muted"])
        self._label(surface, "This keeps the high-contrast feedback concentrated around the current robot position, closer to the reference mockup.", rect.x + 72, rect.y + 184, 22, COLORS["muted"])

    def _draw_corner_brackets(self, surface: pygame.Surface, rect: pygame.Rect, color: tuple[int, int, int]) -> None:
        size = 20
        width = 3
        corners = [
            ((rect.left + 12, rect.top + 12), (rect.left + 12 + size, rect.top + 12), (rect.left + 12, rect.top + 12 + size)),
            ((rect.right - 12, rect.top + 12), (rect.right - 12 - size, rect.top + 12), (rect.right - 12, rect.top + 12 + size)),
            ((rect.left + 12, rect.bottom - 12), (rect.left + 12 + size, rect.bottom - 12), (rect.left + 12, rect.bottom - 12 - size)),
            ((rect.right - 12, rect.bottom - 12), (rect.right - 12 - size, rect.bottom - 12), (rect.right - 12, rect.bottom - 12 - size)),
        ]
        for origin, horiz, vert in corners:
            pygame.draw.line(surface, color, origin, horiz, width)
            pygame.draw.line(surface, color, origin, vert, width)

    def _current_style(self) -> dict:
        return LAYOUT_STYLE

    def _process_status_limits(self, proc_key: str) -> tuple[float | None, float]:
        base_proc = _subsystem_name_from_proc(proc_key)
        fps_min = default_runtime_setting(base_proc, "fps_min", None)
        heartbeat_age_max_ms = default_runtime_setting(base_proc, "heartbeat_age_max", DEFAULT_HEARTBEAT_AGE_MAX_MS)
        if self._profile is not None:
            fps_min = self._profile.subsystem_float(base_proc, "fps_min", fps_min)
            heartbeat_age_max_ms = self._profile.subsystem_float(base_proc, "heartbeat_age_max", heartbeat_age_max_ms)
        return fps_min, heartbeat_age_max_ms

    def _label(
        self,
        surface: pygame.Surface,
        text: str,
        x: int,
        y: int,
        size: int,
        color: tuple[int, int, int],
        *,
        bold: bool = False,
        align: str = "left",
    ) -> None:
        font = self._fonts.bold(size) if bold else self._fonts.medium(size)
        rendered = font.render(text, True, color)
        rect = rendered.get_rect()
        if align == "right":
            rect.topright = (x, y)
        elif align == "center":
            rect.midtop = (x, y)
        else:
            rect.topleft = (x, y)
        surface.blit(rendered, rect)


def _fit_size(src_w: int, src_h: int, max_w: int, max_h: int) -> tuple[int, int]:
    scale = min(max_w / src_w, max_h / src_h)
    return max(1, int(src_w * scale)), max(1, int(src_h * scale))


def _value_to_lane_y(value: float, rect: pygame.Rect) -> int:
    clipped = max(-1.0, min(1.0, value))
    norm = (clipped + 1.0) * 0.5
    return int(rect.bottom - norm * rect.h)


def _coerce_float_list(value: Any, length: int, default: float) -> list[float]:
    if not isinstance(value, list):
        return [default] * length
    out = []
    for item in value[:length]:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            out.append(default)
    if len(out) < length:
        out.extend([default] * (length - len(out)))
    return out


def _coerce_bool_list(value: Any, length: int, default: bool) -> list[bool]:
    if not isinstance(value, list):
        return [default] * length
    out = [bool(item) for item in value[:length]]
    if len(out) < length:
        out.extend([default] * (length - len(out)))
    return out


def _format_timer_label(value: Any) -> str:
    try:
        total_s = max(0, int(math.ceil(float(value))))
    except (TypeError, ValueError):
        return "--:--"
    minutes, seconds = divmod(total_s, 60)
    return f"{minutes:02d}:{seconds:02d}"


def _format_bucket_value(value: float) -> str:
    rounded = int(round(value))
    return str(rounded)


def _coerce_int(value: Any, default: int) -> int:
    try:
        return int(round(float(value)))
    except (TypeError, ValueError):
        return default


def _local_conclusion_winner(stage: str, teams: dict[str, Any]) -> str | None:
    if stage != "conclusion" or not isinstance(teams, dict):
        return None
    active = [
        (team, body)
        for team, body in teams.items()
        if isinstance(team, str) and isinstance(body, dict)
    ]
    if not active:
        return None
    if not all(_team_buckets_empty(body) for _, body in active):
        return None
    scores = sorted(
        ((team, _coerce_int(body.get("summed_score"), 0)) for team, body in active),
        key=lambda item: item[1],
        reverse=True,
    )
    if len(scores) >= 2 and scores[0][1] == scores[1][1]:
        return "tie"
    return scores[0][0]


def _team_buckets_empty(team_body: dict[str, Any]) -> bool:
    buckets = team_body.get("buckets")
    if not isinstance(buckets, list):
        return False
    return all(abs(float(value)) < 1e-6 for value in buckets[:3])


def _team_conclusion_outcome(team_key: str, conclusion_winner: str | None) -> tuple[str | None, tuple[int, int, int]]:
    if conclusion_winner is None:
        return None, COLORS["muted"]
    if conclusion_winner == "tie":
        return "TIE", COLORS["cyan"]
    if conclusion_winner == team_key:
        return "WINNER", COLORS["success"]
    return "LOSE", COLORS["warning"]


def _control_action_label(action: Any) -> str:
    if not isinstance(action, str):
        return "COMMAND"
    return CONTROL_ACTION_LABELS.get(action, action.replace("_", " ").upper())


def _control_status_text(state: dict[str, Any]) -> tuple[str, tuple[int, int, int]]:
    pending_action = state.get("control_pending_action")
    if isinstance(pending_action, str) and pending_action:
        return f"WAITING ACK | {_control_action_label(pending_action)}", COLORS["warning"]

    control_error = state.get("control_error")
    if isinstance(control_error, str) and control_error:
        return control_error.upper(), COLORS["danger"]

    if bool(state.get("soft_estop", False)) or state.get("pause_reason") == "soft_estop":
        return "SOFT E-STOP ACTIVE", COLORS["danger"]
    if bool(state.get("paused", False)):
        reason = state.get("pause_reason")
        if isinstance(reason, str) and reason:
            return f"PAUSED | {reason}", COLORS["warning"]
        return "PAUSED", COLORS["warning"]

    last_ack_action = state.get("control_last_ack_action")
    if isinstance(last_ack_action, str) and last_ack_action:
        return f"ACKED | {_control_action_label(last_ack_action)}", COLORS["cyan"]

    active_stage = str(state.get("active_stage", state.get("stage", "waiting"))).upper()
    return f"ACTIVE STAGE | {active_stage}", COLORS["success"]


def _load_profile_if_available(profile_path: str | None):
    if not profile_path:
        return None
    try:
        return load_profile(profile_path)
    except (ConfigError, OSError, ValueError):
        return None


def _profile_joint_limits_rad(profile) -> tuple[list[float], list[float]]:
    if profile is None:
        return list(DEFAULT_Q_MIN_RAD), list(DEFAULT_Q_MAX_RAD)
    robot_tune = profile.tuning.get("robot") if isinstance(profile.tuning, dict) else None
    if not isinstance(robot_tune, dict):
        return list(DEFAULT_Q_MIN_RAD), list(DEFAULT_Q_MAX_RAD)
    q_min_deg = _coerce_float_list(robot_tune.get("q_limits_min_deg"), 6, -180.0)
    q_max_deg = _coerce_float_list(robot_tune.get("q_limits_max_deg"), 6, 180.0)
    return [math.radians(v) for v in q_min_deg], [math.radians(v) for v in q_max_deg]


def _profile_collision_worker_count(profile) -> int:
    if profile is None:
        return MAX_COLLISION_WORKER_SLOTS
    node = profile.subsystems.get("collision_workers") if isinstance(profile.subsystems, dict) else None
    if not isinstance(node, dict):
        return MAX_COLLISION_WORKER_SLOTS
    try:
        return max(0, min(MAX_COLLISION_WORKER_SLOTS, int(node.get("count", MAX_COLLISION_WORKER_SLOTS))))
    except (TypeError, ValueError):
        return MAX_COLLISION_WORKER_SLOTS


def _profile_fps_target(profile, subsystem: str) -> float:
    target = default_runtime_setting(subsystem, "fps_target", 30.0)
    if profile is not None:
        target = profile.subsystem_float(subsystem, "fps_target", target)
    try:
        return max(1.0, float(target))
    except (TypeError, ValueError):
        return 30.0


def _subsystem_name_from_proc(proc_key: str) -> str:
    if proc_key.startswith("collision_worker_"):
        return "collision_workers"
    return proc_key.split(".", 1)[0]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="4k-scaled observer dashboard mockup")
    parser.add_argument("--profile", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--proc", default="gamemaster_ui", help=argparse.SUPPRESS)
    parser.add_argument("--instance", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument(
        "--native-4k",
        action="store_true",
        help="Open an actual 3840x2160 window instead of fitting to the monitor",
    )
    parser.add_argument(
        "--fit",
        type=float,
        default=0.9,
        help="Fraction of the current monitor used for the fitted window",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    app = DashboardMockup(native_4k=args.native_4k, fit=args.fit, profile_path=args.profile)
    try:
        return app.run()
    finally:
        pygame.quit()


if __name__ == "__main__":
    sys.exit(main())