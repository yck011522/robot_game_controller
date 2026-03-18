#!/usr/bin/env python3
"""Standalone Tkinter viewer for robot game state broadcast.

Copy this single file to any machine on the same network and run:

    python view_game_state.py

No dependencies beyond the Python standard library (tkinter is included
with all standard Python distributions).

Press the window close button or Ctrl-C in the terminal to exit.
"""

import json
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PORT = 9000  # must match GameSettings.broadcast_port
STALE_S = 1.0  # seconds without a packet before "NO SIGNAL" is shown
UI_HZ = 30  # UI refresh rate
# ---------------------------------------------------------------------------

_MOTOR_IDS = ["11", "12", "13", "14", "15", "16"]
_JOINT_LABELS = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
_TEAM1_BUCKETS = ["11", "12", "13"]
_TEAM2_BUCKETS = ["21", "22", "23"]
_BUCKET_LABELS = {
    "11": "B1 ×1",
    "12": "B2 ×2",
    "13": "B3 ×3",
    "21": "B1 ×1",
    "22": "B2 ×2",
    "23": "B3 ×3",
}
_SLIDER_MIN = -180
_SLIDER_MAX = 180
_UI_MS = round(1000 / UI_HZ)


# ---------------------------------------------------------------------------
# Receiver thread
# ---------------------------------------------------------------------------


class _Receiver:
    """Background UDP listener.  Thread-safe via a simple lock."""

    def __init__(self, port: int):
        self._port = port
        self._lock = threading.Lock()
        self._state: dict = {}
        self._last_rx: float = 0.0
        self._rx_count: int = 0
        self._hz_count: int = 0
        self._hz_window: float = time.time()
        self._rx_hz: float = 0.0
        self._error: str = ""
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def snapshot(self) -> tuple[dict, float, int, float, str]:
        """Return (state, last_rx_time, rx_count, rx_hz, error)."""
        with self._lock:
            return (
                dict(self._state),
                self._last_rx,
                self._rx_count,
                self._rx_hz,
                self._error,
            )

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("", self._port))
        except OSError as e:
            with self._lock:
                self._error = str(e)
            return
        sock.settimeout(0.5)

        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue

            try:
                packet = json.loads(data.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

            now = time.time()
            with self._lock:
                self._state = packet
                self._last_rx = now
                self._rx_count += 1
                self._hz_count += 1
                elapsed = now - self._hz_window
                if elapsed >= 1.0:
                    self._rx_hz = self._hz_count / elapsed
                    self._hz_count = 0
                    self._hz_window = now


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------


class GameStateViewer:

    def __init__(self, receiver: _Receiver):
        self._rx = receiver

        self._root = tk.Tk()
        self._root.title("Game State Viewer")
        self._root.configure(bg="#1e1e1e")
        self._root.resizable(True, True)

        # Styles
        style = ttk.Style()
        style.theme_use("clam")
        bg = "#1e1e1e"
        panel = "#252526"
        fg = "#d4d4d4"
        accent = "#007acc"
        green = "#4ec9b0"
        red = "#f44747"

        style.configure("TFrame", background=panel)
        style.configure("TLabel", background=panel, foreground=fg, font=("Consolas", 9))
        style.configure(
            "Title.TLabel",
            background=panel,
            foreground="#ffffff",
            font=("Consolas", 11, "bold"),
        )
        style.configure(
            "Value.TLabel", background=panel, foreground=accent, font=("Consolas", 10)
        )
        style.configure(
            "Stage.TLabel",
            background=panel,
            foreground=green,
            font=("Consolas", 16, "bold"),
        )
        style.configure(
            "Estop.TLabel",
            background=panel,
            foreground=red,
            font=("Consolas", 12, "bold"),
        )
        style.configure("TLabelframe", background=panel, foreground=fg)
        style.configure(
            "TLabelframe.Label",
            background=panel,
            foreground=fg,
            font=("Consolas", 10, "bold"),
        )
        style.configure(
            "TScale",
            background=panel,
            troughcolor="#333333",  # darker track for contrast
        )
        style.map(
            "TScale",
            background=[("disabled", "#5a5a5a")],  # visible thumb even when disabled
            troughcolor=[("disabled", "#333333")],
        )

        self._build_ui()
        self._root.after(_UI_MS, self._update_loop)

    def run(self):
        self._root.mainloop()

    # --- Layout -----------------------------------------------------------

    def _build_ui(self):
        root = self._root
        root.columnconfigure(0, weight=3)
        root.columnconfigure(1, weight=1)
        root.columnconfigure(2, weight=1)
        root.rowconfigure(0, weight=1)

        self._build_joint_panel(root)
        self._build_state_panel(root)
        self._build_score_panel(root)

    def _build_joint_panel(self, root):
        frame = ttk.LabelFrame(root, text="Joint Positions", padding=5)
        frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self._robot_vars: dict[str, tk.DoubleVar] = {}
        self._dial_vars: dict[str, tk.DoubleVar] = {}
        self._robot_lbls: dict[str, ttk.Label] = {}
        self._dial_lbls: dict[str, ttk.Label] = {}

        for i, (mid, label) in enumerate(zip(_MOTOR_IDS, _JOINT_LABELS)):
            frame.columnconfigure(i, weight=1, uniform="joints")

            col = ttk.Frame(frame)
            col.grid(row=0, column=i, sticky="nsew", padx=3, pady=2)
            col.rowconfigure(2, weight=1)

            # Header
            ttk.Label(col, text=f"J{i+1}", style="Title.TLabel", anchor="center").grid(
                row=0, column=0, columnspan=2, sticky="ew"
            )

            # Column sub-headers
            ttk.Label(col, text="Rob", anchor="center").grid(
                row=1, column=0, sticky="ew"
            )
            ttk.Label(col, text="Dial", anchor="center").grid(
                row=1, column=1, sticky="ew"
            )

            # Robot slider (display only)
            robot_var = tk.DoubleVar(value=0.0)
            robot_scale = ttk.Scale(
                col,
                from_=_SLIDER_MAX,
                to=_SLIDER_MIN,
                orient="vertical",
                variable=robot_var,
                length=220,
            )
            robot_scale.grid(row=2, column=0, sticky="ns", padx=2)
            robot_scale.state(["disabled"])
            self._robot_vars[mid] = robot_var

            # Dial slider (display only)
            dial_var = tk.DoubleVar(value=0.0)
            dial_scale = ttk.Scale(
                col,
                from_=_SLIDER_MAX,
                to=_SLIDER_MIN,
                orient="vertical",
                variable=dial_var,
                length=220,
            )
            dial_scale.grid(row=2, column=1, sticky="ns", padx=2)
            dial_scale.state(["disabled"])
            self._dial_vars[mid] = dial_var

            # Numeric readouts — fixed width=7 prevents layout jitter when
            # value changes between e.g. "0.0°" and "+180.0°"
            r_lbl = ttk.Label(
                col, text="0.0°", style="Value.TLabel", anchor="center", width=7
            )
            r_lbl.grid(row=3, column=0, sticky="ew")
            self._robot_lbls[mid] = r_lbl

            d_lbl = ttk.Label(
                col, text="0.0°", style="Value.TLabel", anchor="center", width=7
            )
            d_lbl.grid(row=3, column=1, sticky="ew")
            self._dial_lbls[mid] = d_lbl

            # Joint label + motor ID
            ttk.Label(col, text=f"M{mid}", anchor="center").grid(
                row=4, column=0, columnspan=2, sticky="ew"
            )
            ttk.Label(col, text=label, anchor="center").grid(
                row=5, column=0, columnspan=2, sticky="ew"
            )

    def _build_state_panel(self, root):
        frame = ttk.Frame(root, padding=5)
        frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        frame.columnconfigure(0, weight=1)

        # --- Connection status ---
        conn_frame = ttk.LabelFrame(frame, text="Connection", padding=5)
        conn_frame.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        conn_frame.columnconfigure(1, weight=1)

        ttk.Label(conn_frame, text="Port").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Label(conn_frame, text=str(PORT), style="Value.TLabel", anchor="e").grid(
            row=0, column=1, sticky="e", padx=5
        )

        ttk.Label(conn_frame, text="Packets").grid(row=1, column=0, sticky="w", padx=5)
        self._pkt_lbl = ttk.Label(
            conn_frame, text="0", style="Value.TLabel", anchor="e"
        )
        self._pkt_lbl.grid(row=1, column=1, sticky="e", padx=5)

        ttk.Label(conn_frame, text="Recv Hz").grid(row=2, column=0, sticky="w", padx=5)
        self._rx_hz_lbl = ttk.Label(
            conn_frame, text="0.0 Hz", style="Value.TLabel", anchor="e"
        )
        self._rx_hz_lbl.grid(row=2, column=1, sticky="e", padx=5)

        ttk.Label(conn_frame, text="Lag").grid(row=3, column=0, sticky="w", padx=5)
        self._lag_lbl = ttk.Label(
            conn_frame, text="-- ms", style="Value.TLabel", anchor="e"
        )
        self._lag_lbl.grid(row=3, column=1, sticky="e", padx=5)

        ttk.Label(conn_frame, text="Signal").grid(row=4, column=0, sticky="w", padx=5)
        self._signal_lbl = ttk.Label(
            conn_frame, text="Waiting...", style="Value.TLabel", anchor="e"
        )
        self._signal_lbl.grid(row=4, column=1, sticky="e", padx=5)

        # --- Game stage ---
        stage_frame = ttk.LabelFrame(frame, text="Game Stage", padding=5)
        stage_frame.grid(row=1, column=0, sticky="ew", pady=(0, 5))
        stage_frame.columnconfigure(0, weight=1)

        self._stage_lbl = ttk.Label(
            stage_frame, text="—", style="Stage.TLabel", anchor="center"
        )
        self._stage_lbl.grid(row=0, column=0, sticky="ew", pady=2)

        self._countdown_lbl = ttk.Label(stage_frame, text="", anchor="center")
        self._countdown_lbl.grid(row=1, column=0, sticky="ew")

        self._estop_lbl = ttk.Label(
            stage_frame, text="", style="Estop.TLabel", anchor="center"
        )
        self._estop_lbl.grid(row=2, column=0, sticky="ew")

        # --- System health ---
        health_frame = ttk.LabelFrame(frame, text="System Health", padding=5)
        health_frame.grid(row=2, column=0, sticky="ew", pady=(0, 5))
        health_frame.columnconfigure(1, weight=1)

        health_rows = [
            ("game_loop_hz", "Game Loop"),
            ("robot_physics_hz", "Robot Physics"),
            ("publisher_hz", "Publisher"),
            ("haptic_connected", "Haptic"),
            ("weight_sensors", "Weight Sens."),
        ]
        self._health_lbls: dict[str, ttk.Label] = {}
        for i, (key, label) in enumerate(health_rows):
            ttk.Label(health_frame, text=label).grid(
                row=i, column=0, sticky="w", padx=5
            )
            lbl = ttk.Label(health_frame, text="—", style="Value.TLabel", anchor="e")
            lbl.grid(row=i, column=1, sticky="e", padx=5)
            self._health_lbls[key] = lbl

    def _build_score_panel(self, root):
        frame = ttk.Frame(root, padding=5)
        frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        frame.columnconfigure(0, weight=1)

        # --- Scores ---
        score_frame = ttk.LabelFrame(frame, text="Scores", padding=5)
        score_frame.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        score_frame.columnconfigure(1, weight=1)

        ttk.Label(score_frame, text="Team 1", style="Title.TLabel").grid(
            row=0, column=0, sticky="w", padx=5
        )
        self._score1_lbl = ttk.Label(
            score_frame, text="0.0", style="Value.TLabel", anchor="e"
        )
        self._score1_lbl.grid(row=0, column=1, sticky="e", padx=5)

        ttk.Label(score_frame, text="Team 2", style="Title.TLabel").grid(
            row=1, column=0, sticky="w", padx=5
        )
        self._score2_lbl = ttk.Label(
            score_frame, text="0.0", style="Value.TLabel", anchor="e"
        )
        self._score2_lbl.grid(row=1, column=1, sticky="e", padx=5)

        ttk.Separator(score_frame, orient="horizontal").grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=3
        )

        ttk.Label(score_frame, text="High Score").grid(
            row=3, column=0, sticky="w", padx=5
        )
        self._high_lbl = ttk.Label(
            score_frame, text="0.0", style="Value.TLabel", anchor="e"
        )
        self._high_lbl.grid(row=3, column=1, sticky="e", padx=5)

        ttk.Label(score_frame, text="Holder").grid(row=4, column=0, sticky="w", padx=5)
        self._holder_lbl = ttk.Label(
            score_frame, text="—", style="Value.TLabel", anchor="e"
        )
        self._holder_lbl.grid(row=4, column=1, sticky="e", padx=5)

        # --- Buckets ---
        bucket_frame = ttk.LabelFrame(frame, text="Bucket Weights (g)", padding=5)
        bucket_frame.grid(row=1, column=0, sticky="ew", pady=(0, 5))
        bucket_frame.columnconfigure(1, weight=1)

        self._bucket_lbls: dict[str, ttk.Label] = {}
        r = 0
        for team, ids in [("Team 1", _TEAM1_BUCKETS), ("Team 2", _TEAM2_BUCKETS)]:
            ttk.Label(bucket_frame, text=team, style="Title.TLabel").grid(
                row=r, column=0, columnspan=2, sticky="w", padx=5
            )
            r += 1
            for bid in ids:
                ttk.Label(bucket_frame, text=f"  {_BUCKET_LABELS[bid]}").grid(
                    row=r, column=0, sticky="w", padx=5
                )
                lbl = ttk.Label(
                    bucket_frame, text="0 g", style="Value.TLabel", anchor="e"
                )
                lbl.grid(row=r, column=1, sticky="e", padx=5)
                self._bucket_lbls[bid] = lbl
                r += 1

        # --- Protocol info ---
        info_frame = ttk.LabelFrame(frame, text="Protocol", padding=5)
        info_frame.grid(row=2, column=0, sticky="ew")
        info_frame.columnconfigure(1, weight=1)

        ttk.Label(info_frame, text="Version").grid(row=0, column=0, sticky="w", padx=5)
        self._proto_lbl = ttk.Label(
            info_frame, text="—", style="Value.TLabel", anchor="e"
        )
        self._proto_lbl.grid(row=0, column=1, sticky="e", padx=5)

    # --- Update loop ------------------------------------------------------

    def _update_loop(self):
        state, last_rx, rx_count, rx_hz, error = self._rx.snapshot()
        age = time.time() - last_rx
        fresh = (last_rx > 0) and (age < STALE_S)

        # Connection panel
        self._pkt_lbl.configure(text=str(rx_count))
        self._rx_hz_lbl.configure(text=f"{rx_hz:.1f} Hz")

        if error:
            self._signal_lbl.configure(text=f"ERROR: {error}", foreground="#f44747")
        elif not fresh:
            self._signal_lbl.configure(text="NO SIGNAL", foreground="#f44747")
            self._lag_lbl.configure(text="-- ms")
        else:
            self._signal_lbl.configure(text="OK", foreground="#4ec9b0")
            lag_ms = (time.time() - state.get("ts", time.time())) * 1000
            self._lag_lbl.configure(text=f"{lag_ms:.1f} ms")

        if fresh:
            self._update_joints(state)
            self._update_stage(state)
            self._update_health(state)
            self._update_scores(state)
        else:
            # Dim sliders to zero when signal is stale
            for mid in _MOTOR_IDS:
                self._robot_vars[mid].set(0.0)
                self._dial_vars[mid].set(0.0)
                self._robot_lbls[mid].configure(text="—")
                self._dial_lbls[mid].configure(text="—")
            self._stage_lbl.configure(text="—")
            self._countdown_lbl.configure(text="")
            self._estop_lbl.configure(text="")

        self._root.after(_UI_MS, self._update_loop)

    def _update_joints(self, state: dict):
        joints = state.get("joints", {})
        for mid in _MOTOR_IDS:
            j = joints.get(mid, {})
            r = j.get("robot_deg", 0.0)
            d = j.get("dial_deg", 0.0)
            self._robot_vars[mid].set(max(_SLIDER_MIN, min(_SLIDER_MAX, r)))
            self._dial_vars[mid].set(max(_SLIDER_MIN, min(_SLIDER_MAX, d)))
            self._robot_lbls[mid].configure(text=f"{r:+.1f}°")
            self._dial_lbls[mid].configure(text=f"{d:+.1f}°")

    def _update_stage(self, state: dict):
        stage = state.get("stage", "—")
        countdown = state.get("countdown_s", 0)
        estop = state.get("estop", False)

        self._stage_lbl.configure(text=stage)
        if countdown > 0:
            m, s = divmod(countdown, 60)
            self._countdown_lbl.configure(text=f"{m}:{s:02d} remaining")
        else:
            self._countdown_lbl.configure(text="")
        self._estop_lbl.configure(text="*** EMERGENCY STOP ***" if estop else "")

        self._proto_lbl.configure(text=str(state.get("v", "—")))

    def _update_health(self, state: dict):
        h = state.get("health", {})
        for key, lbl in self._health_lbls.items():
            val = h.get(key)
            if val is None:
                lbl.configure(text="—")
            elif isinstance(val, float):
                lbl.configure(text=f"{val:.1f} Hz")
            else:
                lbl.configure(text=str(val))

    def _update_scores(self, state: dict):
        scores = state.get("scores", {})
        self._score1_lbl.configure(text=f"{scores.get('team1', 0.0):.1f}")
        self._score2_lbl.configure(text=f"{scores.get('team2', 0.0):.1f}")
        self._high_lbl.configure(text=f"{scores.get('high', 0.0):.1f}")
        self._holder_lbl.configure(text=scores.get("high_holder", "—") or "—")

        buckets = state.get("buckets", {})
        mults = state.get("multipliers", {})
        for bid, lbl in self._bucket_lbls.items():
            w = buckets.get(bid, 0.0)
            m = mults.get(bid, 1.0)
            lbl.configure(text=f"{w:.0f} g  (={w*m:.0f})")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    rx = _Receiver(PORT)
    rx.start()

    viewer = GameStateViewer(rx)
    viewer.run()


if __name__ == "__main__":
    main()
