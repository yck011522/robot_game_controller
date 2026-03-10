"""Game Master UI — Tkinter-based monitoring and control panel.

Runs on the main thread (Tkinter requirement). Reads observable state from
GameSettings and displays it. Writes control inputs (stage overrides, timing,
haptic parameters) back into GameSettings for the GameController to pick up.

Layout zones:
  Left   — Joint visualizer (vertical sliders: robot pos vs dial/clamped pos)
  Center — Game state, timing, frequencies
  Right  — Haptic & control parameters, scoring
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional

from game_settings import GameSettings

# UI refresh rate
_UI_UPDATE_MS = 20  # 50 Hz

# Joint display range (degrees)
_SLIDER_MIN = -180
_SLIDER_MAX = 180

# Motor IDs
_MOTOR_IDS = [11, 12, 13, 14, 15, 16]
_JOINT_LABELS = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]

# Bucket IDs and labels
_TEAM1_BUCKET_IDS = [11, 12, 13]
_TEAM2_BUCKET_IDS = [21, 22, 23]
_ALL_BUCKET_IDS = _TEAM1_BUCKET_IDS + _TEAM2_BUCKET_IDS
_BUCKET_LABELS = {
    11: "T1-B1",
    12: "T1-B2",
    13: "T1-B3",
    21: "T2-B1",
    22: "T2-B2",
    23: "T2-B3",
}

# Game stages
_STAGES = ["Idle", "Tutorial", "GameOn", "Conclusion", "Reset"]


class GameMasterUI:
    """Tkinter Game Master control panel."""

    def __init__(self, settings: GameSettings):
        self._settings = settings

        self._root = tk.Tk()
        sim = settings.get("simulate_mode")
        title = "Game Master — Robot Game Controller"
        if sim:
            title += "  [SIMULATE MODE]"
        self._root.title(title)
        self._root.geometry("1280x720")
        self._root.configure(bg="#1e1e1e")

        # Style
        self._style = ttk.Style()
        self._style.theme_use("clam")
        self._configure_styles()

        # Build layout
        self._build_ui()

        # Start update loop
        self._root.after(_UI_UPDATE_MS, self._update_loop)

    def run(self):
        """Start the Tkinter main loop (blocking — call on main thread)."""
        self._root.mainloop()

    def request_stop(self):
        """Request the UI to close (can be called from any thread)."""
        self._root.after(0, self._root.destroy)

    # --- Style ------------------------------------------------------------

    def _configure_styles(self):
        bg = "#1e1e1e"
        fg = "#d4d4d4"
        accent = "#007acc"
        panel_bg = "#252526"

        self._style.configure("TFrame", background=panel_bg)
        self._style.configure(
            "TLabel", background=panel_bg, foreground=fg, font=("Consolas", 9)
        )
        self._style.configure(
            "Title.TLabel",
            background=panel_bg,
            foreground="#ffffff",
            font=("Consolas", 11, "bold"),
        )
        self._style.configure(
            "Value.TLabel",
            background=panel_bg,
            foreground=accent,
            font=("Consolas", 10),
        )
        self._style.configure(
            "Stage.TLabel",
            background=panel_bg,
            foreground="#4ec9b0",
            font=("Consolas", 14, "bold"),
        )
        self._style.configure(
            "Emergency.TButton",
            foreground="#ffffff",
            background="#cc0000",
            font=("Consolas", 10, "bold"),
        )
        self._style.configure("TButton", font=("Consolas", 9))
        self._style.configure("TLabelframe", background=panel_bg, foreground=fg)
        self._style.configure(
            "TLabelframe.Label",
            background=panel_bg,
            foreground=fg,
            font=("Consolas", 10, "bold"),
        )
        self._style.configure("TScale", background=panel_bg)
        self._style.configure("TSpinbox", font=("Consolas", 9))

    # --- Layout -----------------------------------------------------------

    def _build_ui(self):
        # Main container with 3 columns (+ optional 4th for simulator)
        self._root.columnconfigure(0, weight=3)  # Joint visualizer
        self._root.columnconfigure(1, weight=2)  # Game state & frequencies
        self._root.columnconfigure(2, weight=2)  # Parameters & scoring
        self._root.rowconfigure(0, weight=1)

        self._build_joint_panel()
        self._build_state_panel()
        self._build_params_panel()

        # Add simulator panel if in simulate mode
        if self._settings.get("simulate_mode"):
            self._root.columnconfigure(3, weight=2)
            self._root.geometry("1600x720")
            self._build_simulator_column()

    # --- Left panel: Joint Visualizer -------------------------------------

    def _build_joint_panel(self):
        frame = ttk.LabelFrame(self._root, text="Joint Positions", padding=5)
        frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # One column per joint, each with two vertical scales + labels
        self._robot_scales = {}
        self._dial_scales = {}
        self._robot_labels = {}
        self._dial_labels = {}

        for i, (mid, label) in enumerate(zip(_MOTOR_IDS, _JOINT_LABELS)):
            frame.columnconfigure(i, weight=1)

            joint_frame = ttk.Frame(frame)
            joint_frame.grid(row=0, column=i, sticky="nsew", padx=3, pady=2)
            joint_frame.rowconfigure(1, weight=1)

            # Header
            ttk.Label(
                joint_frame, text=f"J{i+1}", style="Title.TLabel", anchor="center"
            ).grid(row=0, column=0, columnspan=2, sticky="ew")
            ttk.Label(joint_frame, text=label, anchor="center").grid(
                row=5, column=0, columnspan=2, sticky="ew"
            )

            # Column headers
            ttk.Label(joint_frame, text="Rob", anchor="center").grid(
                row=1, column=0, sticky="ew"
            )
            ttk.Label(joint_frame, text="Dial", anchor="center").grid(
                row=1, column=1, sticky="ew"
            )

            # Robot position scale (read-only visual)
            robot_var = tk.DoubleVar(value=0.0)
            robot_scale = ttk.Scale(
                joint_frame,
                from_=_SLIDER_MAX,
                to=_SLIDER_MIN,
                orient="vertical",
                variable=robot_var,
                length=250,
            )
            robot_scale.grid(row=2, column=0, sticky="ns", padx=2)
            robot_scale.state(["disabled"])
            self._robot_scales[mid] = robot_var

            # Dial/clamped position scale (read-only visual)
            dial_var = tk.DoubleVar(value=0.0)
            dial_scale = ttk.Scale(
                joint_frame,
                from_=_SLIDER_MAX,
                to=_SLIDER_MIN,
                orient="vertical",
                variable=dial_var,
                length=250,
            )
            dial_scale.grid(row=2, column=1, sticky="ns", padx=2)
            dial_scale.state(["disabled"])
            self._dial_scales[mid] = dial_var

            # Numeric readouts
            robot_lbl = ttk.Label(
                joint_frame, text="0.0°", style="Value.TLabel", anchor="center"
            )
            robot_lbl.grid(row=3, column=0, sticky="ew")
            self._robot_labels[mid] = robot_lbl

            dial_lbl = ttk.Label(
                joint_frame, text="0.0°", style="Value.TLabel", anchor="center"
            )
            dial_lbl.grid(row=3, column=1, sticky="ew")
            self._dial_labels[mid] = dial_lbl

            # Motor ID
            ttk.Label(joint_frame, text=f"M{mid}", anchor="center").grid(
                row=4, column=0, columnspan=2, sticky="ew"
            )

    # --- Simulator column: All simulation controls in one column -----------

    def _build_simulator_column(self):
        """Build column 3 with haptic sim sliders on top and weight sim below."""
        outer = ttk.Frame(self._root, padding=0)
        outer.grid(row=0, column=3, sticky="nsew", padx=5, pady=5)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=3)  # haptic sim gets more space
        outer.rowconfigure(1, weight=2)  # weight sim

        self._build_haptic_sim(outer)
        self._build_weight_sim(outer)

    def _build_haptic_sim(self, parent):
        """Haptic dial simulator — 6 vertical sliders."""
        frame = ttk.LabelFrame(parent, text="Haptic Simulator", padding=5)
        frame.grid(row=0, column=0, sticky="nsew", pady=(0, 3))

        self._sim_vars = {}
        self._sim_labels = {}

        for i, (mid, label) in enumerate(zip(_MOTOR_IDS, _JOINT_LABELS)):
            frame.columnconfigure(i, weight=1)

            col_frame = ttk.Frame(frame)
            col_frame.grid(row=0, column=i, sticky="nsew", padx=3, pady=2)
            col_frame.rowconfigure(2, weight=1)

            ttk.Label(
                col_frame, text=f"J{i+1}", style="Title.TLabel", anchor="center"
            ).grid(row=0, column=0, sticky="ew")
            ttk.Label(col_frame, text=label, anchor="center").grid(
                row=5, column=0, sticky="ew"
            )

            var = tk.DoubleVar(value=0.0)
            self._sim_vars[mid] = var

            scale = ttk.Scale(
                col_frame,
                from_=_SLIDER_MAX,
                to=_SLIDER_MIN,
                orient="vertical",
                variable=var,
                length=200,
                command=lambda val, m=mid: self._on_sim_slider(m, float(val)),
            )
            scale.grid(row=2, column=0, sticky="ns", padx=2)

            lbl = ttk.Label(
                col_frame, text="0.0°", style="Value.TLabel", anchor="center"
            )
            lbl.grid(row=3, column=0, sticky="ew")
            self._sim_labels[mid] = lbl

            ttk.Label(col_frame, text=f"M{mid}", anchor="center").grid(
                row=4, column=0, sticky="ew"
            )

        reset_btn = ttk.Button(
            frame, text="Reset All to 0°", command=self._reset_sim_sliders
        )
        reset_btn.grid(
            row=1, column=0, columnspan=len(_MOTOR_IDS), sticky="ew", pady=(5, 0)
        )

    def _build_weight_sim(self, parent):
        """Weight sensor simulator — 6 vertical sliders mirroring the haptic sim above."""
        frame = ttk.LabelFrame(parent, text="Weight Sensor Simulator", padding=5)
        frame.grid(row=1, column=0, sticky="nsew", pady=(3, 0))

        self._weight_sim_vars = {}
        self._weight_sim_labels = {}

        multipliers = self._settings.get("bucket_multipliers")

        for i, bid in enumerate(_ALL_BUCKET_IDS):
            frame.columnconfigure(i, weight=1)

            col_frame = ttk.Frame(frame)
            col_frame.grid(row=0, column=i, sticky="nsew", padx=3, pady=2)
            col_frame.rowconfigure(1, weight=1)

            # Team header above first/fourth bucket
            team_text = "T1" if bid < 20 else "T2"
            ttk.Label(
                col_frame, text=team_text, style="Title.TLabel", anchor="center"
            ).grid(row=0, column=0, sticky="ew")

            var = tk.DoubleVar(value=0.0)
            self._weight_sim_vars[bid] = var

            scale = ttk.Scale(
                col_frame,
                from_=500,
                to=0,
                orient="vertical",
                variable=var,
                length=120,
                command=lambda val, b=bid: self._on_weight_sim_slider(b, float(val)),
            )
            scale.grid(row=1, column=0, sticky="ns", padx=2)

            # Bucket label + multiplier
            mult = multipliers.get(bid, 1.0)
            label = _BUCKET_LABELS[bid]
            ttk.Label(col_frame, text=f"{label}", anchor="center").grid(
                row=2, column=0, sticky="ew"
            )
            ttk.Label(col_frame, text=f"×{mult:.0f}", anchor="center").grid(
                row=3, column=0, sticky="ew"
            )

            # Value readout
            lbl = ttk.Label(
                col_frame, text="0 g", style="Value.TLabel", anchor="center"
            )
            lbl.grid(row=4, column=0, sticky="ew")
            self._weight_sim_labels[bid] = lbl

        reset_btn = ttk.Button(
            frame, text="Reset All to 0g", command=self._reset_weight_sim_sliders
        )
        reset_btn.grid(
            row=1, column=0, columnspan=len(_ALL_BUCKET_IDS), sticky="ew", pady=(5, 0)
        )

    def _on_sim_slider(self, motor_id: int, value: float):
        """Called when a simulator slider is dragged."""
        # Update the label
        self._sim_labels[motor_id].configure(text=f"{value:+.1f}°")
        # Write to shared settings register
        sim_angles = self._settings.get("sim_dial_angles")
        sim_angles[motor_id] = value
        self._settings.set("sim_dial_angles", sim_angles)

    def _reset_sim_sliders(self):
        """Reset all simulator sliders to zero."""
        for mid in _MOTOR_IDS:
            self._sim_vars[mid].set(0.0)
            self._sim_labels[mid].configure(text="0.0°")
        self._settings.set("sim_dial_angles", {mid: 0.0 for mid in _MOTOR_IDS})

    def _on_weight_sim_slider(self, bucket_id: int, value: float):
        """Called when a weight simulator slider is dragged."""
        self._weight_sim_labels[bucket_id].configure(text=f"{value:.0f} g")
        sim_weights = self._settings.get("sim_bucket_weights")
        sim_weights[bucket_id] = value
        self._settings.set("sim_bucket_weights", sim_weights)

    def _reset_weight_sim_sliders(self):
        """Reset all weight simulator sliders to zero."""
        for bid in _ALL_BUCKET_IDS:
            self._weight_sim_vars[bid].set(0.0)
            self._weight_sim_labels[bid].configure(text="0 g")
        self._settings.set("sim_bucket_weights", {bid: 0.0 for bid in _ALL_BUCKET_IDS})

    # --- Center panel: Game State & Frequencies ---------------------------

    def _build_state_panel(self):
        frame = ttk.Frame(self._root, padding=5)
        frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        frame.columnconfigure(0, weight=1)

        row = 0

        # --- Game Stage ---
        stage_frame = ttk.LabelFrame(frame, text="Game Stage", padding=5)
        stage_frame.grid(row=row, column=0, sticky="ew", pady=(0, 5))
        stage_frame.columnconfigure(0, weight=1)
        row += 1

        self._stage_label = ttk.Label(
            stage_frame, text="Idle", style="Stage.TLabel", anchor="center"
        )
        self._stage_label.grid(row=0, column=0, sticky="ew")

        self._countdown_label = ttk.Label(stage_frame, text="", anchor="center")
        self._countdown_label.grid(row=1, column=0, sticky="ew")

        # Stage override buttons
        btn_frame = ttk.Frame(stage_frame)
        btn_frame.grid(row=2, column=0, sticky="ew", pady=5)
        for i, stage in enumerate(_STAGES):
            btn = ttk.Button(
                btn_frame,
                text=stage,
                width=10,
                command=lambda s=stage: self._override_stage(s),
            )
            btn.grid(row=0, column=i, padx=1)

        # Auto-cycle toggle
        self._auto_cycle_var = tk.BooleanVar(value=self._settings.get("auto_cycle"))
        auto_cb = ttk.Checkbutton(
            stage_frame,
            text="Auto-cycle",
            variable=self._auto_cycle_var,
            command=self._on_auto_cycle_toggle,
        )
        auto_cb.grid(row=3, column=0, sticky="w")

        # Emergency stop
        self._estop_btn = tk.Button(
            stage_frame,
            text="EMERGENCY STOP",
            bg="#cc0000",
            fg="white",
            font=("Consolas", 12, "bold"),
            height=2,
            command=self._toggle_estop,
        )
        self._estop_btn.grid(row=4, column=0, sticky="ew", pady=5)

        # --- Timing ---
        timing_frame = ttk.LabelFrame(frame, text="Timing (seconds)", padding=5)
        timing_frame.grid(row=row, column=0, sticky="ew", pady=(0, 5))
        row += 1

        self._timing_vars = {}
        timing_fields = [
            ("game_duration_s", "Game Duration"),
            ("tutorial_duration_s", "Tutorial Duration"),
            ("conclusion_duration_s", "Conclusion Duration"),
            ("reset_duration_s", "Reset Duration"),
        ]
        for i, (key, label) in enumerate(timing_fields):
            ttk.Label(timing_frame, text=label).grid(
                row=i, column=0, sticky="w", padx=5
            )
            var = tk.IntVar(value=self._settings.get(key))
            spinbox = ttk.Spinbox(
                timing_frame,
                from_=5,
                to=600,
                width=6,
                textvariable=var,
                command=lambda k=key, v=var: self._settings.set(k, v.get()),
            )
            spinbox.grid(row=i, column=1, sticky="e", padx=5)
            self._timing_vars[key] = var

        # --- Frequencies ---
        freq_frame = ttk.LabelFrame(frame, text="System Health", padding=5)
        freq_frame.grid(row=row, column=0, sticky="ew", pady=(0, 5))
        row += 1

        self._freq_labels = {}
        freq_fields = [
            ("game_loop_hz", "Game Loop"),
            ("robot_physics_hz", "Robot Physics"),
            ("weight_sensor_hz", "Weight Sensor"),
        ]
        for i, (key, label) in enumerate(freq_fields):
            ttk.Label(freq_frame, text=label).grid(row=i, column=0, sticky="w", padx=5)
            lbl = ttk.Label(freq_frame, text="0.0 Hz", style="Value.TLabel", anchor="e")
            lbl.grid(row=i, column=1, sticky="e", padx=5)
            self._freq_labels[key] = lbl

        # FOC Hz per motor (dynamic rows)
        self._foc_label_start_row = len(freq_fields)
        self._foc_labels = {}
        for i, mid in enumerate(_MOTOR_IDS):
            r = self._foc_label_start_row + i
            ttk.Label(freq_frame, text=f"FOC M{mid}").grid(
                row=r, column=0, sticky="w", padx=5
            )
            lbl = ttk.Label(freq_frame, text="-- Hz", style="Value.TLabel", anchor="e")
            lbl.grid(row=r, column=1, sticky="e", padx=5)
            self._foc_labels[mid] = lbl

        # Connection status
        conn_row = self._foc_label_start_row + len(_MOTOR_IDS)
        ttk.Label(freq_frame, text="Haptic Connected").grid(
            row=conn_row, column=0, sticky="w", padx=5
        )
        self._conn_label = ttk.Label(
            freq_frame, text="--", style="Value.TLabel", anchor="e"
        )
        self._conn_label.grid(row=conn_row, column=1, sticky="e", padx=5)

        # Weight sensor connection status
        ws_row = conn_row + 1
        ttk.Label(freq_frame, text="Weight Sensors").grid(
            row=ws_row, column=0, sticky="w", padx=5
        )
        self._weight_conn_label = ttk.Label(
            freq_frame, text="--", style="Value.TLabel", anchor="e"
        )
        self._weight_conn_label.grid(row=ws_row, column=1, sticky="e", padx=5)

    # --- Right panel: Parameters & Scoring --------------------------------

    def _build_params_panel(self):
        frame = ttk.Frame(self._root, padding=5)
        frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        frame.columnconfigure(0, weight=1)

        row = 0

        # --- Haptic Parameters ---
        haptic_frame = ttk.LabelFrame(frame, text="Haptic Parameters", padding=5)
        haptic_frame.grid(row=row, column=0, sticky="ew", pady=(0, 5))
        row += 1

        self._haptic_vars = {}
        haptic_fields = [
            ("tracking_kp", "Tracking Kp", 0.0, 20.0, 0.1),
            ("tracking_kd", "Tracking Kd", 0.0, 5.0, 0.01),
            ("tracking_max_torque", "Track Max Torque (A)", 0.0, 5.0, 0.1),
            ("bounds_kp", "Bounds Kp", 0.0, 50.0, 0.5),
            ("oob_kick_amplitude", "OOB Kick Amp", 0.0, 5.0, 0.1),
        ]
        for i, (key, label, lo, hi, step) in enumerate(haptic_fields):
            ttk.Label(haptic_frame, text=label).grid(
                row=i, column=0, sticky="w", padx=5
            )
            var = tk.DoubleVar(value=self._settings.get(key))
            spinbox = ttk.Spinbox(
                haptic_frame,
                from_=lo,
                to=hi,
                increment=step,
                width=8,
                textvariable=var,
                format="%.2f",
                command=lambda k=key, v=var: self._settings.set(k, v.get()),
            )
            spinbox.grid(row=i, column=1, sticky="e", padx=5)
            self._haptic_vars[key] = var

        # OOB kick toggle
        oob_row = len(haptic_fields)
        self._oob_var = tk.BooleanVar(value=self._settings.get("oob_kick_enabled"))
        oob_cb = ttk.Checkbutton(
            haptic_frame,
            text="OOB Kick Enabled",
            variable=self._oob_var,
            command=lambda: self._settings.set("oob_kick_enabled", self._oob_var.get()),
        )
        oob_cb.grid(row=oob_row, column=0, columnspan=2, sticky="w", padx=5)

        # --- Control Parameters ---
        ctrl_frame = ttk.LabelFrame(frame, text="Control Parameters", padding=5)
        ctrl_frame.grid(row=row, column=0, sticky="ew", pady=(0, 5))
        row += 1

        self._ctrl_vars = {}
        ctrl_fields = [
            ("gear_ratio", "Gear Ratio", 1.0, 50.0, 0.5),
            ("dial_max_velocity_dps", "Dial Rate Limit (°/s)", 0.5, 90.0, 0.5),
            ("robot_max_velocity_dps", "Robot Max Vel (°/s)", 1.0, 180.0, 1.0),
        ]
        for i, (key, label, lo, hi, step) in enumerate(ctrl_fields):
            ttk.Label(ctrl_frame, text=label).grid(row=i, column=0, sticky="w", padx=5)
            var = tk.DoubleVar(value=self._settings.get(key))
            spinbox = ttk.Spinbox(
                ctrl_frame,
                from_=lo,
                to=hi,
                increment=step,
                width=8,
                textvariable=var,
                format="%.1f",
                command=lambda k=key, v=var: self._settings.set(k, v.get()),
            )
            spinbox.grid(row=i, column=1, sticky="e", padx=5)
            self._ctrl_vars[key] = var

        # --- Scoring ---
        score_frame = ttk.LabelFrame(frame, text="Scoring", padding=5)
        score_frame.grid(row=row, column=0, sticky="ew", pady=(0, 5))
        row += 1

        r = 0
        ttk.Label(score_frame, text="Team 1", style="Title.TLabel").grid(
            row=r, column=0, sticky="w", padx=5
        )
        self._score1_label = ttk.Label(
            score_frame, text="0.0", style="Value.TLabel", anchor="e"
        )
        self._score1_label.grid(row=r, column=1, sticky="e", padx=5)
        r += 1

        # Team 1 bucket breakdown
        self._bucket_weight_labels = {}
        for bid in _TEAM1_BUCKET_IDS:
            mult = self._settings.get("bucket_multipliers").get(bid, 1.0)
            ttk.Label(score_frame, text=f"  {_BUCKET_LABELS[bid]} (×{mult:.0f})").grid(
                row=r, column=0, sticky="w", padx=10
            )
            lbl = ttk.Label(score_frame, text="0.0 g", style="Value.TLabel", anchor="e")
            lbl.grid(row=r, column=1, sticky="e", padx=5)
            self._bucket_weight_labels[bid] = lbl
            r += 1

        ttk.Label(score_frame, text="Team 2", style="Title.TLabel").grid(
            row=r, column=0, sticky="w", padx=5
        )
        self._score2_label = ttk.Label(
            score_frame, text="0.0", style="Value.TLabel", anchor="e"
        )
        self._score2_label.grid(row=r, column=1, sticky="e", padx=5)
        r += 1

        # Team 2 bucket breakdown
        for bid in _TEAM2_BUCKET_IDS:
            mult = self._settings.get("bucket_multipliers").get(bid, 1.0)
            ttk.Label(score_frame, text=f"  {_BUCKET_LABELS[bid]} (×{mult:.0f})").grid(
                row=r, column=0, sticky="w", padx=10
            )
            lbl = ttk.Label(score_frame, text="0.0 g", style="Value.TLabel", anchor="e")
            lbl.grid(row=r, column=1, sticky="e", padx=5)
            self._bucket_weight_labels[bid] = lbl
            r += 1

        ttk.Label(score_frame, text="High Score").grid(
            row=r, column=0, sticky="w", padx=5
        )
        self._high_score_label = ttk.Label(
            score_frame, text="0.0", style="Value.TLabel", anchor="e"
        )
        self._high_score_label.grid(row=r, column=1, sticky="e", padx=5)

        # --- Profiles (placeholder) ---
        profile_frame = ttk.LabelFrame(frame, text="Profiles (Coming Soon)", padding=5)
        profile_frame.grid(row=row, column=0, sticky="ew", pady=(0, 5))
        row += 1

        ttk.Label(profile_frame, text="Save/Load presets").grid(
            row=0, column=0, sticky="w", padx=5
        )
        ttk.Button(profile_frame, text="Save", state="disabled").grid(
            row=0, column=1, padx=2
        )
        ttk.Button(profile_frame, text="Load", state="disabled").grid(
            row=0, column=2, padx=2
        )

    # --- Callbacks --------------------------------------------------------

    def _override_stage(self, stage: str):
        self._settings.set("manual_override", stage)

    def _on_auto_cycle_toggle(self):
        self._settings.set("auto_cycle", self._auto_cycle_var.get())

    def _toggle_estop(self):
        current = self._settings.get("emergency_stop")
        new_val = not current
        self._settings.set("emergency_stop", new_val)
        if new_val:
            self._estop_btn.configure(
                bg="#ff4444", text="E-STOP ACTIVE (click to release)"
            )
        else:
            self._estop_btn.configure(bg="#cc0000", text="EMERGENCY STOP")

    # --- Update loop (50 Hz) ---------------------------------------------

    def _update_loop(self):
        """Pull state from GameSettings and refresh all UI elements."""
        try:
            snap = self._settings.snapshot()
            self._update_joints(snap)
            self._update_stage(snap)
            self._update_frequencies(snap)
            self._update_scores(snap)
        except Exception:
            pass  # Don't crash the UI on transient errors

        self._root.after(_UI_UPDATE_MS, self._update_loop)

    def _update_joints(self, snap: dict):
        robot_deg = snap.get("robot_actual_deg", {})
        clamped = snap.get("clamped_deg", {})

        for mid in _MOTOR_IDS:
            r_val = robot_deg.get(mid, 0.0)
            d_val = clamped.get(mid, 0.0)

            # Clamp for slider range
            r_clamped = max(_SLIDER_MIN, min(_SLIDER_MAX, r_val))
            d_clamped = max(_SLIDER_MIN, min(_SLIDER_MAX, d_val))

            self._robot_scales[mid].set(r_clamped)
            self._dial_scales[mid].set(d_clamped)

            self._robot_labels[mid].configure(text=f"{r_val:+.1f}°")
            self._dial_labels[mid].configure(text=f"{d_val:+.1f}°")

    def _update_stage(self, snap: dict):
        stage = snap.get("current_stage", "Idle")
        countdown = snap.get("stage_countdown_s", 0)
        estop = snap.get("emergency_stop", False)

        self._stage_label.configure(text=stage)

        if countdown > 0:
            mins, secs = divmod(countdown, 60)
            self._countdown_label.configure(text=f"Time remaining: {mins}:{secs:02d}")
        else:
            self._countdown_label.configure(text="")

        if estop:
            self._estop_btn.configure(
                bg="#ff4444", text="E-STOP ACTIVE (click to release)"
            )
        else:
            self._estop_btn.configure(bg="#cc0000", text="EMERGENCY STOP")

    def _update_frequencies(self, snap: dict):
        game_hz = snap.get("game_loop_hz", 0.0)
        robot_hz = snap.get("robot_physics_hz", 0.0)
        weight_hz = snap.get("weight_sensor_hz", 0.0)

        self._freq_labels["game_loop_hz"].configure(text=f"{game_hz:.1f} Hz")
        self._freq_labels["robot_physics_hz"].configure(text=f"{robot_hz:.1f} Hz")
        self._freq_labels["weight_sensor_hz"].configure(text=f"{weight_hz:.1f} Hz")

        foc = snap.get("foc_hz", {})
        for mid in _MOTOR_IDS:
            hz = foc.get(mid, 0.0)
            self._foc_labels[mid].configure(text=f"{hz:.0f} Hz" if hz > 0 else "-- Hz")

        conn = snap.get("haptic_connected_count", "--")
        self._conn_label.configure(text=str(conn))

        ws_conn = snap.get("weight_sensor_connected_count", "--")
        self._weight_conn_label.configure(text=str(ws_conn))

    def _update_scores(self, snap: dict):
        self._score1_label.configure(text=f"{snap.get('team1_score', 0.0):.1f}")
        self._score2_label.configure(text=f"{snap.get('team2_score', 0.0):.1f}")
        self._high_score_label.configure(text=f"{snap.get('high_score', 0.0):.1f}")

        # Per-bucket weights
        weights = snap.get("bucket_weights", {})
        for bid, lbl in self._bucket_weight_labels.items():
            w = weights.get(bid, 0.0)
            lbl.configure(text=f"{w:.0f} g")
