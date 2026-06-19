"""Browse easy and hard reset configurations in the curated PyBullet scene.

Typical commands
----------------

python tools/visualize_configs.py --case-set hard --case-index 1
python tools/visualize_configs.py --case-set easy --case-index 1
python tools/visualize_configs.py --case-set hard --case-index 1 --validation-log tools/free_motion_planner_validation_20260619_121415.json
python tools/visualize_configs.py --case-set hard --case-index 1 --scan-on-start
python tools/visualize_configs.py --list

PyBullet-window controls
------------------------

N: next case
P: previous case
E: switch to easy cases
H: switch to hard cases
S: show selected start configuration
G: show reset goal configuration
C: scan at 0.05 deg and show the first straight-line collision
A: animate the straight start-to-goal line
Q: quit
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
DEFAULT_EASY_DATASET = REPO_ROOT / "tools" / "free_motion_reset_starts.json"
DEFAULT_HARD_DATASET = REPO_ROOT / "tools" / "free_motion_reset_hard_starts.json"
RESET_POSE_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from subsystems.motion_planning import discretize_joint_line  # noqa: E402


@dataclass(frozen=True)
class CollisionScan:
    """Result of scanning one direct joint-space line."""

    collision: bool  # True when a sampled configuration collides.
    point_index: int  # Zero-based point index, or final index when clear.
    point_count: int  # Total number of samples on the direct line.
    q_rad: list[float]  # Collision configuration, or goal when clear.
    detail: str | None  # compas_fab collision pair description.


def _load_starts(path: Path) -> list[list[float]]:
    """Load degree configurations from one reusable dataset JSON."""
    if not path.exists():
        return []
    payload = json.loads(path.read_text(encoding="utf-8"))
    starts = payload.get("starts_deg") or []
    return [[float(v) for v in q[:6]] for q in starts if isinstance(q, list) and len(q) >= 6]


def _config_key(q_deg: list[float]) -> tuple[float, ...]:
    """Return the same 1e-8 degree identity used by case generation."""
    return tuple(round(float(v), 8) for v in q_deg[:6])


def _load_validation_rows(path: Path | None) -> dict[tuple[float, ...], dict]:
    """Index optional validation rows by their starting configuration."""
    if path is None or not path.exists():
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    indexed: dict[tuple[float, ...], dict] = {}
    for row in payload.get("rows") or []:
        start_deg = row.get("start_deg")
        if isinstance(start_deg, list) and len(start_deg) >= 6:
            indexed[_config_key(start_deg)] = row
    return indexed


def _deg_to_rad(q_deg: list[float]) -> list[float]:
    """Convert one six-axis configuration from degrees to radians."""
    return [math.radians(float(v)) for v in q_deg[:6]]


def _format_joints(q_deg: list[float]) -> str:
    """Format six joint values compactly for console and GUI overlays."""
    return "[" + ", ".join(f"{v:+.1f}" for v in q_deg[:6]) + "]"


class PyBulletConfigViewer:
    """Own one GUI PyBullet scene and teleport the robot between configs."""

    def __init__(self) -> None:
        """Load the shared RobotCell scene in a GUI client."""
        from compas_fab.backends.exceptions import CollisionCheckError
        from subsystems.robot.shared_compas_scene import make_planner

        client, planner, _robot_cell, rcs, _stats = make_planner(
            connection_type="gui",
            verbose=False,
        )
        self.client = client  # compas_fab client owning the GUI connection.
        self.planner = planner  # Planner used for teleport and collision checks.
        self.rcs = rcs  # Mutable RobotCellState displayed in the GUI.
        self.cfg = rcs.robot_configuration.copy()  # Reused joint-value carrier.
        self.collision_error = CollisionCheckError  # Expected collision exception type.
        self.hud_id = -1  # PyBullet debug-text id reused without flicker.

        import pybullet as pb

        self.pb = pb  # Raw GUI helpers for keyboard, camera, and HUD.
        self.client_id = client.client_id  # Explicit client id for all raw calls.
        self.pb.resetDebugVisualizerCamera(
            cameraDistance=2.8,
            cameraYaw=45.0,
            cameraPitch=-28.0,
            cameraTargetPosition=[0.0, 0.0, 0.65],
            physicsClientId=self.client_id,
        )

    def set_configuration(self, q_rad: list[float], *, check_collision: bool) -> tuple[bool, str | None]:
        """Display a configuration and optionally collision-check it."""
        self.cfg.joint_values = [float(v) for v in q_rad[:6]]
        self.rcs.robot_configuration = self.cfg
        self.planner.set_robot_cell_state(self.rcs)
        if not check_collision:
            return False, None
        try:
            self.planner.check_collision(self.rcs, options={"verbose": False})
            return False, None
        except self.collision_error as exc:
            return True, str(exc)

    def set_hud(self, text: str, *, collision: bool = False) -> None:
        """Display case state and controls above the robot."""
        kwargs = {
            "textColorRGB": [1.0, 0.15, 0.1] if collision else [0.1, 0.85, 0.2],
            "textSize": 1.25,
            "physicsClientId": self.client_id,
        }
        if self.hud_id >= 0:
            kwargs["replaceItemUniqueId"] = self.hud_id
        self.hud_id = self.pb.addUserDebugText(
            text,
            [0.0, 0.0, 1.75],
            **kwargs,
        )

    def is_open(self) -> bool:
        """Return False after the user closes the PyBullet window."""
        return bool(self.pb.isConnected(self.client_id))

    def triggered(self, events: dict[int, int], key: str) -> bool:
        """Return True when one character key was newly pressed."""
        return bool(events.get(ord(key), 0) & self.pb.KEY_WAS_TRIGGERED)

    def close(self) -> None:
        """Close the compas_fab client and GUI window."""
        try:
            self.client.__exit__(None, None, None)
        except Exception:
            pass


def _case_hud(
    case_set: str,
    index: int,
    total: int,
    q_deg: list[float],
    validation_row: dict | None,
    state: str,
) -> str:
    """Build the compact GUI overlay for a selected case."""
    validation = "not in selected validation log"
    if validation_row is not None:
        validation = (
            f"{validation_row.get('status', 'unknown')}  "
            f"t={float(validation_row.get('elapsed_s') or 0.0):.2f}s  "
            f"iter={int(validation_row.get('iterations') or 0)}"
        )
    return (
        f"{case_set.upper()} case {index + 1}/{total}  {state}\n"
        f"q_deg={_format_joints(q_deg)}\n"
        f"validation={validation}\n"
        "N/P next/previous | E/H set | S start | G goal | C collision | A animate | Q quit"
    )


def _show_selected_case(
    viewer: PyBulletConfigViewer,
    case_set: str,
    index: int,
    cases: list[list[float]],
    validation_rows: dict[tuple[float, ...], dict],
) -> None:
    """Display and report one dataset start configuration."""
    q_deg = cases[index]
    collision, detail = viewer.set_configuration(_deg_to_rad(q_deg), check_collision=True)
    row = validation_rows.get(_config_key(q_deg))
    state = "START COLLIDES" if collision else "start is collision-free"
    viewer.set_hud(_case_hud(case_set, index, len(cases), q_deg, row, state), collision=collision)
    print(
        f"[viewer] {case_set} case={index + 1}/{len(cases)} "
        f"collision={collision} q_deg={_format_joints(q_deg)}",
        flush=True,
    )
    if detail:
        print(f"[viewer] collision: {detail}", flush=True)


def _scan_direct_path(
    viewer: PyBulletConfigViewer,
    start_deg: list[float],
    goal_deg: list[float],
    step_deg: float,
) -> CollisionScan:
    """Find and display the first collision on the exact direct line."""
    points = discretize_joint_line(
        _deg_to_rad(start_deg),
        _deg_to_rad(goal_deg),
        math.radians(step_deg),
    )
    print(f"[viewer] scanning {len(points)} direct-line points at {step_deg:.4f} deg...", flush=True)
    for point_index, q_rad in enumerate(points):
        collision, detail = viewer.set_configuration(q_rad, check_collision=True)
        if collision:
            print(
                f"[viewer] first collision point={point_index + 1}/{len(points)} "
                f"progress={point_index / max(1, len(points) - 1):.1%}",
                flush=True,
            )
            print(f"[viewer] collision: {detail}", flush=True)
            return CollisionScan(True, point_index, len(points), list(q_rad), detail)
        if point_index and point_index % 500 == 0:
            print(f"[viewer] scan progress {point_index}/{len(points)}", flush=True)
    print("[viewer] direct line is collision-free", flush=True)
    return CollisionScan(False, len(points) - 1, len(points), list(points[-1]), None)


def main(argv: list[str] | None = None) -> int:
    """Load datasets and run the interactive PyBullet configuration browser."""
    parser = argparse.ArgumentParser(description="Visualize reset datasets in PyBullet")
    parser.add_argument("--easy-dataset", default=str(DEFAULT_EASY_DATASET), help="Easy-case JSON path")
    parser.add_argument("--hard-dataset", default=str(DEFAULT_HARD_DATASET), help="Hard-case JSON path")
    parser.add_argument("--case-set", choices=("easy", "hard"), default="hard", help="Initial dataset")
    parser.add_argument("--case-index", type=int, default=1, help="Initial one-based case index")
    parser.add_argument("--validation-log", default=None, help="Optional validation JSON displayed in HUD")
    parser.add_argument("--scan-step-deg", type=float, default=0.05, help="Exact collision scan spacing")
    parser.add_argument("--animation-step-deg", type=float, default=1.0, help="Visual animation spacing")
    parser.add_argument("--animation-hz", type=float, default=30.0, help="Visual animation update rate")
    parser.add_argument("--scan-on-start", action="store_true", help="Scan selected direct path immediately")
    parser.add_argument("--list", action="store_true", help="Print dataset/log counts without opening GUI")
    args = parser.parse_args(argv)

    easy_path = Path(args.easy_dataset).expanduser().resolve()
    hard_path = Path(args.hard_dataset).expanduser().resolve()
    validation_path = Path(args.validation_log).expanduser().resolve() if args.validation_log else None
    datasets = {
        "easy": _load_starts(easy_path),
        "hard": _load_starts(hard_path),
    }
    validation_rows = _load_validation_rows(validation_path)
    matched_easy = sum(1 for q in datasets["easy"] if _config_key(q) in validation_rows)
    matched_hard = sum(1 for q in datasets["hard"] if _config_key(q) in validation_rows)
    print(
        f"[viewer] easy={len(datasets['easy'])} hard={len(datasets['hard'])} "
        f"validation_rows={len(validation_rows)} "
        f"matched_easy={matched_easy} matched_hard={matched_hard}",
        flush=True,
    )
    if args.list:
        return 0
    if not datasets[args.case_set]:
        print(f"[viewer] no {args.case_set} cases available", file=sys.stderr)
        return 2

    case_set = args.case_set
    case_index = max(0, min(args.case_index - 1, len(datasets[case_set]) - 1))
    goal_deg = list(RESET_POSE_DEG)
    viewer = PyBulletConfigViewer()
    animation_points: list[list[float]] = []  # Coarse points used only for visual playback.
    animation_index = 0  # Next animation point to display.
    next_animation_time = 0.0  # Monotonic deadline for the next frame.

    def show_start() -> None:
        """Display the currently selected start and clear animation."""
        nonlocal animation_points, animation_index
        animation_points = []
        animation_index = 0
        _show_selected_case(viewer, case_set, case_index, datasets[case_set], validation_rows)

    def scan_current() -> None:
        """Scan the selected direct line and show its first collision."""
        nonlocal animation_points, animation_index
        animation_points = []
        animation_index = 0
        start_deg = datasets[case_set][case_index]
        scan = _scan_direct_path(viewer, start_deg, goal_deg, args.scan_step_deg)
        row = validation_rows.get(_config_key(start_deg))
        if scan.collision:
            progress = scan.point_index / max(1, scan.point_count - 1)
            state = f"FIRST DIRECT COLLISION at {progress:.1%}"
        else:
            state = "direct line is collision-free"
        viewer.set_hud(
            _case_hud(case_set, case_index, len(datasets[case_set]), start_deg, row, state),
            collision=scan.collision,
        )

    try:
        show_start()
        if args.scan_on_start:
            scan_current()
        while viewer.is_open():
            events = viewer.pb.getKeyboardEvents(physicsClientId=viewer.client_id)
            if viewer.triggered(events, "q"):
                break
            if viewer.triggered(events, "n"):
                case_index = (case_index + 1) % len(datasets[case_set])
                show_start()
            if viewer.triggered(events, "p"):
                case_index = (case_index - 1) % len(datasets[case_set])
                show_start()
            if viewer.triggered(events, "e") and datasets["easy"]:
                case_set = "easy"
                case_index = min(case_index, len(datasets[case_set]) - 1)
                show_start()
            if viewer.triggered(events, "h") and datasets["hard"]:
                case_set = "hard"
                case_index = min(case_index, len(datasets[case_set]) - 1)
                show_start()
            if viewer.triggered(events, "s"):
                show_start()
            if viewer.triggered(events, "g"):
                animation_points = []
                viewer.set_configuration(_deg_to_rad(goal_deg), check_collision=False)
                viewer.set_hud("RESET GOAL\nq_deg=" + _format_joints(goal_deg))
            if viewer.triggered(events, "c"):
                scan_current()
            if viewer.triggered(events, "a"):
                animation_points = discretize_joint_line(
                    _deg_to_rad(datasets[case_set][case_index]),
                    _deg_to_rad(goal_deg),
                    math.radians(args.animation_step_deg),
                )
                animation_index = 0
                next_animation_time = time.perf_counter()
                print(f"[viewer] animating {len(animation_points)} visual points", flush=True)

            now = time.perf_counter()
            if animation_points and now >= next_animation_time:
                viewer.set_configuration(animation_points[animation_index], check_collision=False)
                animation_index += 1
                next_animation_time = now + 1.0 / max(1.0, args.animation_hz)
                if animation_index >= len(animation_points):
                    animation_points = []
                    animation_index = 0
            time.sleep(1.0 / 120.0)
    except KeyboardInterrupt:
        print("\n[viewer] Ctrl+C received", flush=True)
    finally:
        viewer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
