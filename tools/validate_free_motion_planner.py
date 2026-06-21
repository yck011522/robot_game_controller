bounds"""Generate reusable reset cases and validate the free-motion planner.

Generation classifies each unique random configuration through the planner
in direct-only mode. Directly reachable starts accumulate in the easy JSON;
blocked direct paths accumulate in the hard JSON. Validation can replay the
easy set, hard set, both sets, or a custom dataset after planner changes.
Pressing Ctrl+C saves completed generation/validation work through atomic
JSON replacement, shuts down spawned processes, and exits with code 130.

By default, generated run artifacts are written under
`tools/runs/free_motion_planner/`.

Typical commands
----------------

Generate and classify 100 new unique cases for up to 20 minutes. Cases
append to the persistent easy/hard JSON files; duplicates are skipped:

python tools/validate_free_motion_planner.py --generate --generate-only --starts 100 --generation-duration-min 20

Run a shorter five-minute generation pass with a different deterministic
sample sequence:

python tools/validate_free_motion_planner.py --generate --generate-only --starts 25 --generation-duration-min 5 --seed 20260620

Validate every easy, hard, or combined saved case using planner defaults
(18 workers, 0.05 deg spacing, 1000 iterations per attempt, two seconds
per attempt, four restarts, and ten seconds total):

python tools/validate_free_motion_planner.py --case-set easy
python tools/validate_free_motion_planner.py --case-set hard
python tools/validate_free_motion_planner.py --case-set all

Validate only the first ten hard cases with a smaller worker pool:

python tools/validate_free_motion_planner.py --case-set hard --max-cases 10 --workers 4

Use the planner as a direct-line checker. Easy cases should return a direct
trajectory; blocked hard cases should return `no_direct_path`:

python tools/validate_free_motion_planner.py --case-set all --iterations-per-attempt 0 --max-restarts 0

Run the known-good longer BiRRT-Connect configuration for hard cases:

python tools/validate_free_motion_planner.py --case-set hard --iterations-per-attempt 1000 --attempt-timeout-s 30 --max-restarts 0 --total-timeout-s 30 --batch-size 8

Apply a global collision-sample budget to each planning case:

python tools/validate_free_motion_planner.py --case-set hard --max-collision-samples 100000

Compare one-request sequential dispatch with full 18-worker dispatch:

python tools/validate_free_motion_planner.py --case-set easy --max-cases 1 --workers 18 --max-in-flight 1
python tools/validate_free_motion_planner.py --case-set easy --max-cases 1 --workers 18 --max-in-flight 18

Validate an arbitrary compatible JSON dataset instead of the canonical
easy/hard files:

python tools/validate_free_motion_planner.py --case-set custom --dataset tools/my_reset_cases.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
DEFAULT_PROFILE = REPO_ROOT / "config" / "profiles" / "dev_keyboard_headless.yaml"
DEFAULT_DATASET = REPO_ROOT / "tools" / "free_motion_reset_starts.json"
DEFAULT_HARD_DATASET = REPO_ROOT / "tools" / "free_motion_reset_hard_starts.json"
DEFAULT_RESULTS_DIR = REPO_ROOT / "tools" / "runs" / "free_motion_planner"
RESET_POSE_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.config import load as load_profile
from subsystems.motion_planning import (
    BiRRTConnectPlanner,
    PlannerSettings,
    PlanStatus,
    path_max_axis_step,
)
from subsystems.motion_planning.collision_client import CollisionWorkerClient, WorkerCollisionOracle
from subsystems.robot.joint_limits import resolve_joint_limits_rad


def _spawn(module: str, profile: Path, *extra: str) -> subprocess.Popen:
    """Spawn one repository process with `PYTHONPATH=src`."""
    env = os.environ.copy()
    env["PYTHONPATH"] = str(SRC) + (os.pathsep + env.get("PYTHONPATH", "") if env.get("PYTHONPATH") else "")
    cmd = [sys.executable, "-m", module, "--profile", str(profile), *extra]
    kwargs: dict = {}
    if os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # New process group for Ctrl-Break shutdown.
    return subprocess.Popen(
        cmd,
        cwd=str(REPO_ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        **kwargs,
    )


def _read_line(proc: subprocess.Popen) -> str | None:
    """Read one stdout line from a child process, if available."""
    if proc.stdout is None:
        return None
    line = proc.stdout.readline()
    return line.rstrip("\n") if line else None


def _terminate(proc: subprocess.Popen, grace_s: float = 5.0) -> None:
    """Terminate one child process without leaving broker/worker orphans."""
    if proc.poll() is not None:
        return
    try:
        if os.name == "nt":
            proc.send_signal(signal.CTRL_BREAK_EVENT)  # Ask Windows child process group to exit.
        else:
            proc.terminate()
    except Exception:
        pass
    try:
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        try:
            proc.kill()
        except Exception:
            pass


def _start_collision_pool(profile: Path, worker_count: int) -> list[tuple[str, subprocess.Popen]]:
    """Start bus broker, collision broker, and collision workers."""
    procs: list[tuple[str, subprocess.Popen]] = []
    procs.append(("bus_broker", _spawn("apps.bus_broker", profile, "--proc", "bus_broker")))
    _wait_for_banner(procs[-1][1], "bus broker up", "bus_broker", 20.0)

    procs.append(("collision_broker", _spawn("apps.collision_broker", profile, "--proc", "collision_broker")))
    _wait_for_banner(procs[-1][1], "ready:", "collision_broker", 20.0)

    for idx in range(worker_count):
        name = f"worker_{idx:02d}"
        proc = _spawn("subsystems.collision_worker", profile, "--proc", "collision_worker", "--instance", str(idx))
        procs.append((name, proc))

    ready = set()  # Worker indices that printed a ready banner.
    deadline = time.perf_counter() + 90.0
    while time.perf_counter() < deadline and len(ready) < worker_count:
        for idx, (name, proc) in enumerate(procs[2:]):
            line = _read_line(proc)
            if not line:
                continue
            print(f"[{name}] {line}")
            if "ready:" in line:
                ready.add(idx)
        failed = [name for name, proc in procs if proc.poll() is not None]
        if failed:
            raise RuntimeError(f"process exited early: {failed}")
    if len(ready) < worker_count:
        raise TimeoutError(f"only {len(ready)}/{worker_count} collision workers became ready")
    return procs


def _wait_for_banner(proc: subprocess.Popen, needle: str, label: str, timeout_s: float) -> None:
    """Block until a child process prints a startup banner."""
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        line = _read_line(proc)
        if line:
            print(f"[{label}] {line}")
            if needle in line:
                return
        if proc.poll() is not None:
            raise RuntimeError(f"{label} exited early with rc={proc.returncode}")
    raise TimeoutError(f"{label} did not print {needle!r} within {timeout_s:.1f}s")


def _shutdown_pool(procs: list[tuple[str, subprocess.Popen]]) -> None:
    """Stop all spawned child processes in reverse startup order."""
    for _, proc in reversed(procs):
        _terminate(proc)


def _deg_to_rad(values: list[float]) -> list[float]:
    """Convert a six-axis degree vector to radians."""
    return [math.radians(float(v)) for v in values[:6]]


def _rad_to_deg(values: list[float]) -> list[float]:
    """Convert a six-axis radian vector to degrees."""
    return [math.degrees(float(v)) for v in values[:6]]


def _make_settings(args: argparse.Namespace) -> PlannerSettings:
    """Build planner settings from CLI degree-based tuning values."""
    step_deg = _resolve_step_deg(args)
    return PlannerSettings(
        max_iterations_per_attempt=args.iterations_per_attempt,
        extend_step_rad=math.radians(args.extend_step_deg),
        trajectory_step_rad=math.radians(step_deg),
        goal_sample_rate=args.goal_sample_rate,
        max_connect_steps=args.max_connect_steps,
        smooth_iterations=args.smooth_iterations,
        corner_window=args.corner_window,
        attempt_timeout_s=args.attempt_timeout_s,
        max_restarts=args.max_restarts,
        total_timeout_s=args.total_timeout_s,
        max_collision_samples=args.max_collision_samples,
        rng_seed=args.seed,
    )


def _resolve_step_deg(args: argparse.Namespace) -> float:
    """Resolve the single collision/output trajectory spacing in degrees."""
    step_deg = float(args.step_deg)
    legacy_values = [
        value for value in (args.collision_step_deg, args.output_step_deg)
        if value is not None
    ]
    for legacy in legacy_values:
        if abs(float(legacy) - step_deg) > 1e-12:
            raise ValueError(
                "collision sampling spacing and output spacing must be identical; "
                f"use --step-deg {step_deg:g} or pass matching legacy values"
            )
    return step_deg


def _config_key(q_deg: list[float]) -> tuple[float, ...]:
    """Return a stable cross-file deduplication key at 1e-8 degrees."""
    return tuple(round(float(v), 8) for v in q_deg[:6])


def _atomic_write_json(path: Path, payload: dict) -> None:
    """Write JSON atomically so interruption cannot truncate the target.

    The complete document is flushed to a process-specific temporary file
    beside the target. `os.replace` then swaps it into place atomically.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(f".{path.name}.{os.getpid()}.tmp")  # Private incomplete output.
    try:
        with temp_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, path)
    finally:
        try:
            temp_path.unlink(missing_ok=True)
        except OSError:
            pass


def _load_dataset_payload(path: Path, dataset_kind: str) -> dict:
    """Load an existing dataset or create an empty compatible payload."""
    if path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            data.setdefault("starts_deg", [])
            data.setdefault("created_wall", datetime.now().isoformat(timespec="seconds"))
            data.setdefault("units", "degrees")
            data.setdefault("dataset_kind", dataset_kind)
            return data
    return {
        "created_wall": datetime.now().isoformat(timespec="seconds"),
        "updated_wall": datetime.now().isoformat(timespec="seconds"),
        "dataset_kind": dataset_kind,
        "units": "degrees",
        "starts_deg": [],
    }


def _write_dataset_payload(path: Path, payload: dict) -> None:
    """Persist one accumulated dataset after updating its metadata."""
    payload["updated_wall"] = datetime.now().isoformat(timespec="seconds")
    payload["starts_count"] = len(payload.get("starts_deg") or [])
    _atomic_write_json(path, payload)


def _classify_generated_cases(
    *,
    easy_path: Path,
    hard_path: Path,
    starts: int,
    seed: int,
    duration_s: float,
    goal_rad: list[float],
    q_min_rad: list[float],
    q_max_rad: list[float],
    oracle: WorkerCollisionOracle,
    settings: PlannerSettings,
    log_every: int,
    sample_batch_size: int,
    out_dir: Path,
) -> tuple[Path, bool]:
    """Classify new unique samples through the planner's direct-only mode."""
    easy_payload = _load_dataset_payload(easy_path, "easy")
    hard_payload = _load_dataset_payload(hard_path, "hard")
    easy_starts = [[float(v) for v in q[:6]] for q in easy_payload.get("starts_deg") or []]
    hard_starts = [[float(v) for v in q[:6]] for q in hard_payload.get("starts_deg") or []]
    known_keys = {_config_key(q) for q in easy_starts}
    known_keys.update(_config_key(q) for q in hard_starts)

    direct_settings = replace(
        settings,
        max_iterations_per_attempt=0,
        max_restarts=0,
    )
    planner = BiRRTConnectPlanner(
        q_min_rad=q_min_rad,
        q_max_rad=q_max_rad,
        collision_oracle=oracle,
        settings=direct_settings,
    )
    rng = random.Random(seed)
    started = time.perf_counter()
    deadline = started + max(0.0, duration_s)
    log_every = max(1, int(log_every))
    sample_batch_size = max(1, int(sample_batch_size))
    attempts = duplicates = colliding = operational_failures = 0
    new_easy = new_hard = 0
    rows: list[dict] = []  # Per-candidate classification records for this run.
    interrupted = False  # True when the operator requested Ctrl+C shutdown.

    print(
        f"[generate] classify new={starts} duration={duration_s / 60.0:.1f}min "
        f"existing_easy={len(easy_starts)} existing_hard={len(hard_starts)} "
        f"step={math.degrees(settings.trajectory_step_rad):.4f}deg "
        f"sample_batch={sample_batch_size}",
        flush=True,
    )
    try:
        while new_easy + new_hard < starts and time.perf_counter() < deadline:
            candidates: list[tuple[int, list[float], list[float], tuple[float, ...]]] = []
            for _ in range(sample_batch_size):
                q_rad = [rng.uniform(lo, hi) for lo, hi in zip(q_min_rad, q_max_rad)]
                q_deg = _rad_to_deg(q_rad)
                attempts += 1
                key = _config_key(q_deg)
                if key in known_keys:
                    duplicates += 1
                    if duplicates % log_every == 0:
                        print(f"[generate] duplicate skipped attempts={attempts} duplicates={duplicates}", flush=True)
                    continue
                candidates.append((attempts, q_rad, q_deg, key))
            if not candidates:
                continue

            endpoint_free = oracle.are_configs_free(
                [candidate[1] for candidate in candidates],
                batch_size=1,
            )
            print(
                f"[generate] endpoint batch checked={len(candidates)} "
                f"free={sum(1 for free in endpoint_free if free)} attempts={attempts}",
                flush=True,
            )
            for (attempt, q_rad, q_deg, key), is_free in zip(candidates, endpoint_free):
                if new_easy + new_hard >= starts or time.perf_counter() >= deadline:
                    break
                if not is_free:
                    colliding += 1
                    rows.append({
                        "attempt": attempt,
                        "status": PlanStatus.START_IN_COLLISION.value,
                        "elapsed_s": 0.0,
                        "collision_samples": 1,
                        "start_deg": q_deg,
                    })
                    if colliding % log_every == 0:
                        print(f"[generate] sampled configuration collides; skipped attempts={attempt}", flush=True)
                    continue

                result = planner.plan_detailed(q_rad, goal_rad)
                row = {
                    "attempt": attempt,
                    "status": result.status.value,
                    "elapsed_s": result.elapsed_s,
                    "collision_samples": result.collision_samples,
                    "start_deg": q_deg,
                }
                rows.append(row)
                if result.status == PlanStatus.DIRECT_PATH:
                    easy_starts.append(q_deg)
                    easy_payload["starts_deg"] = easy_starts
                    known_keys.add(key)
                    new_easy += 1
                    classification = "easy/direct"
                elif result.status == PlanStatus.NO_DIRECT_PATH:
                    hard_starts.append(q_deg)
                    hard_payload["starts_deg"] = hard_starts
                    known_keys.add(key)
                    new_hard += 1
                    classification = "hard/blocked-direct"
                else:
                    operational_failures += 1
                    print(
                        f"[generate] unclassified status={result.status.value} attempts={attempt} "
                        f"start_deg={[round(v, 2) for v in q_deg]}",
                        flush=True,
                    )
                    continue

                _write_dataset_payload(easy_path, easy_payload)
                _write_dataset_payload(hard_path, hard_payload)
                print(
                    f"[generate] {classification} new={new_easy + new_hard}/{starts} "
                    f"easy_added={new_easy} hard_added={new_hard} "
                    f"start_deg={[round(v, 2) for v in q_deg]}",
                    flush=True,
                )
    except KeyboardInterrupt:
        interrupted = True
        print("\n[generate] Ctrl+C received; saving completed classifications...", flush=True)

    _write_dataset_payload(easy_path, easy_payload)
    _write_dataset_payload(hard_path, hard_payload)
    summary = {
        "created_wall": datetime.now().isoformat(timespec="seconds"),
        "seed": seed,
        "requested_new_cases": starts,
        "attempts": attempts,
        "duplicates_skipped": duplicates,
        "colliding_skipped": colliding,
        "operational_failures": operational_failures,
        "new_easy": new_easy,
        "new_hard": new_hard,
        "total_easy": len(easy_starts),
        "total_hard": len(hard_starts),
        "elapsed_s": time.perf_counter() - started,
        "interrupted": interrupted,
        "rows": rows,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = out_dir / f"free_motion_case_generation_{stamp}.json"
    _atomic_write_json(log_path, summary)
    print(
        f"[generate] complete easy_added={new_easy} hard_added={new_hard} "
        f"duplicates={duplicates} log={log_path}",
        flush=True,
    )
    return log_path, interrupted


def _load_dataset(path: Path) -> list[list[float]]:
    """Load random starts from the validation dataset."""
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    starts = data.get("starts_deg") or []
    return [[float(v) for v in q[:6]] for q in starts]


def _load_case_set(
    case_set: str,
    *,
    easy_path: Path,
    hard_path: Path,
    custom_path: Path,
) -> tuple[list[list[float]], list[str]]:
    """Load easy, hard, combined, or custom validation cases."""
    if case_set == "easy":
        return _load_dataset(easy_path), [str(easy_path)]
    if case_set == "hard":
        return _load_dataset(hard_path), [str(hard_path)]
    if case_set == "custom":
        return _load_dataset(custom_path), [str(custom_path)]
    easy = _load_dataset(easy_path)
    hard = _load_dataset(hard_path)
    combined: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for q in easy + hard:
        key = _config_key(q)
        if key not in seen:
            combined.append(q)
            seen.add(key)
    return combined, [str(easy_path), str(hard_path)]


def _validate_cases(
    *,
    starts_deg: list[list[float]],
    goal_deg: list[float],
    q_min_rad: list[float],
    q_max_rad: list[float],
    oracle: WorkerCollisionOracle,
    settings: PlannerSettings,
    max_cases: int | None,
) -> dict:
    """Run planner validation for each selected start configuration."""
    selected = starts_deg[:max_cases] if max_cases is not None else starts_deg
    rows: list[dict] = []  # Per-case validation rows.
    goal_rad = _deg_to_rad(goal_deg)
    passed = 0
    interrupted = False  # True when Ctrl+C stops validation between completed rows.
    try:
        for case_idx, start_deg in enumerate(selected, start=1):
            start_rad = _deg_to_rad(start_deg)
            planner = BiRRTConnectPlanner(
                q_min_rad=q_min_rad,
                q_max_rad=q_max_rad,
                collision_oracle=oracle,
                settings=settings,
            )
            print(f"[plan] case {case_idx}/{len(selected)} start_deg={[round(v, 2) for v in start_deg]}")
            result = planner.plan_detailed(start_rad, goal_rad)
            max_step_deg = math.degrees(path_max_axis_step(result.path_rad))
            ok_step = (not result.path_rad) or max_step_deg <= math.degrees(settings.trajectory_step_rad) + 1e-9
            ok_collision = result.success and oracle.is_edge_free(result.path_rad)
            ok = bool(result.success and ok_step and ok_collision)
            passed += 1 if ok else 0
            row = {
                "case": case_idx,
                "success": result.success,
                "valid": ok,
                "status": result.status.value,
                "message": result.message,
                "iterations": result.iterations,
                "attempts": result.attempts,
                "planner_collision_samples": result.collision_samples,
                "nodes_added": result.nodes_added,
                "connect_steps": result.connect_steps,
                "elapsed_s": result.elapsed_s,
                "sparse_points": len(result.sparse_path_rad),
                "dense_points": len(result.path_rad),
                "corners": len(result.corners),
                "max_axis_step_deg": max_step_deg,
                "start_deg": start_deg,
            }
            rows.append(row)
            print(
                f"[plan] {'PASS' if ok else 'FAIL'} case={case_idx} "
                f"status={result.status.value} attempts={result.attempts} "
                f"iterations={result.iterations} sparse={len(result.sparse_path_rad)} "
                f"dense={len(result.path_rad)} step={max_step_deg:.4f}deg "
                f"elapsed={result.elapsed_s:.2f}s"
            )
    except KeyboardInterrupt:
        interrupted = True
        print("\n[plan] Ctrl+C received; writing partial validation results...", flush=True)
    processed = len(rows)  # Fully completed validation cases.
    return {
        "cases_requested": len(selected),
        "cases": processed,
        "passed": passed,
        "pass_rate": (passed / processed) if processed else 0.0,
        "interrupted": interrupted,
        "rows": rows,
        "collision_config_checks": oracle.config_checks,
        "collision_batch_checks": oracle.batch_checks,
    }


def _write_results(summary: dict, out_dir: Path) -> Path:
    """Write validation summary JSON and return its path."""
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"free_motion_planner_validation_{stamp}.json"
    _atomic_write_json(path, summary)
    return path


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint for dataset generation and planner validation."""
    parser = argparse.ArgumentParser(description="Validate BiRRT-Connect reset planning")
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE), help="Profile used for joint limits and worker startup")
    parser.add_argument("--dataset", default=str(DEFAULT_DATASET), help="JSON dataset used when --case-set=custom")
    parser.add_argument("--case-set", choices=("easy", "hard", "all", "custom"), default="easy", help="Dataset group to validate")
    parser.add_argument("--generate", "--generate-hard", dest="generate", action="store_true", help="Classify and append new unique easy/hard cases")
    parser.add_argument("--generate-only", action="store_true", help="Generate the requested dataset and skip planning validation")
    parser.add_argument("--hard-dataset", default=str(DEFAULT_HARD_DATASET), help="JSON output path for hard starts")
    parser.add_argument("--generation-duration-min", "--hard-duration-min", dest="generation_duration_min", type=float, default=20.0, help="Maximum generation/classification duration")
    parser.add_argument("--generation-log-every", "--hard-log-every", dest="generation_log_every", type=int, default=1, help="Print every N duplicate/collision skips")
    parser.add_argument("--generation-sample-batch", type=int, default=64, help="Random endpoints screened concurrently per generation batch")
    parser.add_argument("--starts", type=int, default=100, help="Number of new unique collision-free cases to classify")
    parser.add_argument("--max-cases", type=int, default=None, help="Limit validation to the first N starts")
    parser.add_argument("--workers", type=int, default=18, help="Collision worker process count")
    parser.add_argument("--max-in-flight", type=int, default=None, help="Concurrent collision requests; defaults to worker count")
    parser.add_argument("--seed", type=int, default=20260618, help="RNG seed for dataset and planner")
    parser.add_argument("--batch-size", type=int, default=64, help="Collision configs per worker request")
    parser.add_argument("--extend-step-deg", type=float, default=4.0, help="RRT expansion max per-axis step")
    parser.add_argument("--step-deg", type=float, default=0.05, help="Shared collision/output max per-axis step")
    parser.add_argument("--collision-step-deg", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--output-step-deg", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--iterations-per-attempt", "--max-iterations", dest="iterations_per_attempt", type=int, default=1000, help="RRT expansion attempts before restart; zero is direct-only")
    parser.add_argument("--attempt-timeout-s", type=float, default=2.0, help="Wall-clock limit per RRT tree-pair attempt")
    parser.add_argument("--max-restarts", type=int, default=4, help="Fresh RRT tree-pair retries after the first attempt")
    parser.add_argument("--total-timeout-s", "--plan-timeout-s", dest="total_timeout_s", type=float, default=10.0, help="Total wall-clock limit across all attempts")
    parser.add_argument("--max-collision-samples", type=int, default=0, help="Collision configuration budget; zero is unlimited")
    parser.add_argument("--goal-sample-rate", type=float, default=0.12, help="Probability of sampling the opposite root")
    parser.add_argument("--max-connect-steps", type=int, default=64, help="Greedy opposite-tree steps per BiRRT-Connect iteration")
    parser.add_argument("--smooth-iterations", type=int, default=120, help="Shortcut smoothing attempts")
    parser.add_argument("--corner-window", type=int, default=20, help="Sparse-point window around smoothing kinks")
    parser.add_argument("--out-dir", default=str(DEFAULT_RESULTS_DIR), help="Directory for generated validation and classification artifacts")
    args = parser.parse_args(argv)

    profile_path = Path(args.profile).expanduser().resolve()
    dataset_path = Path(args.dataset).expanduser().resolve()
    hard_dataset_path = Path(args.hard_dataset).expanduser().resolve()
    profile = load_profile(profile_path)
    q_min_rad, q_max_rad = resolve_joint_limits_rad(profile.tuning.get("robot", {}))
    goal_deg = profile.tuning.get("robot", {}).get("initial_pose_deg", RESET_POSE_DEG)
    goal_deg = [float(v) for v in goal_deg[:6]]
    settings = _make_settings(args)
    out_dir = Path(args.out_dir).expanduser().resolve()

    procs: list[tuple[str, subprocess.Popen]] = []
    client: CollisionWorkerClient | None = None
    try:
        print(f"[setup] profile={profile.name} workers={args.workers}")
        procs = _start_collision_pool(profile_path, args.workers)
        client = CollisionWorkerClient(timeout_s=max(args.total_timeout_s, 30.0))
        max_in_flight = args.max_in_flight if args.max_in_flight is not None else args.workers
        oracle = WorkerCollisionOracle(
            client,
            batch_size=args.batch_size,
            max_in_flight=max_in_flight,
        )

        if args.generate:
            _, generation_interrupted = _classify_generated_cases(
                easy_path=Path(DEFAULT_DATASET).resolve(),
                hard_path=hard_dataset_path,
                starts=args.starts,
                seed=args.seed,
                duration_s=args.generation_duration_min * 60.0,
                goal_rad=_deg_to_rad(goal_deg),
                q_min_rad=q_min_rad,
                q_max_rad=q_max_rad,
                oracle=oracle,
                settings=settings,
                log_every=args.generation_log_every,
                sample_batch_size=args.generation_sample_batch,
                out_dir=out_dir,
            )
            if generation_interrupted:
                return 130

        if args.generate_only:
            return 0

        starts_deg, dataset_sources = _load_case_set(
            args.case_set,
            easy_path=Path(DEFAULT_DATASET).resolve(),
            hard_path=hard_dataset_path,
            custom_path=dataset_path,
        )
        summary = _validate_cases(
            starts_deg=starts_deg,
            goal_deg=goal_deg,
            q_min_rad=q_min_rad,
            q_max_rad=q_max_rad,
            oracle=oracle,
            settings=settings,
            max_cases=args.max_cases,
        )
        summary.update({
            "profile": profile.name,
            "case_set": args.case_set,
            "datasets": dataset_sources,
            "goal_deg": goal_deg,
            "settings": {
                "extend_step_deg": args.extend_step_deg,
                "step_deg": math.degrees(settings.trajectory_step_rad),
                "iterations_per_attempt": settings.max_iterations_per_attempt,
                "attempt_timeout_s": settings.attempt_timeout_s,
                "max_restarts": settings.max_restarts,
                "total_timeout_s": settings.total_timeout_s,
                "max_collision_samples": settings.max_collision_samples,
                "max_connect_steps": settings.max_connect_steps,
                "workers": args.workers,
                "max_in_flight": max_in_flight,
                "batch_size": args.batch_size,
                "edge_batch_size": oracle.edge_batch_size,
                "edge_max_in_flight": oracle.edge_max_in_flight,
            },
        })
        results_path = _write_results(summary, out_dir)
        print(f"[summary] passed={summary['passed']}/{summary['cases']} rate={summary['pass_rate']:.1%}")
        print(f"[summary] collision checks={summary['collision_config_checks']} batches={summary['collision_batch_checks']}")
        print(f"[summary] wrote {results_path}")
        if summary.get("interrupted"):
            return 130
        return 0 if summary["passed"] == summary["cases"] else 1
    except KeyboardInterrupt:
        print("\n[shutdown] Ctrl+C received; stopping collision processes...", flush=True)
        return 130
    finally:
        if client is not None:
            client.close()
        _shutdown_pool(procs)


if __name__ == "__main__":
    sys.exit(main())
