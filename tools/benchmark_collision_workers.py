"""Standalone collision-worker benchmark.

This is a utility, not a regular test.

It measures a synthetic game-loop workload against the collision broker
+ worker pool by generating random joint configurations, bundling them
into req.collision_check requests, and waiting for the replies before
advancing to the next simulated frame.

Default workload matches the current dev_keyboard tuning:
- forward path: 12 checks per robot
- proximity: 20 checks per robot (probe_half_deg=10 -> +/- 1..10)
- default robot count: 2

The benchmark sweeps the requested worker counts and bundle sizes and
prints frame-rate and collision-check throughput for each combination.

Run from the repo root, ideally inside the `game` conda environment:

    conda activate game
    python tools/benchmark_collision_workers.py

Generated benchmark artifacts are written under
`tools/runs/collision_benchmark/` by default.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from statistics import mean

import zmq  # type: ignore[import-not-found]

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
DEFAULT_PROFILE = REPO_ROOT / "config" / "profiles" / "dev_keyboard.yaml"
DEFAULT_WORKER_COUNTS = [14, 16, 18, 20, 22, 24]
DEFAULT_BUNDLE_SIZES = [1, 2]
DEFAULT_ROBOTS = 2
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "runs" / "collision_benchmark"

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core import bus
from core.config import load as load_profile
def _parse_csv_ints(raw: str) -> list[int]:
    values = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        values.append(int(part))
    if not values:
        raise ValueError("expected at least one integer")
    return values


def _spawn(module: str, profile: Path, *extra: str) -> subprocess.Popen:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(SRC) + (os.pathsep + env.get("PYTHONPATH", "") if env.get("PYTHONPATH") else "")
    cmd = [sys.executable, "-m", module, "--profile", str(profile), *extra]
    kwargs: dict = {}
    if os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]
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


def _drain_line(proc: subprocess.Popen) -> str | None:
    if proc.stdout is None:
        return None
    line = proc.stdout.readline()
    if not line:
        return None
    return line.rstrip("\n")


def _terminate(proc: subprocess.Popen, grace_s: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    try:
        if os.name == "nt":
            proc.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
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


def _make_configs(total: int, seed: int, home_rad: list[float]) -> list[list[float]]:
    rng = random.Random(seed)
    configs: list[list[float]] = []
    for _ in range(total):
        q = []
        for idx, center in enumerate(home_rad):
            # Keep the sample around the ready pose so the worker pool
            # is exercised with realistic-ish robot motion instead of
            # fully random joint-space jumps.
            jitter_deg = 35.0 if idx < 3 else 20.0
            q.append(center + math.radians(rng.uniform(-jitter_deg, jitter_deg)))
        configs.append(q)
    return configs


def _chunked(seq: list[list[float]], size: int) -> list[list[list[float]]]:
    size = max(1, size)
    return [seq[i:i + size] for i in range(0, len(seq), size)]


def _send_request(sock: zmq.Socket, request_id: int, configs: list[list[float]]) -> None:
    env = bus.make_envelope(f"bench_collision.{request_id}")
    env.update({
        "request_id": request_id,
        "configs_rad": configs,
        "check_self": True,
        "check_world": True,
    })
    sock.send_multipart([
        b"",
        b"req.collision_check",
        json.dumps(env, separators=(",", ":")).encode("utf-8"),
    ])


def _recv_reply(sock: zmq.Socket) -> dict:
    frames = sock.recv_multipart()
    if not frames:
        return {}
    if len(frames) >= 3 and frames[0] == b"":
        payload = frames[2]
    elif len(frames) >= 2:
        payload = frames[1]
    else:
        payload = frames[0]
    return json.loads(payload.decode("utf-8"))


def _run_frame(sock: zmq.Socket, configs: list[list[float]], bundle_size: int,
               timeout_s: float) -> tuple[float, float, int]:
    chunks = _chunked(configs, bundle_size)
    pending = set(range(1, len(chunks) + 1))
    started = time.perf_counter()
    for request_id, chunk in enumerate(chunks, start=1):
        _send_request(sock, request_id, chunk)
    reply_latencies: list[float] = []
    deadline = started + timeout_s
    while pending and time.perf_counter() < deadline:
        remaining_ms = max(0, int((deadline - time.perf_counter()) * 1000))
        if sock.poll(remaining_ms) == 0:
            continue
        reply = _recv_reply(sock)
        request_id = reply.get("request_id")
        if isinstance(request_id, int) and request_id in pending:
            pending.remove(request_id)
            reply_latencies.append((time.perf_counter() - started) * 1000.0)
    if pending:
        missing = sorted(pending)
        raise TimeoutError(f"timed out waiting for replies: {missing}")
    elapsed = time.perf_counter() - started
    fps = 1.0 / elapsed if elapsed > 0 else 0.0
    mean_reply_ms = mean(reply_latencies) if reply_latencies else 0.0
    return fps, mean_reply_ms, len(chunks)


def _run_combo(profile: Path, home_rad: list[float], worker_count: int, bundle_size: int, *,
               robots: int, forward_steps: int, probe_half_deg: int,
               warmup_frames: int, measure_seconds: float, timeout_s: float,
               seed: int) -> dict[str, float]:
    bus_broker = _spawn("apps.bus_broker", profile, "--proc", "bus_broker")
    collision_broker = None
    workers: list[subprocess.Popen] = []
    try:
        # Wait for the bus broker to come up before binding anything else.
        deadline = time.perf_counter() + 20.0
        while time.perf_counter() < deadline:
            line = _drain_line(bus_broker)
            if line:
                print(f"[bus_broker] {line}")
                if "bus broker up" in line:
                    break
            if bus_broker.poll() is not None:
                raise RuntimeError(f"bus_broker exited early rc={bus_broker.returncode}")
        else:
            raise TimeoutError("bus_broker did not start in time")

        collision_broker = _spawn("apps.collision_broker", profile, "--proc", "collision_broker")
        deadline = time.perf_counter() + 20.0
        while time.perf_counter() < deadline:
            line = _drain_line(collision_broker)
            if line:
                print(f"[collision_broker] {line}")
                if "ready:" in line:
                    break
            if collision_broker.poll() is not None:
                raise RuntimeError(f"collision_broker exited early rc={collision_broker.returncode}")
        else:
            raise TimeoutError("collision_broker did not start in time")

        for idx in range(worker_count):
            worker = _spawn(
                "subsystems.collision_worker",
                profile,
                "--proc", "collision_worker",
                "--instance", str(idx),
            )
            workers.append(worker)

        ready = set()
        deadline = time.perf_counter() + 60.0
        while time.perf_counter() < deadline and len(ready) < worker_count:
            for idx, worker in enumerate(workers):
                line = _drain_line(worker)
                if not line:
                    continue
                print(f"[worker_{idx:02d}] {line}")
                if "ready:" in line:
                    ready.add(idx)
            if any(worker.poll() is not None for worker in workers):
                bad = [i for i, worker in enumerate(workers) if worker.poll() is not None]
                raise RuntimeError(f"worker(s) exited early: {bad}")
        if len(ready) < worker_count:
            raise TimeoutError(f"only {len(ready)}/{worker_count} workers became ready")

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.DEALER)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(bus.COLLISION_ROUTER_ENDPOINT)
        time.sleep(0.25)

        configs_per_robot = forward_steps + (2 * probe_half_deg)
        configs_per_frame = configs_per_robot * robots
        warmup_configs = _make_configs(configs_per_frame, seed=seed, home_rad=home_rad)
        for _ in range(max(0, warmup_frames)):
            _run_frame(sock, warmup_configs, bundle_size, timeout_s)

        measured_frames = 0
        total_reply_ms = 0.0
        total_reqs = 0
        start = time.perf_counter()
        frame_seed = seed + 1
        while time.perf_counter() - start < measure_seconds:
            configs = _make_configs(configs_per_frame, seed=frame_seed, home_rad=home_rad)
            frame_seed += 1
            fps, reply_ms, req_count = _run_frame(sock, configs, bundle_size, timeout_s)
            measured_frames += 1
            total_reply_ms += reply_ms
            total_reqs += req_count
        elapsed = time.perf_counter() - start
        total_configs = measured_frames * configs_per_frame
        checks_per_sec = total_configs / elapsed if elapsed > 0 else 0.0
        return {
            "workers": float(worker_count),
            "bundle_size": float(bundle_size),
            "frames": float(measured_frames),
            "elapsed_s": elapsed,
            "fps": measured_frames / elapsed if elapsed > 0 else 0.0,
            "checks_per_sec": checks_per_sec,
            "avg_reply_ms": total_reply_ms / measured_frames if measured_frames else 0.0,
            "requests_per_frame": total_reqs / measured_frames if measured_frames else 0.0,
            "configs_per_frame": float(configs_per_frame),
        }
    finally:
        for worker in workers:
            _terminate(worker)
        if collision_broker is not None:
            _terminate(collision_broker)
        _terminate(bus_broker)


def _fmt_num(value: float) -> str:
    return f"{value:8.2f}"


def _slugify(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in value)


def _write_outputs(*, out_dir: Path, profile_name: str,
                   rows: list[dict[str, float]],
                   worker_counts: list[int], bundle_sizes: list[int]) -> tuple[Path, Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"benchmark_collision_workers_{_slugify(profile_name)}_{stamp}"
    csv_path = out_dir / f"{stem}.csv"
    json_path = out_dir / f"{stem}.json"
    svg_path = out_dir / f"{stem}.svg"

    fieldnames = [
        "workers", "bundle_size", "frames", "elapsed_s", "fps",
        "checks_per_sec", "avg_reply_ms", "requests_per_frame",
        "configs_per_frame",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})

    payload = {
        "profile": profile_name,
        "worker_counts": worker_counts,
        "bundle_sizes": bundle_sizes,
        "rows": rows,
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    _write_svg(svg_path, rows, worker_counts, bundle_sizes)
    return csv_path, json_path, svg_path


def _write_svg(svg_path: Path, rows: list[dict[str, float]],
               worker_counts: list[int], bundle_sizes: list[int]) -> None:
    width = 1200
    height = 760
    margin = 70
    panel_gap = 50
    panel_h = (height - (margin * 2) - panel_gap) / 2
    panel_w = width - (margin * 2)
    plots = ["fps", "checks_per_sec"]
    palette = {
        1: "#c23b22",
        2: "#1b7f79",
        4: "#2f6fed",
        8: "#7a4cff",
        12: "#da8a00",
        14: "#0f9d58",
        16: "#444444",
    }
    max_values = {metric: max((float(row[metric]) for row in rows), default=1.0) for metric in plots}

    def row_for(worker_count: int, bundle_size: int) -> dict[str, float]:
        for row in rows:
            if int(row["workers"]) == worker_count and int(row["bundle_size"]) == bundle_size:
                return row
        raise KeyError((worker_count, bundle_size))

    def x_for(index: int, count: int) -> float:
        if count <= 1:
            return 0.5
        return index / (count - 1)

    def y_for(value: float, max_value: float) -> float:
        if max_value <= 0:
            return 1.0
        return max(0.0, 1.0 - min(value / max_value, 1.0))

    def polyline(metric: str, y_top: float) -> list[str]:
        max_value = max_values[metric] * 1.1 if max_values[metric] > 0 else 1.0
        parts = []
        for bundle_index, bundle_size in enumerate(bundle_sizes):
            points = []
            for worker_index, worker_count in enumerate(worker_counts):
                row = row_for(worker_count, bundle_size)
                x = margin + 40 + x_for(worker_index, len(worker_counts)) * (panel_w - 80)
                y = y_top + 20 + y_for(float(row[metric]), max_value) * (panel_h - 60)
                points.append(f"{x:.1f},{y:.1f}")
            color = palette.get(bundle_size, "#333333")
            parts.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="3" '
                f'points="{" ".join(points)}" />'
            )
            for worker_index, worker_count in enumerate(worker_counts):
                row = row_for(worker_count, bundle_size)
                x = margin + 40 + x_for(worker_index, len(worker_counts)) * (panel_w - 80)
                y = y_top + 20 + y_for(float(row[metric]), max_value) * (panel_h - 60)
                parts.append(
                    f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="{color}" />'
                )
        return parts

    def axes(metric: str, title: str, y_top: float) -> list[str]:
        max_value = max_values[metric] * 1.1 if max_values[metric] > 0 else 1.0
        y_bottom = y_top + panel_h - 40
        x_left = margin + 40
        x_right = margin + panel_w - 40
        parts = [
            f'<rect x="{margin}" y="{y_top}" width="{panel_w}" height="{panel_h}" '
            f'rx="18" fill="#111827" opacity="0.04" stroke="#cbd5e1" />',
            f'<text x="{margin + 10}" y="{y_top + 26}" font-size="22" font-weight="700" fill="#111827">{title}</text>',
            f'<line x1="{x_left}" y1="{y_bottom}" x2="{x_right}" y2="{y_bottom}" stroke="#374151" stroke-width="1.5" />',
            f'<line x1="{x_left}" y1="{y_top + 20}" x2="{x_left}" y2="{y_bottom}" stroke="#374151" stroke-width="1.5" />',
        ]
        for idx, worker_count in enumerate(worker_counts):
            x = x_left + x_for(idx, len(worker_counts)) * (panel_w - 80)
            parts.append(f'<line x1="{x:.1f}" y1="{y_bottom}" x2="{x:.1f}" y2="{y_bottom + 6}" stroke="#374151" />')
            parts.append(f'<text x="{x:.1f}" y="{y_bottom + 24}" text-anchor="middle" font-size="14" fill="#111827">{worker_count}</text>')
        for tick in range(5):
            value = max_value * (tick / 4.0)
            y = y_bottom - (panel_h - 60) * (tick / 4.0)
            parts.append(f'<line x1="{x_left - 6}" y1="{y:.1f}" x2="{x_left}" y2="{y:.1f}" stroke="#374151" />')
            parts.append(f'<text x="{x_left - 10}" y="{y + 5:.1f}" text-anchor="end" font-size="13" fill="#111827">{value:.0f}</text>')
        legend_x = x_right - 220
        legend_y = y_top + 24
        for idx, bundle_size in enumerate(bundle_sizes):
            color = palette.get(bundle_size, "#333333")
            yy = legend_y + idx * 22
            parts.append(f'<rect x="{legend_x}" y="{yy - 12}" width="14" height="14" fill="{color}" />')
            parts.append(f'<text x="{legend_x + 20}" y="{yy}" font-size="13" fill="#111827">bundle {bundle_size}</text>')
        return parts

    svg_parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc" />',
        f'<text x="{margin}" y="40" font-size="28" font-weight="700" fill="#111827">Collision worker benchmark</text>',
        f'<text x="{margin}" y="60" font-size="14" fill="#475569">worker counts {worker_counts} | bundle sizes {bundle_sizes}</text>',
    ]
    svg_parts.extend(axes("fps", "Frame rate (simulated game-loop FPS)", margin + 20))
    svg_parts.extend(polyline("fps", margin + 20))
    svg_parts.extend(axes("checks_per_sec", "Collision checks per second", margin + 20 + panel_h + panel_gap))
    svg_parts.extend(polyline("checks_per_sec", margin + 20 + panel_h + panel_gap))
    svg_parts.append('</svg>')
    svg_path.write_text("\n".join(svg_parts), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Benchmark collision worker pool throughput")
    ap.add_argument("--profile", default=str(DEFAULT_PROFILE),
                    help="profile YAML to load for tuning values")
    ap.add_argument("--worker-counts", default=",".join(map(str, DEFAULT_WORKER_COUNTS)),
                    help="comma-separated worker counts to benchmark")
    ap.add_argument("--bundle-sizes", default=",".join(map(str, DEFAULT_BUNDLE_SIZES)),
                    help="comma-separated bundle sizes to benchmark")
    ap.add_argument("--robots", type=int, default=DEFAULT_ROBOTS,
                    help="number of robots to synthesize per frame")
    ap.add_argument("--forward-steps", type=int, default=None,
                    help="override forward path checks per robot; defaults from profile")
    ap.add_argument("--probe-half-deg", type=int, default=None,
                    help="override proximity probe span; defaults from profile")
    ap.add_argument("--warmup-frames", type=int, default=5,
                    help="frames to warm up before timing")
    ap.add_argument("--measure-seconds", type=float, default=6.0,
                    help="timed window per worker/bundle combination")
    ap.add_argument("--timeout-s", "--frame-timeout-s", dest="timeout_s",
                    type=float, default=30.0,
                    help="per-frame reply timeout")
    ap.add_argument("--seed", type=int, default=12345,
                    help="RNG seed for repeatable workloads")
    ap.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR),
                    help="directory where CSV/JSON/SVG outputs are written")
    args = ap.parse_args(argv)

    profile_path = Path(args.profile).expanduser().resolve()
    if not profile_path.exists():
        print(f"missing profile: {profile_path}", file=sys.stderr)
        return 2
    profile = load_profile(profile_path)
    jog = profile.tuning.get("jogging", {}) or {}
    forward_steps = args.forward_steps if args.forward_steps is not None else int(jog.get("n_forward_steps", 12))
    probe_half_deg = args.probe_half_deg if args.probe_half_deg is not None else int(jog.get("probe_half_deg", 10))
    robot_tune = profile.tuning.get("robot", {}) or {}
    home_deg = robot_tune.get("initial_pose_deg", [0.0, -90.0, 90.0, 0.0, 0.0, 0.0])
    home_rad = [math.radians(float(v)) for v in home_deg][:6]
    while len(home_rad) < 6:
        home_rad.append(0.0)

    worker_counts = _parse_csv_ints(args.worker_counts)
    bundle_sizes = _parse_csv_ints(args.bundle_sizes)

    print(f"[bench] profile={profile.name} ({profile_path})")
    print(f"[bench] workload: robots={args.robots} forward_steps={forward_steps} probe_half_deg={probe_half_deg}")
    print(f"[bench] sweep: workers={worker_counts} bundle_sizes={bundle_sizes}")
    print("[bench] note: each frame uses forward_steps + 2*probe_half_deg configs per robot")
    print()

    rows: list[dict[str, float]] = []
    for worker_count in worker_counts:
        for bundle_size in bundle_sizes:
            print(f"[bench] running workers={worker_count} bundle_size={bundle_size} ...", flush=True)
            row = _run_combo(
                profile_path,
                home_rad,
                worker_count,
                bundle_size,
                robots=args.robots,
                forward_steps=forward_steps,
                probe_half_deg=probe_half_deg,
                warmup_frames=args.warmup_frames,
                measure_seconds=args.measure_seconds,
                timeout_s=args.timeout_s,
                seed=args.seed,
            )
            rows.append(row)
            print(
                f"[bench] workers={worker_count:2d} bundle={bundle_size:1d}  "
                f"fps={_fmt_num(row['fps'])}  "
                f"checks/s={_fmt_num(row['checks_per_sec'])}  "
                f"avg_reply_ms={_fmt_num(row['avg_reply_ms'])}  "
                f"reqs/frame={_fmt_num(row['requests_per_frame'])}"
            )
            print()

    print("[bench] summary")
    print("workers  bundle      fps  checks/s  avg_reply_ms  reqs/frame  configs/frame")
    for row in rows:
        print(
            f"{int(row['workers']):7d}  {int(row['bundle_size']):6d}  "
            f"{row['fps']:8.2f}  {row['checks_per_sec']:8.2f}  "
            f"{row['avg_reply_ms']:12.2f}  {row['requests_per_frame']:10.2f}  "
            f"{row['configs_per_frame']:13.0f}"
        )
    out_dir = Path(args.out_dir).expanduser().resolve()
    csv_path, json_path, svg_path = _write_outputs(
        out_dir=out_dir,
        profile_name=profile.name,
        rows=rows,
        worker_counts=worker_counts,
        bundle_sizes=bundle_sizes,
    )
    print()
    print(f"[bench] wrote {csv_path}")
    print(f"[bench] wrote {json_path}")
    print(f"[bench] wrote {svg_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
