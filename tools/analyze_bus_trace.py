"""Plot a recorded bus trace to eyeball dial / speed-override behaviour.

This is the Phase-0 observation companion to ``tools/bus_trace_recorder.py``.
It deliberately applies NO pass/fail thresholds: it just draws what happened so
the operator can spot dial glitches, false movement-detect wakes, and
speed-override-to-zero events by eye.

Data sources read from the JSONL trace (one ``{ts_recv_wall_ns, topic, body}``
row per line):
  * ``telem.haptic.<team>``        -> dial positions (and raw firmware fields)
  * ``telem.jogging.debug.<team>`` -> event-based speed-override drops
  * ``state.full``                 -> dense final_scalar + stage timeline

Typical commands
----------------
# Default: plot logs/trace/bus_trace_latest.jsonl for team A, show window.
python tools/analyze_bus_trace.py

# Pick a team and a specific trace file.
python tools/analyze_bus_trace.py --team a --input logs/trace/bus_trace_latest.jsonl

# Use the raw firmware decidegree field (shows glitches without rad rounding).
python tools/analyze_bus_trace.py --team a --decideg

# Save a PNG instead of opening an interactive window (e.g. headless capture).
python tools/analyze_bus_trace.py --team a --save logs/trace/bus_trace_latest.png

# Restrict to one joint (1-6) to declutter when chasing a single dial.
python tools/analyze_bus_trace.py --team a --joint 6
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
# Stable default written by tools/bus_trace_recorder.py.
DEFAULT_INPUT = REPO_ROOT / "logs" / "trace" / "bus_trace_latest.jsonl"

# Stage -> background shade colour for the timeline band. Unknown stages fall
# back to white (no shade). Colours are intentionally pale so the line plots
# stay readable on top of them.
_STAGE_COLORS = {
    "daydreaming": "#e8eaf6",
    "idle": "#e0f2f1",
    "tutorial": "#fff8e1",
    "play": "#e8f5e9",
    "reset": "#fbe9e7",
    "conclusion": "#f3e5f5",
    "paused": "#eeeeee",
    "(init)": "#ffffff",
}


def main(argv: list[str] | None = None) -> int:
    """Load one trace file and render the dial / override diagnostic figure."""

    args = _parse_args(argv)
    rows = _load_rows(Path(args.input))
    if not rows:
        print(f"[analyze] no rows loaded from {args.input}", flush=True)
        return 1

    # t0_ns is the wall-clock receive time of the first row; every plotted
    # series is expressed in seconds relative to it so the axes are readable.
    t0_ns = rows[0]["ts_recv_wall_ns"]

    haptic = _extract_haptic(rows, args.team, t0_ns, use_decideg=bool(args.decideg))
    override = _extract_override(rows, args.team, t0_ns)
    debug_events = _extract_jogging_debug(rows, args.team, t0_ns)
    stages = _extract_stages(rows, t0_ns)

    _print_summary(args.team, haptic, override, debug_events, stages)
    _plot(args, haptic, override, debug_events, stages)
    return 0


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse CLI arguments for the trace plotter."""

    parser = argparse.ArgumentParser(description="Plot a recorded bus trace.")
    parser.add_argument(
        "--input",
        default=str(DEFAULT_INPUT),
        help="JSONL trace path. Defaults to logs/trace/bus_trace_latest.jsonl.",
    )
    parser.add_argument(
        "--team",
        default="a",
        choices=("a", "b"),
        help="Team whose dial/override topics to plot. Default a.",
    )
    parser.add_argument(
        "--decideg",
        action="store_true",
        help=(
            "Plot the raw firmware dial_pos_decideg field (/10 -> deg) instead "
            "of dial_pos_rad, so integer glitches show without rad rounding."
        ),
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=0,
        help="Plot only this 1-based joint (1-6). 0 (default) plots all six.",
    )
    parser.add_argument(
        "--save",
        default=None,
        help="Save the figure to this PNG path instead of showing a window.",
    )
    return parser.parse_args(argv)


def _load_rows(path: Path) -> list[dict]:
    """Read the JSONL trace into a time-ordered list of decoded rows.

    Malformed or non-dict lines are skipped silently; rows missing a usable
    receive timestamp are dropped so the relative time axis stays monotonic.
    """

    if not path.exists():
        print(f"[analyze] trace file not found: {path}", flush=True)
        return []
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(row, dict):
                continue
            if not isinstance(row.get("ts_recv_wall_ns"), int):
                continue
            rows.append(row)
    rows.sort(key=lambda r: r["ts_recv_wall_ns"])
    return rows


def _extract_haptic(
    rows: list[dict], team: str, t0_ns: int, *, use_decideg: bool
) -> dict:
    """Pull per-joint dial positions (deg) and raw firmware fields over time.

    Returns a dict with:
      ``t``            : list of sample times (s, relative to t0)
      ``pos_deg``      : list[6] of per-joint position series (deg)
      ``seq``          : list[6] of per-joint firmware sequence series (or None)
      ``status``       : list[6] of per-joint firmware status_bits series
    """

    topic = f"telem.haptic.{team}"
    t: list[float] = []
    pos_deg: list[list[float]] = [[] for _ in range(6)]
    seq: list[list[float]] = [[] for _ in range(6)]
    status: list[list[float]] = [[] for _ in range(6)]
    for row in rows:
        if row.get("topic") != topic:
            continue
        body = row.get("body") or {}
        if use_decideg:
            raw = body.get("dial_pos_decideg")
            if isinstance(raw, list) and len(raw) >= 6:
                joints_deg = [float(v) / 10.0 for v in raw[:6]]
            else:
                continue
        else:
            rad = body.get("dial_pos_rad")
            if isinstance(rad, list) and len(rad) >= 6:
                joints_deg = [math.degrees(float(v)) for v in rad[:6]]
            else:
                continue
        t.append((row["ts_recv_wall_ns"] - t0_ns) / 1e9)
        for i in range(6):
            pos_deg[i].append(joints_deg[i])
        seq_list = body.get("dial_seq")
        status_list = body.get("dial_status_bits")
        for i in range(6):
            seq[i].append(
                float(seq_list[i]) if isinstance(seq_list, list) and i < len(seq_list) else float("nan")
            )
            status[i].append(
                float(status_list[i]) if isinstance(status_list, list) and i < len(status_list) else float("nan")
            )
    return {"t": t, "pos_deg": pos_deg, "seq": seq, "status": status}


def _extract_override(rows: list[dict], team: str, t0_ns: int) -> dict:
    """Pull the dense per-tick speed-override scalars from state.full.

    Returns ``t`` plus ``final``/``path``/``prox`` scalar series for the team.
    """

    t: list[float] = []
    final: list[float] = []
    path: list[float] = []
    prox: list[float] = []
    for row in rows:
        if row.get("topic") != "state.full":
            continue
        body = row.get("body") or {}
        team_block = ((body.get("teams") or {}).get(team) or {})
        coll = team_block.get("collision") or {}
        if not coll:
            continue
        t.append((row["ts_recv_wall_ns"] - t0_ns) / 1e9)
        final.append(_as_float(coll.get("final_scalar"), 1.0))
        path.append(_as_float(coll.get("path_scalar"), 1.0))
        prox.append(_as_float(coll.get("prox_scalar"), 1.0))
    return {"t": t, "final": final, "path": path, "prox": prox}


def _extract_jogging_debug(rows: list[dict], team: str, t0_ns: int) -> list[dict]:
    """Pull the sparse event-based jogging debug messages for the team.

    Each returned dict carries the event time (s) plus the raw debug body so
    the caller can annotate forward-timeout / collision events on the plot.
    """

    topic = f"telem.jogging.debug.{team}"
    events: list[dict] = []
    for row in rows:
        if row.get("topic") != topic:
            continue
        body = row.get("body") or {}
        events.append(
            {
                "t": (row["ts_recv_wall_ns"] - t0_ns) / 1e9,
                "reason": body.get("forward_stop_reason"),
                "final_scalar": _as_float(body.get("final_scalar"), 1.0),
                "wait_ms": _as_float(body.get("forward_wait_ms"), 0.0),
                "dispatched": body.get("forward_chunks_dispatched"),
                "replied": body.get("forward_chunks_replied"),
                "certified": body.get("forward_certified"),
                "stage": body.get("stage"),
            }
        )
    return events


def _extract_stages(rows: list[dict], t0_ns: int) -> list[tuple[float, str]]:
    """Return ordered (time_s, stage) edges from state.full for the timeline."""

    edges: list[tuple[float, str]] = []
    prev: str | None = None
    for row in rows:
        if row.get("topic") != "state.full":
            continue
        body = row.get("body") or {}
        # Prefer active_stage (the true machine stage) over the display stage,
        # which is overwritten with "paused" while paused.
        stage = body.get("active_stage") or body.get("stage")
        if not isinstance(stage, str):
            continue
        if stage != prev:
            edges.append(((row["ts_recv_wall_ns"] - t0_ns) / 1e9, stage))
            prev = stage
    return edges


def _print_summary(
    team: str,
    haptic: dict,
    override: dict,
    debug_events: list[dict],
    stages: list[tuple[float, str]],
) -> None:
    """Print a non-judgemental console summary (counts and ranges only)."""

    print(f"[analyze] team={team}", flush=True)
    if haptic["t"]:
        span = haptic["t"][-1] - haptic["t"][0]
        print(
            f"[analyze] haptic samples={len(haptic['t'])} span={span:.1f}s",
            flush=True,
        )
        for i in range(6):
            series = haptic["pos_deg"][i]
            if series:
                print(
                    f"[analyze]   J{i + 1}: min={min(series):.1f} "
                    f"max={max(series):.1f} deg",
                    flush=True,
                )
    else:
        print("[analyze] no haptic samples found", flush=True)

    if override["t"]:
        zeros = sum(1 for v in override["final"] if v <= 1e-6)
        print(
            f"[analyze] state.full ticks={len(override['t'])} "
            f"final_scalar==0 ticks={zeros}",
            flush=True,
        )
    print(f"[analyze] jogging debug events={len(debug_events)}", flush=True)
    reasons: dict[str, int] = {}
    for ev in debug_events:
        reasons[str(ev["reason"])] = reasons.get(str(ev["reason"]), 0) + 1
    for reason, count in sorted(reasons.items()):
        print(f"[analyze]   stop_reason {reason}: {count}", flush=True)
    if stages:
        names = " -> ".join(f"{s}@{t:.1f}s" for t, s in stages)
        print(f"[analyze] stage edges: {names}", flush=True)


def _plot(
    args: argparse.Namespace,
    haptic: dict,
    override: dict,
    debug_events: list[dict],
    stages: list[tuple[float, str]],
) -> None:
    """Render the three-panel diagnostic figure (dials / override / wait_ms)."""

    import matplotlib

    if args.save:
        matplotlib.use("Agg")  # headless backend when only saving a PNG
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    ax_dial, ax_ovr, ax_wait = axes

    _shade_stages(axes, stages)

    # --- Panel 1: dial positions over time -------------------------------
    joints = (
        [args.joint - 1] if 1 <= args.joint <= 6 else list(range(6))
    )
    for i in joints:
        if haptic["pos_deg"][i]:
            ax_dial.plot(
                haptic["t"], haptic["pos_deg"][i], label=f"J{i + 1}", linewidth=1.0
            )
    unit = "decideg/10" if args.decideg else "rad->deg"
    ax_dial.set_ylabel(f"dial pos (deg, {unit})")
    ax_dial.set_title(
        f"Team {args.team} dial positions over time "
        f"({'raw firmware' if args.decideg else 'published rad'})"
    )
    ax_dial.legend(loc="upper right", ncol=6, fontsize=8)
    ax_dial.grid(True, alpha=0.3)

    # --- Panel 2: speed-override scalar + debug-event markers ------------
    if override["t"]:
        ax_ovr.plot(override["t"], override["final"], color="#1565c0",
                    label="final_scalar", linewidth=1.0)
        ax_ovr.plot(override["t"], override["path"], color="#ef6c00",
                    label="path_scalar", linewidth=0.7, alpha=0.6)
        ax_ovr.plot(override["t"], override["prox"], color="#2e7d32",
                    label="prox_scalar", linewidth=0.7, alpha=0.6)
    # Mark forward-timeout events (the speed-override-to-zero cause we care
    # about most) distinctly from genuine collision blocks.
    _scatter_events(ax_ovr, debug_events, "forward_timeout", "red", "forward_timeout")
    _scatter_events(ax_ovr, debug_events, "collision_block", "black", "collision_block")
    ax_ovr.set_ylabel("speed override")
    ax_ovr.set_ylim(-0.05, 1.05)
    ax_ovr.legend(loc="upper right", fontsize=8)
    ax_ovr.grid(True, alpha=0.3)

    # --- Panel 3: forward-gate blocking wait (ms) per debug event --------
    if debug_events:
        ax_wait.scatter(
            [ev["t"] for ev in debug_events],
            [ev["wait_ms"] for ev in debug_events],
            s=12, color="#6a1b9a", label="forward_wait_ms",
        )
    ax_wait.set_ylabel("forward wait (ms)")
    ax_wait.set_xlabel("time since trace start (s)")
    ax_wait.grid(True, alpha=0.3)
    ax_wait.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"[analyze] saved figure to {args.save}", flush=True)
    else:
        plt.show()


def _shade_stages(axes, stages: list[tuple[float, str]]) -> None:
    """Shade each axis background by game stage across the time axis."""

    if not stages:
        return
    for idx, (start, stage) in enumerate(stages):
        end = stages[idx + 1][0] if idx + 1 < len(stages) else None
        color = _STAGE_COLORS.get(stage, "#ffffff")
        for ax in axes:
            ax.axvspan(start, end if end is not None else start + 1e6,
                       color=color, alpha=0.5, zorder=0)
    # Label each stage band once, on the top axis.
    top = axes[0]
    for start, stage in stages:
        top.text(start, 1.01, stage, transform=top.get_xaxis_transform(),
                 fontsize=7, rotation=90, va="bottom", color="#555555")


def _scatter_events(ax, events: list[dict], reason: str, color: str, label: str) -> None:
    """Scatter the final_scalar of every debug event matching ``reason``."""

    xs = [ev["t"] for ev in events if ev["reason"] == reason]
    ys = [ev["final_scalar"] for ev in events if ev["reason"] == reason]
    if xs:
        ax.scatter(xs, ys, s=18, color=color, label=label, zorder=5)


def _as_float(value, default: float) -> float:
    """Coerce a JSON value to float, returning ``default`` on failure/None."""

    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


if __name__ == "__main__":
    raise SystemExit(main())
