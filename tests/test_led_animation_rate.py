"""Visual animation frame-rate test for one or two LED strips.

This script is intended for hardware validation with a real controller.
It sends explicit frames at a target FPS and prints achieved rates.

Scenarios:
  - single: animate one strip with a bouncing fill pattern
  - dual-same: animate two strips with the same fill pattern
  - dual-different: animate two strips with opposite fill and different colors

Examples:
  python tests/test_led_animation_rate.py --port COM19 --mode single --strip-a 11 --fps 20
  python tests/test_led_animation_rate.py --port COM19 --mode dual-same --strip-a 11 --strip-b 12 --fps 15
  python tests/test_led_animation_rate.py --port COM19 --mode dual-different --strip-a 11 --strip-b 12 --fps 30
  python tests/test_led_animation_rate.py --port COM19 --mode dual-different --strip-a 11 --strip-b 12 --fps 60 --duration-s 150

Max A speed achieved during testing with 921600 baud and 2 ms inter-command delay:
  python tests/test_led_animation_rate.py --port COM19 --mode all-strips --fps 20 --inter-cmd-ms 2 --duration-s 10 --cycle-s 2
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from led_serial import LEDCommandBuilder, RS485Connection, Color, RED, BLUE, OFF

LEDS_PER_STRIP = 28
ALL_STRIP_IDS = [11, 12, 21, 22, 31, 32, 41, 42, 51, 52, 61, 62, 71, 72, 81, 82]


@dataclass
class Stats:
    frame_count: int = 0
    cmd_count: int = 0
    late_frames: int = 0
    send_time_s: float = 0.0
    enforced_wait_s: float = 0.0
    min_cmd_gap_s: Optional[float] = None
    max_cmd_gap_s: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LED animation frame-rate validation")
    parser.add_argument("--port", type=str, required=True, help="Serial COM port, e.g. COM19")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default 921600)")
    parser.add_argument(
        "--mode",
        type=str,
        default="single",
        choices=["single", "dual-same", "dual-different", "all-strips"],
        help="Animation scenario",
    )
    parser.add_argument("--strip-a", type=int, default=11, help="Primary strip ID")
    parser.add_argument("--strip-b", type=int, default=12, help="Secondary strip ID")
    parser.add_argument("--fps", type=float, default=15.0, help="Requested frame rate")
    parser.add_argument("--duration-s", type=float, default=12.0, help="Test duration in seconds")
    parser.add_argument(
        "--inter-cmd-ms",
        type=float,
        default=2.0,
        help="Delay used by gap-mode policy (921600 baud: 2 ms reliable, 1 ms unstable)",
    )
    parser.add_argument(
        "--gap-mode",
        type=str,
        default="between-all",
        choices=["between-all", "between-strips"],
        help=(
            "between-all: enforce delay between every command (safer). "
            "between-strips: enforce delay only between commands inside the same frame (faster)."
        ),
    )
    parser.add_argument(
        "--cycle-s",
        type=float,
        default=None,
        help="Total triangle-wave cycle time in seconds (0 -> full -> 0)",
    )
    parser.add_argument(
        "--phase-s",
        type=float,
        default=2.5,
        help="Seconds for one half-cycle (0 -> full or full -> 0). Ignored if --cycle-s is set.",
    )
    parser.add_argument(
        "--hard-steps",
        action="store_true",
        help="Disable fractional edge LED smoothing (uses integer fill only)",
    )
    return parser.parse_args()


def make_fill(count: int, on: Color, off: Color) -> list[Color]:
    count = max(0, min(LEDS_PER_STRIP, count))
    return [on] * count + [off] * (LEDS_PER_STRIP - count)


def blend_color(a: Color, b: Color, t: float) -> Color:
    t = max(0.0, min(1.0, t))
    return Color(
        r=int(a.r + (b.r - a.r) * t),
        g=int(a.g + (b.g - a.g) * t),
        b=int(a.b + (b.b - a.b) * t),
    )


def make_fill_smooth(position: float, on: Color, off: Color) -> list[Color]:
    """Create a fill with a fractional leading LED for smoother motion."""
    pos = max(0.0, min(float(LEDS_PER_STRIP), position))
    full = int(math.floor(pos))
    frac = pos - full

    leds = [off] * LEDS_PER_STRIP
    for i in range(full):
        leds[i] = on
    if full < LEDS_PER_STRIP and frac > 0.0:
        leds[full] = blend_color(off, on, frac)
    return leds


def bouncing_fill_count(t_s: float, phase_s: float) -> int:
    """Return 0..28..0 over time using a triangle wave."""
    phase_s = max(0.2, phase_s)
    cycle = 2.0 * phase_s
    p = (t_s % cycle) / cycle  # 0..1
    tri = 1.0 - abs(2.0 * p - 1.0)  # 0..1..0
    return int(round(tri * LEDS_PER_STRIP))


def bouncing_fill_position(t_s: float, phase_s: float) -> float:
    """Return smooth 0..28..0 position over time (triangle wave)."""
    phase_s = max(0.2, phase_s)
    cycle = 2.0 * phase_s
    p = (t_s % cycle) / cycle  # 0..1
    tri = 1.0 - abs(2.0 * p - 1.0)  # 0..1..0
    return tri * LEDS_PER_STRIP


def print_budget(
    frame_len: int,
    baud: int,
    inter_cmd_ms: float,
    strips_per_frame: int,
    fps: float,
    gap_mode: str,
) -> None:
    tx_ms = (frame_len * 10.0 / baud) * 1000.0
    gap_ms = max(0.0, inter_cmd_ms)

    # Model A: delay between every command (safest pacing)
    frame_ms_all = strips_per_frame * (tx_ms + gap_ms)
    max_fps_all = 1000.0 / frame_ms_all

    # Model B: delay only between strips inside frame (higher throughput)
    frame_ms_between = strips_per_frame * tx_ms + max(0, strips_per_frame - 1) * gap_ms
    max_fps_between = 1000.0 / frame_ms_between

    print("Timing budget:")
    print(f"  - Frame size: {frame_len} bytes")
    print(f"  - Tx time per command: {tx_ms:.2f} ms @ {baud} baud")
    print(f"  - Configured gap:      {inter_cmd_ms:.2f} ms")
    print(f"  - Commands per frame:  {strips_per_frame}")
    print(f"  - Theoretical max FPS (between-all):    {max_fps_all:.1f}")
    print(f"  - Theoretical max FPS (between-strips): {max_fps_between:.1f}")
    print(f"  - Active gap mode:                    {gap_mode}")
    print(f"  - Requested frame rate:         {fps:.1f} FPS")
    limit = max_fps_all if gap_mode == "between-all" else max_fps_between
    if fps > limit:
        print("  - WARNING: Requested FPS exceeds budget for active gap mode.")


def print_measured_bus_analysis(
    frame_len: int,
    baud: int,
    avg_host_send_ms: float,
    avg_enforced_wait_ms: float,
    measured_cmd_rate_hz: float,
) -> None:
    """Explain why measured command rate can exceed the naive wire-time + sleep sum."""
    wire_time_ms = (frame_len * 10.0 / baud) * 1000.0
    buffered_overlap_ms = max(0.0, wire_time_ms - avg_host_send_ms)
    estimated_idle_gap_ms = max(0.0, avg_enforced_wait_ms - buffered_overlap_ms)
    estimated_cmd_period_ms = wire_time_ms + estimated_idle_gap_ms
    estimated_cmd_rate_hz = 1000.0 / estimated_cmd_period_ms if estimated_cmd_period_ms > 0 else 0.0

    print("\nMeasured bus analysis:")
    print(f"  - Wire time per command:           {wire_time_ms:.2f} ms")
    print(f"  - Avg host send()+flush time:      {avg_host_send_ms:.2f} ms")
    print(f"  - Estimated buffered overlap:      {buffered_overlap_ms:.2f} ms")
    print(f"  - Avg enforced wait after send:    {avg_enforced_wait_ms:.2f} ms")
    print(f"  - Estimated real idle gap on bus:  {estimated_idle_gap_ms:.2f} ms")
    print(f"  - Estimated command period:        {estimated_cmd_period_ms:.2f} ms")
    print(f"  - Estimated command rate:          {estimated_cmd_rate_hz:.2f} Hz")
    print(f"  - Measured command rate:           {measured_cmd_rate_hz:.2f} Hz")


def resolve_phase_s(args: argparse.Namespace) -> float:
    """Resolve the triangle-wave half-cycle duration from CLI args."""
    if args.cycle_s is not None:
        return max(0.1, args.cycle_s / 2.0)
    return max(0.1, args.phase_s)


def send_strip(conn: RS485Connection, strip_id: int, colors: list[Color]) -> bool:
    frame = LEDCommandBuilder.build_strip_command(strip_id, colors)
    return conn.send(frame)


def strip_phase_offset(strip_index: int, strip_count: int) -> float:
    """Spread strips evenly across the animation phase."""
    if strip_count <= 0:
        return 0.0
    return strip_index / strip_count


def build_strip_frame_colors(
    strip_index: int,
    strip_count: int,
    elapsed: float,
    phase_s: float,
    hard_steps: bool,
) -> list[Color]:
    """Create a per-strip phased pattern for full-bus tests."""
    offset = strip_phase_offset(strip_index, strip_count) * phase_s
    pos = bouncing_fill_position(elapsed + offset, phase_s)
    count = bouncing_fill_count(elapsed + offset, phase_s)

    # Alternate colors by strip index so strips 11/12 visibly differ.
    on_color = RED if strip_index % 2 == 0 else BLUE
    if hard_steps:
        return make_fill(count, on_color, OFF)
    return make_fill_smooth(pos, on_color, OFF)


def send_with_policy(
    conn: RS485Connection,
    strip_id: int,
    colors: list[Color],
    cmd_idx_in_frame: int,
    args: argparse.Namespace,
    stats: Stats,
    last_cmd_end_s: Optional[float],
) -> tuple[bool, Optional[float]]:
    """Send one strip command while enforcing selected gap policy."""
    inter_cmd_s = max(0.0, args.inter_cmd_ms / 1000.0)

    if args.gap_mode == "between-all":
        if last_cmd_end_s is not None and inter_cmd_s > 0.0:
            now = time.perf_counter()
            needed = inter_cmd_s - (now - last_cmd_end_s)
            if needed > 0.0:
                time.sleep(needed)
                stats.enforced_wait_s += needed
    elif args.gap_mode == "between-strips":
        if cmd_idx_in_frame > 0 and inter_cmd_s > 0.0:
            time.sleep(inter_cmd_s)
            stats.enforced_wait_s += inter_cmd_s

    send_start = time.perf_counter()
    if last_cmd_end_s is not None:
        cmd_gap = send_start - last_cmd_end_s
        if stats.min_cmd_gap_s is None or cmd_gap < stats.min_cmd_gap_s:
            stats.min_cmd_gap_s = cmd_gap
        stats.max_cmd_gap_s = max(stats.max_cmd_gap_s, cmd_gap)

    ok = send_strip(conn, strip_id, colors)
    send_end = time.perf_counter()
    stats.send_time_s += max(0.0, send_end - send_start)
    stats.cmd_count += 1
    return ok, send_end


def run(args: argparse.Namespace) -> int:
    phase_s = resolve_phase_s(args)
    cycle_s = 2.0 * phase_s
    frame_interval = 1.0 / max(1.0, args.fps)

    if args.mode == "single":
        strips_per_frame = 1
    elif args.mode in ("dual-same", "dual-different"):
        strips_per_frame = 2
    else:
        strips_per_frame = len(ALL_STRIP_IDS)
    sample = LEDCommandBuilder.build_strip_command(args.strip_a, make_fill(0, RED, OFF))
    print_budget(
        len(sample),
        args.baud,
        args.inter_cmd_ms,
        strips_per_frame,
        args.fps,
        args.gap_mode,
    )
    print(f"Animation timing:")
    print(f"  - Half-cycle (0 -> full): {phase_s:.2f} s")
    print(f"  - Full cycle (0 -> full -> 0): {cycle_s:.2f} s")

    conn = RS485Connection(port=args.port, baudrate=args.baud)
    if not conn.open():
        print("FAILED: Could not open serial port")
        return 1

    stats = Stats()
    start = time.perf_counter()
    next_tick = start
    last_cmd_end_s: Optional[float] = None

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start
            if elapsed >= args.duration_s:
                break

            pos_a = bouncing_fill_position(elapsed, phase_s)
            pos_b = bouncing_fill_position(elapsed + (phase_s / 2.0), phase_s)
            count_a = bouncing_fill_count(elapsed, phase_s)
            count_b = bouncing_fill_count(elapsed + (phase_s / 2.0), phase_s)

            if args.mode == "single":
                colors_a = (
                    make_fill(count_a, RED, OFF)
                    if args.hard_steps
                    else make_fill_smooth(pos_a, RED, OFF)
                )
                ok, last_cmd_end_s = send_with_policy(
                    conn,
                    args.strip_a,
                    colors_a,
                    cmd_idx_in_frame=0,
                    args=args,
                    stats=stats,
                    last_cmd_end_s=last_cmd_end_s,
                )
                if not ok:
                    print("FAILED: write error on strip A")
                    return 2

            elif args.mode == "dual-same":
                colors = (
                    make_fill(count_a, RED, OFF)
                    if args.hard_steps
                    else make_fill_smooth(pos_a, RED, OFF)
                )
                ok, last_cmd_end_s = send_with_policy(
                    conn,
                    args.strip_a,
                    colors,
                    cmd_idx_in_frame=0,
                    args=args,
                    stats=stats,
                    last_cmd_end_s=last_cmd_end_s,
                )
                if not ok:
                    print("FAILED: write error on strip A")
                    return 3
                ok, last_cmd_end_s = send_with_policy(
                    conn,
                    args.strip_b,
                    colors,
                    cmd_idx_in_frame=1,
                    args=args,
                    stats=stats,
                    last_cmd_end_s=last_cmd_end_s,
                )
                if not ok:
                    print("FAILED: write error on strip B")
                    return 4

            elif args.mode == "dual-different":
                colors_a = (
                    make_fill(count_a, RED, OFF)
                    if args.hard_steps
                    else make_fill_smooth(pos_a, RED, OFF)
                )
                inv_b = LEDS_PER_STRIP - count_b if args.hard_steps else LEDS_PER_STRIP - pos_b
                colors_b = (
                    make_fill(int(inv_b), BLUE, OFF)
                    if args.hard_steps
                    else make_fill_smooth(inv_b, BLUE, OFF)
                )
                ok, last_cmd_end_s = send_with_policy(
                    conn,
                    args.strip_a,
                    colors_a,
                    cmd_idx_in_frame=0,
                    args=args,
                    stats=stats,
                    last_cmd_end_s=last_cmd_end_s,
                )
                if not ok:
                    print("FAILED: write error on strip A")
                    return 5
                ok, last_cmd_end_s = send_with_policy(
                    conn,
                    args.strip_b,
                    colors_b,
                    cmd_idx_in_frame=1,
                    args=args,
                    stats=stats,
                    last_cmd_end_s=last_cmd_end_s,
                )
                if not ok:
                    print("FAILED: write error on strip B")
                    return 6

            else:  # all-strips
                for strip_index, strip_id in enumerate(ALL_STRIP_IDS):
                    colors = build_strip_frame_colors(
                        strip_index=strip_index,
                        strip_count=len(ALL_STRIP_IDS),
                        elapsed=elapsed,
                        phase_s=phase_s,
                        hard_steps=args.hard_steps,
                    )
                    ok, last_cmd_end_s = send_with_policy(
                        conn,
                        strip_id,
                        colors,
                        cmd_idx_in_frame=strip_index,
                        args=args,
                        stats=stats,
                        last_cmd_end_s=last_cmd_end_s,
                    )
                    if not ok:
                        print(f"FAILED: write error on strip {strip_id}")
                        return 7

            stats.frame_count += 1
            next_tick += frame_interval
            delay = next_tick - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                stats.late_frames += 1
                next_tick = time.perf_counter()

        runtime = max(1e-6, time.perf_counter() - start)
        print("\nResults:")
        print(f"  - Runtime:        {runtime:.2f} s")
        print(f"  - Frames sent:    {stats.frame_count}")
        print(f"  - Commands sent:  {stats.cmd_count}")
        print(f"  - Achieved FPS:   {stats.frame_count / runtime:.2f}")
        print(f"  - Cmd rate:       {stats.cmd_count / runtime:.2f} Hz")
        print(f"  - Late frames:    {stats.late_frames}")
        measured_cmd_rate_hz = stats.cmd_count / runtime
        if stats.cmd_count > 0:
            avg_host_send_ms = (stats.send_time_s * 1000.0) / stats.cmd_count
            avg_enforced_wait_ms = (stats.enforced_wait_s * 1000.0) / stats.cmd_count
            print(f"  - Avg host send() time: {(stats.send_time_s * 1000.0) / stats.cmd_count:.2f} ms")
            print(
                f"  - Avg enforced wait: {(stats.enforced_wait_s * 1000.0) / stats.cmd_count:.2f} ms"
            )
            print_measured_bus_analysis(
                frame_len=len(sample),
                baud=args.baud,
                avg_host_send_ms=avg_host_send_ms,
                avg_enforced_wait_ms=avg_enforced_wait_ms,
                measured_cmd_rate_hz=measured_cmd_rate_hz,
            )
        if stats.min_cmd_gap_s is not None:
            print(f"  - Min observed cmd gap: {stats.min_cmd_gap_s * 1000.0:.2f} ms")
            print(f"  - Max observed cmd gap: {stats.max_cmd_gap_s * 1000.0:.2f} ms")
        return 0

    finally:
        # Leave strips in OFF state.
        try:
            strip_ids = [args.strip_a]
            if args.mode in ("dual-same", "dual-different"):
                strip_ids.append(args.strip_b)
            elif args.mode == "all-strips":
                strip_ids = ALL_STRIP_IDS

            off_colors = make_fill(0, OFF, OFF)
            for idx, strip_id in enumerate(strip_ids):
                send_strip(conn, strip_id, off_colors)
                if idx < len(strip_ids) - 1:
                    time.sleep(max(0.0, args.inter_cmd_ms / 1000.0))
        finally:
            conn.close()


def main() -> int:
    args = parse_args()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
