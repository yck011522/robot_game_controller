"""Step-by-step RS485 communication test for LED controller.

Purpose:
  1) Verify serial port opens with expected UART settings.
  2) Verify command frame bytes match the known-working structure.
  3) Verify controller/address/channel path by sending a single solid color.
  4) Verify repeated command sending with configurable inter-frame gap.

Examples:
    python src/test_led_comm.py --port COM19 --baud 921600 --device 1 --channel 1 --color red
    python src/test_led_comm.py --port COM19 --baud 921600 --strip 11 --flash
"""

from __future__ import annotations

import argparse
import time
from typing import List

from led_serial import (
    LEDCommandBuilder,
    RS485Connection,
    Color,
    RED,
    GREEN,
    BLUE,
    WHITE,
    OFF,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RS485 LED communication diagnostic")
    parser.add_argument("--port", type=str, required=True, help="Serial COM port, e.g. COM19")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--strip", type=int, help="Logical strip ID (11,12,...,82)")
    mode.add_argument("--device", type=int, help="Device/controller address (1..8)")

    parser.add_argument("--channel", type=int, default=1, choices=[1, 2], help="Channel (1 or 2)")
    parser.add_argument("--group", type=int, default=0, help="Group address (default 0)")
    parser.add_argument("--repeat", type=int, default=1, help="Repeat field value (default 1)")

    parser.add_argument(
        "--color",
        type=str,
        default="red",
        choices=["red", "green", "blue", "white", "off"],
        help="Color for payload",
    )
    parser.add_argument(
        "--delay-ms",
        type=float,
        default=2.0,
        help="Delay between repeated commands in flash mode (921600 baud: 2 ms reliable, 1 ms unstable)",
    )
    parser.add_argument(
        "--flash",
        action="store_true",
        help="Flash color/off repeatedly after single frame test",
    )
    parser.add_argument(
        "--flash-cycles",
        type=int,
        default=8,
        help="Number of on/off cycles in flash mode",
    )
    return parser.parse_args()


def color_from_name(name: str) -> Color:
    mapping = {
        "red": RED,
        "green": GREEN,
        "blue": BLUE,
        "white": WHITE,
        "off": OFF,
    }
    return mapping[name]


def payload(color: Color) -> List[Color]:
    return [Color(color.r, color.g, color.b) for _ in range(28)]


def build_frame(args: argparse.Namespace, color: Color) -> bytes:
    colors = payload(color)
    if args.strip is not None:
        return LEDCommandBuilder.build_strip_command(args.strip, colors)
    return LEDCommandBuilder.build_device_channel_command(
        device_addr=args.device,
        channel=args.channel,
        colors=colors,
        group_addr=args.group,
        repeat=args.repeat,
    )


def summarize_frame(frame: bytes) -> None:
    print("Frame length:", len(frame), "bytes")
    print("Hex:", frame.hex(" "))


def print_timing_budget(frame_len: int, baud: int, delay_ms: float) -> None:
    """Print serial throughput estimates to guide reliable delay settings."""
    # UART 8N1 uses 10 bits on the wire per byte.
    bits_per_frame = frame_len * 10
    tx_time_ms = (bits_per_frame / baud) * 1000.0
    state_period_ms = tx_time_ms + max(0.0, delay_ms)
    state_rate_hz = 1000.0 / state_period_ms if state_period_ms > 0 else 0.0

    print("Timing budget:")
    print(f"  - On-wire tx time per frame: {tx_time_ms:.2f} ms @ {baud} baud")
    print(f"  - Added inter-frame delay:   {max(0.0, delay_ms):.2f} ms")
    print(f"  - Effective state period:    {state_period_ms:.2f} ms")
    print(f"  - Approx max state rate:     {state_rate_hz:.1f} Hz")

    if delay_ms < 7.0:
        print(
            "  - WARNING: Delay below 7 ms is known to be unreliable on this setup."
        )


def main() -> int:
    args = parse_args()
    test_color = color_from_name(args.color)

    print("[1/4] Building frame...")
    frame_on = build_frame(args, test_color)
    frame_off = build_frame(args, OFF)
    summarize_frame(frame_on)
    print_timing_budget(len(frame_on), args.baud, args.delay_ms)

    print("[2/4] Opening serial...")
    conn = RS485Connection(port=args.port, baudrate=args.baud)
    if not conn.open():
        print("FAILED: could not open serial port")
        return 1

    try:
        print("[3/4] Sending one frame...")
        if not conn.send(frame_on):
            print("FAILED: write error")
            return 2
        time.sleep(0.25)

        if args.flash:
            print("[4/4] Flash test (on/off) ...")
            gap_s = max(0.0, args.delay_ms / 1000.0)
            for i in range(args.flash_cycles):
                if not conn.send(frame_on):
                    print(f"FAILED: write error (cycle {i}, on)")
                    return 3
                time.sleep(gap_s)
                if not conn.send(frame_off):
                    print(f"FAILED: write error (cycle {i}, off)")
                    return 4
                time.sleep(gap_s)

        print("DONE: Serial write path completed")
        return 0
    finally:
        conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
