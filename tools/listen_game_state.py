#!/usr/bin/env python3
"""Standalone UDP broadcast listener for robot game state.

Copy this single file to any machine on the same network and run:

    python listen_game_state.py

No dependencies beyond the Python standard library.

Press Ctrl-C to exit.
"""

import json
import socket
import time
import sys
import os

# ---------------------------------------------------------------------------
# Configuration — edit these if you changed them in GameSettings
# ---------------------------------------------------------------------------
PORT = 9000  # must match GameSettings.broadcast_port
STALE_S = 1.0  # seconds without a packet before "NO SIGNAL" is shown
# ---------------------------------------------------------------------------


def clear():
    """Clear the terminal screen."""
    os.system("cls" if sys.platform == "win32" else "clear")


def bars(value: float, lo: float, hi: float, width: int = 20) -> str:
    """Return a simple ASCII bar representing value in [lo, hi]."""
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo))) if hi != lo else 0.0
    filled = round(frac * width)
    return "[" + "#" * filled + "-" * (width - filled) + f"] {value:7.2f}"


def render(state: dict, last_rx: float, rx_count: int, rx_hz: float):
    """Print a formatted snapshot to stdout."""
    clear()
    age = time.time() - last_rx
    fresh = age < STALE_S

    print("=" * 60)
    print("  Robot Game Controller — State Listener")
    print(f"  Listening on UDP port {PORT}")
    print("=" * 60)

    if not fresh:
        print(f"\n  *** NO SIGNAL  (last packet {age:.1f}s ago) ***\n")
        return

    v = state.get("v", "?")
    ts = state.get("ts", 0.0)
    lag = time.time() - ts

    print(
        f"  Protocol v{v}   Packets: {rx_count}   Rate: {rx_hz:.1f} Hz   Lag: {lag*1000:.1f} ms"
    )
    print()

    # --- Stage ---
    stage = state.get("stage", "?")
    countdown = state.get("countdown_s", 0)
    estop = state.get("estop", False)
    estop_str = "  *** EMERGENCY STOP ***" if estop else ""
    print(f"  Stage: {stage:<12}  Countdown: {countdown:3d}s{estop_str}")
    print()

    # --- Joints ---
    print("  Joints  (dial → robot, degrees)")
    print(f"  {'ID':<4}  {'Dial':>7}  {'Robot':>7}  {'Dial':>22}  {'Robot':>22}")
    print("  " + "-" * 72)
    joints = state.get("joints", {})
    for mid in ["11", "12", "13", "14", "15", "16"]:
        j = joints.get(mid, {})
        d = j.get("dial_deg", 0.0)
        r = j.get("robot_deg", 0.0)
        print(
            f"  M{mid}  {d:+7.2f}°  {r:+7.2f}°  {bars(d,-180,180)}  {bars(r,-180,180)}"
        )
    print()

    # --- Buckets & scores ---
    buckets = state.get("buckets", {})
    mults = state.get("multipliers", {})
    scores = state.get("scores", {})

    print("  Buckets (weight g  ×  multiplier  =  contribution)")
    print(f"  {'':6}  {'Weight':>8}  {'Mult':>6}  {'Contrib':>10}  {'Bar':>22}")
    print("  " + "-" * 60)
    for team, ids in [("Team 1", ["11", "12", "13"]), ("Team 2", ["21", "22", "23"])]:
        print(f"  {team}")
        for bid in ids:
            w = buckets.get(bid, 0.0)
            m = mults.get(bid, 1.0)
            c = w * m
            print(f"    B{bid}    {w:8.1f}g  ×{m:5.1f}  = {c:9.1f}  {bars(w, 0, 500)}")

    print()
    t1 = scores.get("team1", 0.0)
    t2 = scores.get("team2", 0.0)
    hi = scores.get("high", 0.0)
    holder = scores.get("high_holder", "")
    print(f"  Score — Team 1: {t1:8.1f}   Team 2: {t2:8.1f}")
    print(f"  High Score: {hi:.1f}  ({holder})")
    print()

    # --- Health ---
    health = state.get("health", {})
    if health:
        print("  System Health")
        print(f"    Game loop:    {health.get('game_loop_hz', 0.0):.1f} Hz")
        print(f"    Robot physics:{health.get('robot_physics_hz', 0.0):.1f} Hz")
        print(f"    Publisher:    {health.get('publisher_hz', 0.0):.1f} Hz")
        print(f"    Haptic:       {health.get('haptic_connected', '?')}")
        print(f"    Weight sens:  {health.get('weight_sensors', '?')}")

    print()
    print("  Press Ctrl-C to exit.")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", PORT))
    except OSError as e:
        print(f"ERROR: Could not bind to port {PORT}: {e}")
        print("Make sure no other process is using this port.")
        sys.exit(1)

    sock.settimeout(0.5)

    print(f"Listening for game state on UDP port {PORT} ...")
    print("Waiting for first packet ...\n")

    state: dict = {}
    last_rx: float = 0.0
    rx_count: int = 0
    hz_count: int = 0
    hz_window_start: float = time.time()
    rx_hz: float = 0.0

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                # No packet — just re-render with stale indicator if needed
                if last_rx > 0:
                    render(state, last_rx, rx_count, rx_hz)
                continue

            try:
                packet = json.loads(data.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue  # malformed packet — ignore

            state = packet
            last_rx = time.time()
            rx_count += 1
            hz_count += 1

            # Measure receive Hz over 1-second windows
            elapsed = time.time() - hz_window_start
            if elapsed >= 1.0:
                rx_hz = hz_count / elapsed
                hz_count = 0
                hz_window_start = time.time()

            render(state, last_rx, rx_count, rx_hz)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
