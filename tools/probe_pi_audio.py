"""Throwaway connectivity probe for one Raspberry Pi microphone process.

Exercises the OSC control protocol documented in
C:/Users/yck01/GitHub/voiceAnonymizer_PI/docs/AUDIO_INTEGRATION_FOR_GAME_CONTROLLER.md
using that repo's own speech_control.py helpers (no hand-rolled OSC):
send a short log_start -> log_save_stop session on ONE mic, then SSH in with
paramiko to list the files the session produced.

Run:
    & C:/Users/yck01/miniconda3/envs/game/python.exe tools/probe_pi_audio.py
    & C:/Users/yck01/miniconda3/envs/game/python.exe tools/probe_pi_audio.py --ip 192.168.0.11 --port 9001 --seconds 5 --audio
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# speech_control.py lives in the voiceAnonymizer_PI repo, not this one.
VOICE_REPO = Path("C:/Users/yck01/GitHub/voiceAnonymizer_PI")
sys.path.insert(0, str(VOICE_REPO))

import paramiko  # noqa: E402
import yaml  # noqa: E402
from speech_control import parse_target_endpoint, send_ctrl  # noqa: E402

PROBE_DAY = "2099-01-01"    # clearly-fake probe session identity
PROBE_TIME = "99-99-99"
REMOTE_ROOT = "/home/pi/SPEECH_RECORD_ANALYSIS/SESSION_LOGS"  # Pi-side session root
SECRETS = Path("C:/Users/yck01/GitHub/robot_game_controller/config/secrets.yaml")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.0.11", help="Pi IP (default rpi5-11).")
    parser.add_argument("--port", type=int, default=9001, help="Mic control port (9001=MIC1, 9002=MIC2).")
    parser.add_argument("--seconds", type=float, default=5.0, help="Record duration before save_stop.")
    parser.add_argument("--audio", action="store_true", help="Also capture raw audio.flac (record_audio=1).")
    ns = parser.parse_args()

    creds = yaml.safe_load(SECRETS.read_text())["pi_ssh"]
    target = parse_target_endpoint(f"{ns.ip}:{ns.port}")
    mic = f"MIC{1 if ns.port == 9001 else 2}"
    record_audio = "1" if ns.audio else "0"

    print(f"Target mic process: {target.label if hasattr(target, 'label') else target} ({ns.ip}:{ns.port})")

    # --- 0. Clear any leftover open session so the probe starts clean ---
    pre = send_ctrl(target, "query_state", [], ack_timeout=2.0)
    if pre.ok and "log_open=1" in (pre.message or ""):
        print("leftover open session detected; discarding it first ...")
        send_ctrl(target, "log_discard_stop", [], ack_timeout=5.0)

    # --- 1. Start a short session (analysis CSVs only unless --audio) ---
    # First command after a service restart can be slow; use a generous timeout.
    ack = send_ctrl(target, "log_start", [PROBE_DAY, PROBE_TIME, "5", record_audio], ack_timeout=5.0)
    elapsed = f"{ack.elapsed_ms:.0f} ms" if ack.elapsed_ms is not None else "n/a"
    print(f"log_start -> ok={ack.ok} msg={ack.message!r} timed_out={ack.timed_out} ({elapsed})")
    if not ack.ok:
        print("START FAILED; aborting probe.")
        return

    print(f"Recording for {ns.seconds} s ...")
    time.sleep(ns.seconds)

    # --- 2. Test idempotent duplicate start (same identity must be a no-op) ---
    dup = send_ctrl(target, "log_start", [PROBE_DAY, PROBE_TIME, "5", record_audio], ack_timeout=5.0)
    print(f"duplicate log_start (idempotency check) -> ok={dup.ok} msg={dup.message!r} timed_out={dup.timed_out}")

    # --- 3. Save + stop (now async: expect an immediate "saving" ack) ---
    t0 = time.time()
    ack = send_ctrl(target, "log_save_stop", [], ack_timeout=15.0)
    first_ms = (time.time() - t0) * 1000
    elapsed = f"{ack.elapsed_ms:.0f} ms" if ack.elapsed_ms is not None else "n/a"
    print(f"log_save_stop -> ok={ack.ok} msg={ack.message!r} timed_out={ack.timed_out} "
          f"(first ack after {first_ms:.0f} ms; server-reported {elapsed})")

    # --- 4. SSH in and list what the session wrote ---
    remote_dir = f"{REMOTE_ROOT}/{PROBE_DAY}/{PROBE_TIME}/{mic}"
    print(f"SSH listing {ns.ip}:{remote_dir} ...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(ns.ip, username=creds["username"], password=creds["password"], timeout=10)
        _, stdout, stderr = ssh.exec_command(f"ls -la {remote_dir} && echo --- && du -sh {remote_dir}")
        print(stdout.read().decode())
        err = stderr.read().decode().strip()
        if err:
            print("stderr:", err)
    except Exception as exc:  # noqa: BLE001 - probe: report anything
        print(f"SSH FAILED: {exc}")
    finally:
        ssh.close()


if __name__ == "__main__":
    main()
