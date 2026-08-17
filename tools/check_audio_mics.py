"""Audio microphone fleet health check.

Records a short session on ALL 12 Pi microphones (prosody + raw audio), then
stops them, pulls the files back, and prints a clear per-mic PASS/FAIL table so
you can see at a glance which microphones are healthy and which are dead.

This is the tool to run after reseating/swapping a USB microphone. It uses the
same device clients as the external_media_coordinator (single source of truth
for the OSC + SFTP protocol), and it fans the start/stop commands out
CONCURRENTLY across all 12 mics -- never sequentially. (A save holds a Pi's
single-threaded OSC listener while it flushes FLAC to SD, so a sequential loop
would stack delays and skew each mic's recording window. See
EXTERNAL_RECORDING_PLAN.md section 1 "Concurrent start/stop".)

A mic is reported PASS only if it produced BOTH a non-empty audio.flac AND a
non-empty opensmile_lld.csv for the session. A mic whose process answers OSC
but whose USB mic is not streaming (audio=failure / "no audio callbacks")
produces a 0-byte flac and no prosody -> FAIL.

The mic -> (team, player) mapping is read from config/device_ports_and_addr.yaml
(the `audio_capture:` block). Pi SSH credentials come from config/secrets.yaml.

Pulled files are written under a temp dir that is deleted on success unless
--keep is passed.

Run (from the repo root, game conda env):
    conda activate game
    $env:PYTHONPATH = "src"

    # 6-second health check across all 12 mics:
    python tools/check_audio_mics.py

    # Longer window, keep the pulled files for inspection:
    python tools/check_audio_mics.py --seconds 30 --keep

    # Check only one Pi (both its mics):
    python tools/check_audio_mics.py --pi 192.168.0.11
"""

from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import yaml  # noqa: E402

from core.device_connection import load_audio_capture  # noqa: E402
from core.external_media import ExternalMediaConfig  # noqa: E402
from apps.external_media_coordinator.devices import PiMicClient  # noqa: E402

SECRETS_PATH = REPO_ROOT / "config" / "secrets.yaml"


def _load_ssh_creds() -> dict:
    """Read the Pi SSH username/password from config/secrets.yaml."""
    data = yaml.safe_load(SECRETS_PATH.read_text(encoding="utf-8")) or {}
    return data.get("pi_ssh", {"username": "pi", "password": ""})


def _make_config(seconds: float) -> ExternalMediaConfig:
    """Build a minimal ExternalMediaConfig for the mic fleet (raw audio ON).

    Only the fields PiMicClient reads are populated; the rest are inert here.
    The mic fleet itself comes from device_ports_and_addr.yaml.
    """
    ac = load_audio_capture()
    return ExternalMediaConfig(
        enabled=True,
        jetson_host="",
        jetson_ws_port=0,
        jetson_http_port=0,
        keep_raw_video=False,
        keep_raw_audio=True,            # we want audio.flac for the health check
        max_minutes=max(1, int(seconds / 60) + 2),  # in-RAM cap headroom
        ack_timeout_s=ac.ack_timeout_s,
        emotion=False,
        mics=ac.mics,
        pull_timeout_s=30.0,
        retry_interval_s=0.0,
        retry_window_s=0.0,
        use_sim=False,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--seconds", type=float, default=6.0,
                        help="Recording length in seconds (default 6).")
    parser.add_argument("--pi", default=None, metavar="IP",
                        help="Limit the check to one Pi by IP (both its mics).")
    parser.add_argument("--keep", action="store_true",
                        help="Keep the pulled files instead of deleting them.")
    ns = parser.parse_args()

    cfg = _make_config(ns.seconds)
    creds = _load_ssh_creds()

    mics = cfg.mics
    if ns.pi:
        mics = tuple(m for m in mics if m.host == ns.pi)
        if not mics:
            print(f"ERROR: no mics configured for Pi {ns.pi} "
                  f"(check config/device_ports_and_addr.yaml audio_capture.hosts)")
            return 2

    # One client per mic process.
    clients = [PiMicClient(mic, cfg, creds) for mic in mics]

    # Session identity from the local clock (matches the recorder convention).
    now = time.localtime()
    day = time.strftime("%Y-%m-%d", now)
    tim = time.strftime("%H-%M-%S", now)

    dest = Path(tempfile.mkdtemp(prefix=f"mic_check_{day}_{tim}_"))

    print("=" * 64)
    print(f"Audio mic health check: {len(clients)} mics, {ns.seconds:.0f}s recording")
    print(f"session {day}/{tim}")
    print("=" * 64)

    pool = ThreadPoolExecutor(max_workers=max(4, len(clients)))

    # ---- start all concurrently ------------------------------------------
    print("\n[1/3] Starting all mics concurrently ...")
    start_ok = dict(zip(
        (c.device_key for c in clients),
        pool.map(lambda c: c.start(day, tim), clients),
    ))
    for c in clients:
        print(f"    {c.device_key:22s} start -> {'ok' if start_ok[c.device_key] else 'NO ACK'}")

    print(f"\n[2/3] Recording for {ns.seconds:.0f}s ...")
    time.sleep(ns.seconds)

    # ---- stop all concurrently -------------------------------------------
    print("[3/3] Stopping all mics concurrently, then pulling files ...")
    stop_ok = dict(zip(
        (c.device_key for c in clients),
        pool.map(lambda c: c.stop(), clients),
    ))
    time.sleep(3.0)  # let the async on-device saves flush to SD before pulling

    # ---- pull all concurrently -------------------------------------------
    results = dict(zip(
        (c.device_key for c in clients),
        pool.map(lambda c: c.pull(dest, day, tim), clients),
    ))
    pool.shutdown()

    # ---- per-mic verdict ---------------------------------------------------
    print("\n" + "=" * 64)
    print("RESULTS")
    print("=" * 64)
    print(f"{'player':8s} {'device':22s} {'flac':>10s} {'prosody':>9s}  verdict")
    print("-" * 64)
    n_pass = 0
    for c in clients:
        mic = c.t
        res = results[c.device_key]
        flac = next((f for f in res.files if f.rel_path.endswith("audio.flac")), None)
        prosody = next((f for f in res.files if "opensmile_lld" in f.rel_path), None)
        flac_bytes = flac.nbytes if (flac and flac.ok) else 0
        prosody_ok = bool(prosody and prosody.ok and prosody.nbytes > 0)
        started = start_ok[c.device_key]
        stopped = stop_ok[c.device_key]

        if not started:
            verdict = "FAIL (no start ack)"
        elif not stopped:
            verdict = "FAIL (no stop ack)"
        elif flac_bytes > 0 and prosody_ok:
            verdict = "PASS"
            n_pass += 1
        elif flac_bytes == 0 and not prosody_ok:
            verdict = "FAIL (dead mic: no audio)"
        else:
            verdict = "FAIL (partial)"
        print(f"{mic.player_label:8s} {c.device_key:22s} "
              f"{flac_bytes:>9d}B {'yes' if prosody_ok else 'NO':>9s}  {verdict}")

    print("-" * 64)
    print(f"{n_pass}/{len(clients)} mics healthy")
    if ns.keep:
        print(f"pulled files kept at: {dest}")
    else:
        shutil.rmtree(dest, ignore_errors=True)
    return 0 if n_pass == len(clients) else 1


if __name__ == "__main__":
    raise SystemExit(main())
