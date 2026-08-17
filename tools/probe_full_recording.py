"""Full-fidelity stress probe: record both Jetson (skeleton+video) and all
enabled Pi mics (prosody+raw audio) for a fixed duration, then stop, pull
every file back, and validate durations/sizes.

This mirrors exactly what the game controller's external_media_coordinator
will do at game end. Run on this PC; devices must be reachable on the LAN.

Run:
    # 2-minute pass (raw audio + raw video ON):
    & C:/Users/yck01/miniconda3/envs/game/python.exe tools/probe_full_recording.py --minutes 2
    # 6-minute pass:
    & C:/Users/yck01/miniconda3/envs/game/python.exe tools/probe_full_recording.py --minutes 6
    # Limit to one Pi mic / skip Jetson while iterating:
    & C:/Users/yck01/miniconda3/envs/game/python.exe tools/probe_full_recording.py --minutes 2 --pi 192.168.0.11 --no-jetson
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import threading
import time
import urllib.request
from pathlib import Path

import websockets

# speech_control.py lives in the voiceAnonymizer_PI repo, not this one.
sys.path.insert(0, "C:/Users/yck01/GitHub/voiceAnonymizer_PI")
import paramiko  # noqa: E402
import yaml  # noqa: E402
from speech_control import parse_target_endpoint, send_ctrl  # noqa: E402

JETSON_WS = "ws://192.168.0.101:9000"      # Jetson command channel
JETSON_HTTP = "http://192.168.0.101:9100"  # Jetson file server
SECRETS = Path("config/secrets.yaml")
DEST_ROOT = Path("logs/full_recording_probe")  # local landing zone for pulled files

# All six Pis x two mic control ports (MIC1=9001, MIC2=9002).
PIS = ["192.168.0.11", "192.168.0.12", "192.168.0.13",
       "192.168.0.14", "192.168.0.15", "192.168.0.16"]
MIC_PORTS = [9001, 9002]
REMOTE_ROOT = "/home/pi/SPEECH_RECORD_ANALYSIS/SESSION_LOGS"  # Pi-side session root


def session_identity() -> tuple[str, str]:
    """Return (LOG_DAY, LOG_TIME) from the local clock, matching the recorder's
    <date>/<time> folder convention so pulled files land under one folder."""
    now = time.localtime()
    return time.strftime("%Y-%m-%d", now), time.strftime("%H-%M-%S", now)


# --------------------------------------------------------------------------
# Jetson (WebSocket command + HTTP pull)
# --------------------------------------------------------------------------
async def jetson_cmd(ws, command: str, args: dict | None = None) -> dict:
    req_id = f"full-{command}-{time.time_ns()}"
    msg = {"type": "command", "command": command, "request_id": req_id}
    if args:
        msg["args"] = args
    await ws.send(json.dumps(msg))
    while True:
        reply = json.loads(await asyncio.wait_for(ws.recv(), timeout=20))
        if reply.get("request_id") == req_id:
            return reply


async def jetson_run(day: str, tim: str, seconds: float, record_video: bool) -> dict:
    """Start, hold, and stop the Jetson recording; return the stop ack."""
    async with websockets.connect(JETSON_WS) as ws:
        await jetson_cmd(ws, "ping")
        start = await jetson_cmd(ws, "start_recording", {
            "date": day, "time": tim,
            "record_skeleton": True, "record_video": record_video})
        print(f"[jetson] start ok={start.get('ok')} video={record_video}")
        # Hold the connection open but let the main flow sleep; we just wait.
        await asyncio.sleep(seconds)
        stop = await jetson_cmd(ws, "stop_recording")
        print(f"[jetson] stop ok={stop.get('ok')} "
              f"duration={stop.get('recording', {}).get('duration_sec')}")
        return stop


def jetson_pull(stop_ack: dict, dest: Path) -> list[dict]:
    """Download every file listed in the stop ack via its returned URL."""
    pulled = []
    for team, body in (stop_ack.get("recording", {}).get("teams", {}) or {}).items():
        for f in body.get("files", []):
            url, nbytes = f.get("url", ""), f.get("bytes")
            out = dest / "skeleton" / team / f["kind"]
            out = out.with_suffix(Path(url).suffix)  # skeleton.parquet / video.mp4
            out.parent.mkdir(parents=True, exist_ok=True)
            t0 = time.time()
            with urllib.request.urlopen(url, timeout=120) as r:
                data = r.read()
            out.write_bytes(data)
            dt = time.time() - t0
            ok = (nbytes is None) or (len(data) == nbytes)
            pulled.append({"team": team, "kind": f["kind"], "url": url,
                           "bytes": len(data), "expected": nbytes,
                           "ok": ok, "secs": round(dt, 1), "path": str(out)})
            print(f"[jetson] pull {team}/{f['kind']}: {len(data)} B "
                  f"({'OK' if ok else 'SIZE MISMATCH exp ' + str(nbytes)}) in {dt:.1f}s")
    return pulled


# --------------------------------------------------------------------------
# Pis (OSC control + SFTP pull)
# --------------------------------------------------------------------------
def pi_targets(only_ip: str | None) -> list[tuple[str, int]]:
    ips = [only_ip] if only_ip else PIS
    return [(ip, port) for ip in ips for port in MIC_PORTS]


def pi_start(targets, day, tim, max_minutes, record_audio):
    for ip, port in targets:
        t = parse_target_endpoint(f"{ip}:{port}")
        # Clear any stale open session so a crashed prior run never blocks us.
        pre = send_ctrl(t, "query_state", [], ack_timeout=2.0)
        if pre.ok and "log_open=1" in (pre.message or ""):
            send_ctrl(t, "log_discard_stop", [], ack_timeout=5.0)
        ack = send_ctrl(t, "log_start",
                        [day, tim, str(max_minutes), "1" if record_audio else "0"],
                        ack_timeout=5.0)
        print(f"[pi {ip}:{port}] log_start ok={ack.ok}")


def pi_save_stop(targets):
    """Stop/save every mic CONCURRENTLY. The Pi's save holds its single-threaded
    OSC listener while it writes the (possibly large) FLAC buffer to SD, so a
    sequential loop stacks those delays and later mics keep recording (and the
    earliest time out waiting for their ACK). One thread per mic keeps all stop
    latencies equal, so every session covers the same wall-clock window."""
    results: dict[str, tuple] = {}

    def stop_one(ip, port):
        t = parse_target_endpoint(f"{ip}:{port}")
        t0 = time.time()
        ack = send_ctrl(t, "log_save_stop", [], ack_timeout=30.0)
        results[f"{ip}:{port}"] = (ack.ok, ack.message, round((time.time() - t0) * 1000))

    threads = [threading.Thread(target=stop_one, args=(ip, port)) for ip, port in targets]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    for ep, (ok, msg, ms) in sorted(results.items()):
        print(f"[pi {ep}] log_save_stop ok={ok} msg={msg!r} (ack {ms} ms)")


def pi_pull(targets, day, tim, dest, creds, record_audio):
    pulled = []
    for ip, port in targets:
        mic = f"MIC{1 if port == 9001 else 2}"
        remote_dir = f"{REMOTE_ROOT}/{day}/{tim}/{mic}"
        local_dir = dest / "audio" / ip / mic
        local_dir.mkdir(parents=True, exist_ok=True)
        names = ["opensmile_lld.csv", "vad.csv"]
        if record_audio:
            names += ["audio.flac", "audio.json"]
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(ip, username=creds["username"], password=creds["password"], timeout=10)
            sftp = ssh.open_sftp()
            for name in names:
                out = local_dir / name
                t0 = time.time()
                try:
                    sftp.get(f"{remote_dir}/{name}", str(out))
                    sz = out.stat().st_size
                    pulled.append({"ip": ip, "mic": mic, "file": name, "bytes": sz,
                                   "ok": True, "secs": round(time.time() - t0, 1)})
                    print(f"[pi {ip}:{port}] pull {name}: {sz} B")
                except Exception as exc:  # noqa: BLE001
                    pulled.append({"ip": ip, "mic": mic, "file": name, "bytes": 0,
                                   "ok": False, "err": str(exc)})
                    print(f"[pi {ip}:{port}] pull {name} FAILED: {exc}")
            sftp.close()
        finally:
            ssh.close()
    return pulled


# --------------------------------------------------------------------------
# Validation
# --------------------------------------------------------------------------
def validate_audio(pulled, seconds):
    """Check pulled audio.json recorded_secs is within tolerance of the target."""
    print("\n=== audio duration validation ===")
    for rec in pulled:
        if rec["file"] != "audio.json" or not rec["ok"]:
            continue
        meta = json.loads(Path(rec.get("path", "")).read_text()) if rec.get("path") else None
    # simpler: re-read from disk by walking dest
    for js in DEST_ROOT_G.glob("audio/*/*/audio.json"):
        meta = json.loads(js.read_text())
        got = float(meta.get("recorded_secs", 0))
        ok = abs(got - seconds) <= seconds * 0.05 + 3  # 5% + 3 s slack
        print(f"  {js}: recorded_secs={got:.1f} target~{seconds} {'OK' if ok else 'SHORT'}")


def validate_csvs(seconds):
    print("\n=== prosody/vad row validation ===")
    for csvf in sorted(DEST_ROOT_G.glob("audio/*/*/*.csv")):
        lines = csvf.read_text().strip().splitlines()
        rows = max(0, len(lines) - 1)
        # last time_ms value approximates duration
        try:
            last_ms = float(lines[-1].split(";")[0])
        except Exception:
            last_ms = float("nan")
        print(f"  {csvf.name}: {rows} rows, last time_ms={last_ms:.0f} "
              f"(~{last_ms/1000:.1f}s of {seconds}s)")


def main() -> None:
    global DEST_ROOT_G
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--minutes", type=float, default=2.0, help="Recording length.")
    p.add_argument("--pi", default=None, help="Limit to one Pi IP (default: all six).")
    p.add_argument("--no-jetson", action="store_true", help="Skip the Jetson.")
    p.add_argument("--no-pi", action="store_true", help="Skip the Pis.")
    ns = p.parse_args()

    seconds = ns.minutes * 60.0
    day, tim = session_identity()
    dest = DEST_ROOT / f"{day}_{tim}"
    DEST_ROOT_G = dest
    dest.mkdir(parents=True, exist_ok=True)
    creds = yaml.safe_load(SECRETS.read_text())["pi_ssh"]

    print(f"=== FULL RECORDING PROBE: {seconds:.0f}s ({ns.minutes} min), raw audio+video ON ===")
    print(f"session {day}/{tim}  ->  {dest.resolve()}\n")

    targets = [] if ns.no_pi else pi_targets(ns.pi)
    max_minutes = int(ns.minutes) + 5  # Pi in-RAM cap headroom

    # Start everything.
    jetson_stop: dict = {}
    if not ns.no_jetson:
        # Run Jetson start; it holds for `seconds` internally via the coroutine.
        # We drive Pi start first (fast OSC), then start Jetson and sleep.
        pass
    if targets:
        print(f"Starting {len(targets)} Pi mic processes (record_audio=1) ...")
        pi_start(targets, day, tim, max_minutes, record_audio=True)

    if not ns.no_jetson:
        print("Starting Jetson (record_skeleton=1, record_video=1) ...")
        # Drive the Jetson in the same loop: start, sleep, stop.
        jetson_stop = asyncio.run(jetson_run(day, tim, seconds, record_video=True))
    else:
        print(f"Sleeping {seconds:.0f}s (no Jetson) ...")
        time.sleep(seconds)

    # Stop + save Pis.
    if targets:
        print("\nSaving/stopping Pis ...")
        pi_save_stop(targets)
        # Give the async saves a moment to flush to disk before pulling.
        time.sleep(3)

    # Pull everything.
    print("\n=== PULLING FILES ===")
    jetson_files = jetson_pull(jetson_stop, dest) if jetson_stop else []
    pi_files = pi_pull(targets, day, tim, dest, creds, record_audio=True) if targets else []

    # Validate.
    print("\n=== SUMMARY ===")
    j_ok = sum(1 for f in jetson_files if f["ok"])
    p_ok = sum(1 for f in pi_files if f["ok"])
    print(f"Jetson files: {j_ok}/{len(jetson_files)} ok "
          f"({sum(f['bytes'] for f in jetson_files)/1e6:.1f} MB total)")
    print(f"Pi files:     {p_ok}/{len(pi_files)} ok "
          f"({sum(f['bytes'] for f in pi_files)/1e6:.1f} MB total)")
    validate_csvs(seconds)
    validate_audio(pi_files, seconds)
    print(f"\nPulled files under: {dest.resolve()}")


if __name__ == "__main__":
    main()
