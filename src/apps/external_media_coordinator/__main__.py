"""external_media_coordinator - drive per-game recording on the external
skeleton (Jetson) and audio (Raspberry Pi) devices, then pull files back.

Role in the system
------------------
A standalone launcher-spawned process (spawned alongside `gameplay_recorder`)
that watches the `recorder.game` bus topic for game start/end and, for each
game:

1. On `game_started` — fan out a recording **start** to the Jetson and every
   configured Pi mic, **concurrently** (one worker thread per device; see the
   "Concurrent start/stop" rule in EXTERNAL_RECORDING_PLAN.md §1). Devices
   that don't ack are marked `unreachable_at_start` and the game proceeds
   (locked decision: warn-and-play).
2. On `game_ended` — fan out **stop** concurrently, then **pull** each
   device's files back into the game folder asynchronously (a large video
   pull must never delay the next game). CSVs are converted to Parquet on the
   way in. Per-device status lands in `external_media_manifest.json`, and a
   roll-up row is appended to `external_media_index.csv` once all devices are
   terminal.

Configuration: the profile's `external_recording:` block (parsed by
``core.external_media.resolve_external_media_config``). Set
``external_recording: {sim: true}`` to run against no-hardware sim device
clients for offline testing.

Run standalone:

    $env:PYTHONPATH = "src"
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.external_media_coordinator \
        --profile config/profiles/two_teams.yaml --proc external_media_coordinator
"""

from __future__ import annotations

import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402
import yaml  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.external_media import (  # noqa: E402
    GameManifest,
    append_external_ledger_row,
    resolve_external_media_config,
)
from core.gameplay_recording import resolve_gameplay_recording_config  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402

from apps.external_media_coordinator.devices import (  # noqa: E402
    JetsonClient,
    PiMicClient,
    SimJetsonClient,
    SimPiMicClient,
)

DEFAULT_TARGET_HZ = 20.0  # event-driven; this only bounds pull-state polling
SECRETS_PATH = Path("config/secrets.yaml")
# Brief settle delay after the Pi's "saving" ack before pulling, so the async
# on-device write has time to flush to disk (see EXTERNAL_RECORDING_PLAN.md).
_SAVE_SETTLE_S = 3.0


def _load_ssh_creds() -> dict:
    """Read the Pi SSH username/password from config/secrets.yaml."""
    if not SECRETS_PATH.exists():
        return {"username": "pi", "password": ""}
    data = yaml.safe_load(SECRETS_PATH.read_text(encoding="utf-8")) or {}
    return data.get("pi_ssh", {"username": "pi", "password": ""})


def _build_clients(cfg, sim_dir: Path) -> list:
    """Construct one device client per capture device (Jetson + each Pi mic)."""
    if cfg.use_sim:
        clients = [SimJetsonClient(cfg, sim_dir)]
        clients += [SimPiMicClient(mic, cfg) for mic in cfg.mics]
        return clients
    creds = _load_ssh_creds()
    clients: list = [JetsonClient(cfg.jetson_host, cfg.jetson_ws_port,
                                  cfg.jetson_http_port, cfg.keep_raw_video)]
    clients += [PiMicClient(mic, cfg, creds) for mic in cfg.mics]
    return clients


def main(argv: list[str] | None = None) -> int:
    """Run the coordinator: subscribe to recorder.game and orchestrate pulls."""

    args, _ = parse_proc_args(argv, default_proc="external_media_coordinator")
    profile = load_profile(args.profile_path)
    cfg = resolve_external_media_config(profile)
    _, recordings_root = resolve_gameplay_recording_config(profile)
    root_dir = Path(recordings_root)

    target_hz = default_runtime_setting(
        "external_media_coordinator", "fps_target", DEFAULT_TARGET_HZ)
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_TARGET_HZ)

    sub = bus.make_sub(proc.ctx, topics=["recorder.game"])
    pub = bus.make_pub(proc.ctx)
    proc.use_heartbeat_pub(pub)

    clients = _build_clients(cfg, sim_dir=root_dir / "_sim_devices")

    banner(proc.proc,
           f"enabled={cfg.enabled} sim={cfg.use_sim} devices={len(clients)} "
           f"raw_video={cfg.keep_raw_video} raw_audio={cfg.keep_raw_audio}")

    # Worker pool sized to the fleet so start/stop fan out truly concurrently.
    pool = ThreadPoolExecutor(max_workers=max(4, len(clients)))

    # Per-process state. `current` is the in-progress game (manifest + folder);
    # `pull_futures` tracks background pull tasks so the tick can collect them.
    state: dict[str, Any] = {
        "current": None,        # dict with manifest/date/time when a game is active
        "pull_futures": [],     # list of (client, Future, manifest, date, time)
        "games_done": 0,
    }

    def _fan_out(method: str, date_str: str, time_str: str) -> dict[str, bool]:
        """Call `method` on every client concurrently; return device_key->ok."""
        if method == "start":
            futs = {c.device_key: pool.submit(c.start, date_str, time_str)
                    for c in clients}
        else:  # stop takes no args
            futs = {c.device_key: pool.submit(c.stop) for c in clients}
        return {k: bool(f.result()) for k, f in futs.items()}

    def _on_game_started(folder: str, date_str: str, time_str: str) -> None:
        if not cfg.enabled:
            return
        manifest = GameManifest.new(Path(folder), cfg)
        results = _fan_out("start", date_str, time_str)
        for key, ok in results.items():
            if not ok:
                manifest.set_status(key, "unreachable_at_start", files=0, bytes_=0)
        manifest.write()
        state["current"] = {"manifest": manifest, "date": date_str, "time": time_str}
        print(f"[external_media] game started {date_str}/{time_str}: "
              f"{sum(results.values())}/{len(results)} devices recording",
              flush=True)

    def _on_game_ended(folder: str, date_str: str, time_str: str) -> None:
        cur = state["current"]
        if cur is None:
            return
        state["current"] = None
        manifest: GameManifest = cur["manifest"]
        if not cfg.enabled:
            return
        stop_results = _fan_out("stop", date_str, time_str)
        time.sleep(_SAVE_SETTLE_S)  # let async on-device saves flush
        # Kick off a background pull per device that is still reachable.
        for client in clients:
            if manifest.devices.get(client.device_key, {}).get("status") == \
                    "unreachable_at_start":
                continue
            if not stop_results.get(client.device_key, False):
                manifest.set_status(client.device_key, "permanently_failed",
                                    files=0, bytes_=0)
                manifest.write()
                continue
            manifest.set_status(client.device_key, "pulling")
            manifest.write()
            fut = pool.submit(_pull_one, client, folder, date_str, time_str)
            state["pull_futures"].append((client, fut, manifest, date_str, time_str))
        print(f"[external_media] game ended {date_str}/{time_str}: "
              f"pulling from {len(state['pull_futures'])} devices", flush=True)

    def _collect_pulls() -> None:
        """Harvest finished pull futures, update manifests, finalize ledgers."""
        still_running = []
        # Group futures by game so each game's ledger row is written once.
        for client, fut, manifest, date_str, time_str in state["pull_futures"]:
            if not fut.done():
                still_running.append((client, fut, manifest, date_str, time_str))
                continue
            result = fut.result()
            manifest.set_status(client.device_key, result.status,
                                files=result.ok_count, bytes_=result.total_bytes)
            manifest.write()
            if manifest.all_terminal():
                append_external_ledger_row(root_dir, manifest, date_str, time_str)
                state["games_done"] += 1
                print(f"[external_media] {date_str}/{time_str} complete: "
                      f"ledger row written", flush=True)
        state["pull_futures"] = still_running

    def tick(_p: Proc) -> None:
        # Drain recorder.game events (latest-wins is fine; events are rare).
        while True:
            try:
                topic, body = bus.recv(sub, flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            if topic != "recorder.game" or not isinstance(body, dict):
                continue
            event = body.get("event")
            folder = str(body.get("folder") or "")
            date_str = str(body.get("date") or "")
            time_str = str(body.get("time") or "")
            if event == "game_started":
                _on_game_started(folder, date_str, time_str)
            elif event == "game_ended":
                _on_game_ended(folder, date_str, time_str)
        _collect_pulls()

    def teardown(_p: Proc) -> None:
        sub.close(0)
        pool.shutdown(wait=False, cancel_futures=True)

    return proc.run(tick, teardown=teardown)


def _pull_one(client, folder: str, date_str: str, time_str: str):
    """Pull one device's files into the game folder (runs on a worker thread)."""
    if isinstance(client, (SimPiMicClient, PiMicClient)):
        return client.pull(Path(folder), date_str, time_str)
    return client.pull(Path(folder))


if __name__ == "__main__":
    raise SystemExit(main())
