"""External-media coordinator unit/integration tests (no hardware).

Covers the our-side logic that doesn't need the Jetson/Pis:

1. `resolve_external_media_config` parses a profile's `external_recording:`
   block into the device fleet (12 mics, correct ctrl ports, player mapping).
2. The sim device clients run a full start → stop → pull cycle and the
   manifest + `external_media_index.csv` ledger row are written correctly.
3. The folder layout matches EXTERNAL_RECORDING_PLAN.md §3 (skeleton/<team>,
   audio/<player>, CSV→Parquet conversion, raw toggles).

Run with the game env:
    $env:PYTHONPATH = "src"
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m pytest \
        tests/test_external_media_coordinator.py -v
"""

from __future__ import annotations

import csv
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.external_media import (  # noqa: E402
    EXTERNAL_LEDGER_FIELDNAMES,
    GameManifest,
    append_external_ledger_row,
    resolve_external_media_config,
    rollup_status,
)
from core.config import load as load_profile  # noqa: E402
from core.device_connection import load_audio_capture  # noqa: E402
from apps.external_media_coordinator.devices import (  # noqa: E402
    SimJetsonClient,
    SimPiMicClient,
)

TWO_TEAMS = REPO_ROOT / "config" / "profiles" / "two_teams.yaml"


# --------------------------------------------------------------------------
# Config parsing
# --------------------------------------------------------------------------

def test_config_parses_fleet():
    cfg = resolve_external_media_config(load_profile(TWO_TEAMS))
    assert cfg.enabled is True
    assert cfg.jetson_host == "192.168.0.101"
    assert cfg.jetson_ws_port == 9000 and cfg.jetson_http_port == 9100
    # two_teams.yaml is the raw-OFF profile.
    assert cfg.keep_raw_video is False and cfg.keep_raw_audio is False
    assert len(cfg.mics) == 12
    m0 = cfg.mics[0]
    assert (m0.pi_id, m0.ctrl_port, m0.mic_label, m0.player_label) == \
        ("rpi5-11", 9001, "MIC1", "a1")
    # Last mic maps to b6 on port 9002.
    last = cfg.mics[-1]
    assert (last.pi_id, last.ctrl_port, last.mic_label, last.player_label) == \
        ("rpi5-16", 9002, "MIC2", "b6")


def test_fleet_comes_from_device_ports_file():
    """The mic -> player mapping is hardware addressing, sourced from
    config/device_ports_and_addr.yaml (audio_capture block) -- not the profile."""
    ac = load_audio_capture()
    assert len(ac.mics) == 12
    # Spot-check the mapping the user confirmed (rpi5-14/.14 MIC1 -> b1).
    by_key = {(m.pi_id, m.mic): m for m in ac.mics}
    assert by_key[("rpi5-11", 1)].player_label == "a1"
    assert by_key[("rpi5-14", 1)].player_label == "b1"
    assert by_key[("rpi5-16", 2)].player_label == "b6"
    # The coordinator config exposes exactly this fleet.
    cfg = resolve_external_media_config(load_profile(TWO_TEAMS))
    assert [m.device_key for m in cfg.mics] == [m.device_key for m in ac.mics]


def test_config_disabled_when_absent():
    class _P:
        raw = {}
    cfg = resolve_external_media_config(_P())
    assert cfg.enabled is False
    assert cfg.mics == ()


# --------------------------------------------------------------------------
# Sim end-to-end lifecycle -> manifest + ledger + folder layout
# --------------------------------------------------------------------------

def _sim_config(raw_video: bool, raw_audio: bool):
    cfg = resolve_external_media_config(load_profile(TWO_TEAMS))
    # Flip toggles for the sim run (object is frozen; rebuild via __dict__).
    object.__setattr__(cfg, "keep_raw_video", raw_video)
    object.__setattr__(cfg, "keep_raw_audio", raw_audio)
    object.__setattr__(cfg, "use_sim", True)
    return cfg


def _run_sim_game(tmp_path: Path, raw_video: bool, raw_audio: bool):
    cfg = _sim_config(raw_video, raw_audio)
    folder = tmp_path / "games" / "2026-08-17" / "16-00-00"
    manifest = GameManifest.new(folder, cfg)

    clients = [SimJetsonClient(cfg, tmp_path / "_sim")]
    clients += [SimPiMicClient(mic, cfg) for mic in cfg.mics]

    # start all (sim always ok)
    for c in clients:
        assert c.start("2026-08-17", "16-00-00") is True
    # stop all
    for c in clients:
        assert c.stop() is True
    # pull all
    results = {}
    for c in clients:
        if isinstance(c, SimPiMicClient):
            res = c.pull(folder, "2026-08-17", "16-00-00")
        else:
            res = c.pull(folder)
        results[c.device_key] = res
        manifest.set_status(c.device_key, res.status,
                            files=res.ok_count, bytes_=res.total_bytes)
    manifest.write()
    return cfg, folder, manifest, results


def test_sim_full_cycle_raw_on(tmp_path):
    cfg, folder, manifest, results = _run_sim_game(tmp_path, True, True)

    # Every device ok.
    assert manifest.all_terminal()
    assert rollup_status(manifest, "jetson") == "ok"
    assert rollup_status(manifest, "pi:") == "ok"

    # Jetson: skeleton + frames + video per team (raw on).
    assert (folder / "skeleton" / "a" / "skeleton.parquet").exists()
    assert (folder / "skeleton" / "b" / "video.mp4").exists()

    # Audio: per-player folders, vad.csv -> vad.parquet, flac present (raw on).
    a1 = folder / "audio" / "a1"
    assert (a1 / "vad.parquet").exists()
    assert not (a1 / "vad.csv").exists()  # CSV converted + removed
    assert (a1 / "audio.flac").exists()
    assert (folder / "audio" / "b6").is_dir()

    # Manifest written with the mic->player map for this game.
    assert (folder / "external_media_manifest.json").exists()
    assert manifest.mic_player_map["rpi5-11"] == {"MIC1": "a1", "MIC2": "a2"}

    # Ledger row appended, joinable on date+time, correct rollups + toggles.
    ledger = append_external_ledger_row(tmp_path, manifest, "2026-08-17", "16-00-00")
    rows = list(csv.DictReader(ledger.open()))
    assert len(rows) == 1
    row = rows[0]
    assert (row["date"], row["time"]) == ("2026-08-17", "16-00-00")
    assert row["skeleton_status"] == "ok" and row["audio_status"] == "ok"
    assert row["raw_video"] == "1" and row["raw_audio"] == "1"
    assert row["devices_missing"] == ""
    # 6 jetson files (2 teams x skeleton/frames/video), 24 audio (12 mics x 2).
    assert int(row["skeleton_files"]) == 6
    assert int(row["audio_files"]) == 24
    assert tuple(row.keys()) == EXTERNAL_LEDGER_FIELDNAMES


def test_sim_full_cycle_raw_off(tmp_path):
    cfg, folder, manifest, results = _run_sim_game(tmp_path, False, False)
    # Raw off: no video.mp4, no audio.flac.
    assert not (folder / "skeleton" / "a" / "video.mp4").exists()
    assert not (folder / "audio" / "a1" / "audio.flac").exists()
    assert (folder / "audio" / "a1" / "vad.parquet").exists()


def test_unreachable_device_marks_failed(tmp_path):
    cfg = _sim_config(False, False)
    folder = tmp_path / "games" / "2026-08-17" / "16-05-00"
    manifest = GameManifest.new(folder, cfg)
    # Simulate one Pi mic + the jetson unreachable at start.
    manifest.set_status("jetson", "unreachable_at_start", files=0, bytes_=0)
    manifest.set_status("pi:rpi5-11:MIC1", "ok", files=2, bytes_=100)
    for mic in cfg.mics[1:]:
        manifest.set_status(mic.device_key, "ok", files=2, bytes_=100)
    assert manifest.all_terminal()
    assert rollup_status(manifest, "jetson") == "failed"
    assert rollup_status(manifest, "pi:") == "ok"
    ledger = append_external_ledger_row(tmp_path, manifest, "2026-08-17", "16-05-00")
    row = list(csv.DictReader(ledger.open()))[0]
    assert row["skeleton_status"] == "failed"
    assert "jetson" in row["devices_missing"]


if __name__ == "__main__":
    # Standalone runner (pytest not required): each test gets a fresh tmp dir.
    tests = [
        test_config_parses_fleet,
        test_fleet_comes_from_device_ports_file,
        test_config_disabled_when_absent,
        test_sim_full_cycle_raw_on,
        test_sim_full_cycle_raw_off,
        test_unreachable_device_marks_failed,
    ]
    failed = 0
    for fn in tests:
        try:
            if fn.__code__.co_argcount:  # takes tmp_path
                with tempfile.TemporaryDirectory() as d:
                    fn(Path(d))
            else:
                fn()
            print(f"PASS {fn.__name__}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            import traceback
            print(f"FAIL {fn.__name__}: {exc}")
            traceback.print_exc()
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    raise SystemExit(1 if failed else 0)
