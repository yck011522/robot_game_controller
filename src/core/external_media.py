"""External media recording: config model + per-game manifest/ledger helpers.

This module is the our-side half of the external-media feature documented in
``EXTERNAL_RECORDING_PLAN.md`` at the repo root. It knows *what* devices exist
(the Jetson skeleton tracker + the fleet of audio Raspberry Pis), *how* a
profile configures them (the ``external_recording:`` block), and *where* the
pulled artifacts land in the per-game folder. It deliberately owns no network
code — the actual device protocols (WebSocket/HTTP for the Jetson, OSC/SFTP
for the Pis) live in the device-client classes under
``apps/external_media_coordinator/``.

On-disk destination (see EXTERNAL_RECORDING_PLAN.md §3)::

    <recordings_root>/games/<date>/<time>/     # shared with gameplay_recorder
      skeleton/<team>/{skeleton,frames}.parquet[, video.mp4]
      audio/<player>/{opensmile_lld,vad}.parquet[, audio.flac, audio.json]
      external_media_manifest.json

Design notes
------------
* The game folder identity (``<date>/<time>``) is never computed here — it
  arrives on the ``recorder.game`` bus topic published by
  ``apps.gameplay_recorder`` (Stage 1), so both processes always agree.
* The mic→player mapping is applied at pull time to produce per-player
  ``audio/<player>/`` folders, and is recorded in each game's manifest so old
  folders stay interpretable if the mapping ever changes.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from core.device_connection import AudioMicAssignment, load_audio_capture

# Manifest filename written into each game folder (see §4.4 of the plan).
MANIFEST_FILENAME = "external_media_manifest.json"

# Separate per-game ledger for external-media pull status. Deliberately NOT
# `games_index.csv` (that ledger's schema is frozen for backwards
# compatibility); this one joins to it on the shared `date` + `time` columns.
EXTERNAL_LEDGER_FILENAME = "external_media_index.csv"

# Column order for one external_media_index.csv row.
EXTERNAL_LEDGER_FIELDNAMES = (
    "date",
    "time",
    "skeleton_status",
    "audio_status",
    "skeleton_files",
    "skeleton_bytes",
    "audio_files",
    "audio_bytes",
    "raw_video",
    "raw_audio",
    "devices_missing",
)

# Status values a single device/mic can be in (§4.4 of the plan).
DEVICE_STATUSES = (
    "pending",               # game started, start sent, not yet pulled
    "pulling",               # transfer in progress
    "ok",                    # all expected files pulled + converted
    "partial",               # some files pulled, some missing/failed
    "unreachable_at_start",  # did not ack the start command
    "permanently_failed",    # retries exhausted within the retry window
)


# --------------------------------------------------------------------------
# Config model (parsed from the profile's `external_recording:` block)
# --------------------------------------------------------------------------

# The mic fleet (identity, host, ctrl port, mic->player mapping) is hardware
# addressing, so it lives in config/device_ports_and_addr.yaml and is loaded
# via core.device_connection.load_audio_capture -- NOT duplicated per profile.
# `MicTarget` is kept as a short alias so existing call sites read naturally.
MicTarget = AudioMicAssignment


@dataclass(frozen=True)
class ExternalMediaConfig:
    """Resolved `external_recording:` block for one profile.

    The Jetson address and the raw-media toggles come from the profile (they
    are per-run behavior choices). The audio fleet's mic->player mapping, OSC
    timing, and processing toggles come from ``device_ports_and_addr.yaml``
    (hardware addressing), via ``load_audio_capture``.
    """
    enabled: bool
    # Jetson skeleton/video (from the profile)
    jetson_host: str
    jetson_ws_port: int
    jetson_http_port: int
    keep_raw_video: bool
    # Audio raw-media toggle (from the profile; fleet mapping from device file)
    keep_raw_audio: bool
    # Audio fleet (from device_ports_and_addr.yaml audio_capture block)
    max_minutes: int          # Pi in-RAM cap arg to log_start (> game duration)
    ack_timeout_s: float      # per-command OSC ack timeout
    emotion: bool             # enable emotion.csv alongside prosody/VAD
    mics: tuple[MicTarget, ...]
    # Pull behavior
    pull_timeout_s: float     # per-file transfer timeout
    retry_interval_s: float   # offline-device retry cadence
    retry_window_s: float     # give up after this and mark permanently_failed
    use_sim: bool             # True -> sim device clients (no hardware)


def resolve_external_media_config(profile: Any) -> ExternalMediaConfig:
    """Parse + validate the profile's ``external_recording:`` block.

    The block holds the Jetson address, the raw-media toggles, and pull/retry
    tuning. The audio mic fleet is read from ``device_ports_and_addr.yaml``
    (``audio_capture`` block) so the mic->player mapping is defined once.

    Args:
        profile: A loaded ``core.config.Profile`` (reads ``profile.raw``).

    Returns:
        An ``ExternalMediaConfig``. When the block is absent or
        ``enabled: false``, ``enabled`` is False and the device fields carry
        harmless defaults (the coordinator idles). When disabled, the audio
        fleet is left empty so a missing ``audio_capture`` block never breaks
        a profile that doesn't use external recording.
    """
    node = profile.raw.get("external_recording")
    node = node if isinstance(node, dict) else {}
    enabled = bool(node.get("enabled", False))

    skel = node.get("skeleton") if isinstance(node.get("skeleton"), dict) else {}
    aud = node.get("audio") if isinstance(node.get("audio"), dict) else {}
    pull = node.get("pull") if isinstance(node.get("pull"), dict) else {}

    # Audio fleet: shared hardware mapping from device_ports_and_addr.yaml.
    # Only loaded when actually enabled, so non-recording profiles don't need
    # the audio_capture block to exist.
    if enabled:
        ac = load_audio_capture()
        mics: tuple[MicTarget, ...] = ac.mics
        max_minutes = ac.max_minutes
        ack_timeout_s = ac.ack_timeout_s
        emotion = ac.emotion
    else:
        mics = ()
        max_minutes = 60
        ack_timeout_s = 5.0
        emotion = False

    return ExternalMediaConfig(
        enabled=enabled,
        jetson_host=str(skel.get("host") or "192.168.0.101"),
        jetson_ws_port=int(skel.get("ws_port", 9000)),
        jetson_http_port=int(skel.get("http_port", 9100)),
        keep_raw_video=bool(skel.get("keep_raw_video", False)),
        keep_raw_audio=bool(aud.get("keep_raw_audio", False)),
        max_minutes=max_minutes,
        ack_timeout_s=ack_timeout_s,
        emotion=emotion,
        mics=mics,
        pull_timeout_s=float(pull.get("timeout_s", 60.0)),
        retry_interval_s=float(pull.get("retry_interval_s", 60.0)),
        retry_window_s=float(pull.get("retry_window_s", 3600.0)),
        use_sim=bool(node.get("sim", False)),
    )


# --------------------------------------------------------------------------
# Per-game manifest (§4.4) — tracks each device's pull status for one game
# --------------------------------------------------------------------------

@dataclass
class GameManifest:
    """In-memory + on-disk record of one game's external-media pull status."""
    folder: Path
    raw_video: bool
    raw_audio: bool
    mic_player_map: dict[str, dict[str, str]]   # pi_id -> {MIC1: "a1", ...}
    devices: dict[str, dict[str, Any]] = field(default_factory=dict)

    @classmethod
    def new(cls, folder: Path, cfg: ExternalMediaConfig) -> "GameManifest":
        """Create a fresh manifest for a game, with every device `pending`."""
        mic_map: dict[str, dict[str, str]] = {}
        devices: dict[str, dict[str, Any]] = {}
        devices["jetson"] = {"status": "pending", "files": None, "bytes": None}
        for mic in cfg.mics:
            mic_map.setdefault(mic.pi_id, {})[mic.mic_label] = mic.player_label
            devices[mic.device_key] = {
                "status": "pending", "files": None, "bytes": None,
                "player": mic.player_label,
            }
        return cls(folder=Path(folder), raw_video=cfg.keep_raw_video,
                   raw_audio=cfg.keep_raw_audio, mic_player_map=mic_map,
                   devices=devices)

    def set_status(self, device_key: str, status: str,
                   files: int | None = None, bytes_: int | None = None) -> None:
        """Update one device's status (and optionally its file/byte counts)."""
        if device_key not in self.devices:
            self.devices[device_key] = {}
        entry = self.devices[device_key]
        entry["status"] = status
        if files is not None:
            entry["files"] = files
        if bytes_ is not None:
            entry["bytes"] = bytes_

    def all_terminal(self) -> bool:
        """True once every device is in a terminal state (ok/partial/failed)."""
        terminal = {"ok", "partial", "unreachable_at_start", "permanently_failed"}
        return all(d.get("status") in terminal for d in self.devices.values())

    def write(self) -> Path:
        """Persist the manifest into the game folder. Returns the path."""
        self.folder.mkdir(parents=True, exist_ok=True)
        path = self.folder / MANIFEST_FILENAME
        payload = {
            "folder": str(self.folder),
            "raw_video": self.raw_video,
            "raw_audio": self.raw_audio,
            "mic_player_map": self.mic_player_map,
            "devices": self.devices,
        }
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return path


def rollup_status(manifest: GameManifest, prefix: str) -> str:
    """Roll per-device statuses up to one `ok|partial|failed|disabled` value.

    ``prefix`` selects the device group: ``"jetson"`` for skeleton, ``"pi:"``
    for the audio fleet.
    """
    entries = {k: v for k, v in manifest.devices.items() if k.startswith(prefix)}
    if not entries:
        return "disabled"
    statuses = {v.get("status") for v in entries.values()}
    if statuses == {"ok"}:
        return "ok"
    if statuses <= {"unreachable_at_start", "permanently_failed"}:
        return "failed"
    if "ok" in statuses or "partial" in statuses:
        return "partial"
    return "partial"


def append_external_ledger_row(root_dir: Path, manifest: GameManifest,
                               date_str: str, time_str: str) -> Path:
    """Append one row to ``<root_dir>/external_media_index.csv``.

    Called once per game, when every device has reached a terminal state.
    Joins to ``games_index.csv`` on the shared ``date`` + ``time`` columns.
    Returns the ledger path.
    """
    import csv

    def _sums(prefix: str) -> tuple[int, int]:
        files = bytes_ = 0
        for k, v in manifest.devices.items():
            if k.startswith(prefix):
                files += int(v.get("files") or 0)
                bytes_ += int(v.get("bytes") or 0)
        return files, bytes_

    skel_files, skel_bytes = _sums("jetson")
    aud_files, aud_bytes = _sums("pi:")
    missing = ";".join(
        k for k, v in manifest.devices.items()
        if v.get("status") in ("unreachable_at_start", "permanently_failed")
    )
    row = {
        "date": date_str,
        "time": time_str,
        "skeleton_status": rollup_status(manifest, "jetson"),
        "audio_status": rollup_status(manifest, "pi:"),
        "skeleton_files": skel_files,
        "skeleton_bytes": skel_bytes,
        "audio_files": aud_files,
        "audio_bytes": aud_bytes,
        "raw_video": 1 if manifest.raw_video else 0,
        "raw_audio": 1 if manifest.raw_audio else 0,
        "devices_missing": missing,
    }
    root_dir = Path(root_dir)
    root_dir.mkdir(parents=True, exist_ok=True)
    path = root_dir / EXTERNAL_LEDGER_FILENAME
    write_header = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=EXTERNAL_LEDGER_FIELDNAMES)
        if write_header:
            writer.writeheader()
        writer.writerow(row)
    return path
