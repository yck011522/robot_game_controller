"""Device clients for the external-media coordinator.

One client object per capture device (the Jetson, and one per Pi microphone).
All clients share a small interface so the coordinator can treat the fleet
uniformly:

    start(date_str, time_str)  -> begin recording (blocking; returns ack ok)
    stop()                     -> stop + save  (blocking; fast "saving" ack)
    pull(dest_dir)             -> download files into dest_dir -> PullResult
    ping()                     -> liveness / reachability check

The methods perform blocking network I/O. The coordinator calls them from
worker threads (never from the bus tick), and fans start/stop out
*concurrently* across the fleet — a save holds a Pi's single-threaded OSC
listener while it flushes FLAC to SD, so a sequential loop would stack delays
(see EXTERNAL_RECORDING_PLAN.md §1 "Concurrent start/stop" and §10).

Two implementations:
* ``JetsonClient``  — real, WebSocket command channel + HTTP file pull.
* ``PiMicClient``   — real, OSC control + SFTP pull.
* ``SimJetsonClient`` / ``SimPiMicClient`` — no hardware; write small dummy
  files into a local temp dir so the whole lifecycle is testable offline.
"""

from __future__ import annotations

import asyncio
import json
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

from core.external_media import ExternalMediaConfig, MicTarget


@dataclass
class PulledFile:
    """One file transferred from a device to the game folder."""
    rel_path: str       # path relative to the game folder, e.g. "skeleton/a/video.mp4"
    nbytes: int
    ok: bool
    error: str = ""


@dataclass
class PullResult:
    """Aggregate result of pulling one device's files for a game."""
    device_key: str
    files: list[PulledFile] = field(default_factory=list)

    @property
    def ok_count(self) -> int:
        return sum(1 for f in self.files if f.ok)

    @property
    def total_bytes(self) -> int:
        return sum(f.nbytes for f in self.files if f.ok)

    @property
    def status(self) -> str:
        if not self.files:
            return "permanently_failed"
        if all(f.ok for f in self.files):
            return "ok"
        return "partial" if any(f.ok for f in self.files) else "permanently_failed"


# --------------------------------------------------------------------------
# Jetson skeleton tracker (WebSocket commands + HTTP pull)
# --------------------------------------------------------------------------

class JetsonClient:
    """Real client for the Jetson recorder_node (tracker_api.md protocol)."""

    def __init__(self, host: str, ws_port: int, http_port: int,
                 keep_raw_video: bool):
        self.host = host
        self.ws_url = f"ws://{host}:{ws_port}"
        self.http_port = http_port
        self.keep_raw_video = keep_raw_video
        self.device_key = "jetson"
        # Populated by stop(): the per-team file URLs the node returns.
        self._last_stop_ack: dict = {}

    async def _cmd(self, command: str, args: dict | None = None) -> dict:
        import websockets  # local import so sim path needs no dependency
        req_id = f"emc-{command}-{time.time_ns()}"
        msg = {"type": "command", "command": command, "request_id": req_id}
        if args:
            msg["args"] = args
        async with websockets.connect(self.ws_url) as ws:
            await ws.send(json.dumps(msg))
            while True:
                reply = json.loads(await asyncio.wait_for(ws.recv(), timeout=20))
                if reply.get("request_id") == req_id:
                    return reply

    def ping(self) -> bool:
        try:
            ack = asyncio.run(self._cmd("ping"))
            return bool(ack.get("ok"))
        except Exception:
            return False

    def start(self, date_str: str, time_str: str) -> bool:
        try:
            ack = asyncio.run(self._cmd("start_recording", {
                "date": date_str, "time": time_str,
                "record_skeleton": True, "record_video": self.keep_raw_video,
            }))
            return bool(ack.get("ok"))
        except Exception:
            return False

    def stop(self) -> bool:
        try:
            ack = asyncio.run(self._cmd("stop_recording"))
            self._last_stop_ack = ack
            return bool(ack.get("ok"))
        except Exception:
            return False

    def pull(self, dest_dir: Path) -> PullResult:
        """Download every file listed in the stop ack via its returned URL.

        The node returns LAN-reachable URLs (advertise_host), so we GET them
        verbatim. Files land under ``<dest_dir>/skeleton/<team>/<file>``.
        """
        result = PullResult(device_key=self.device_key)
        teams = (self._last_stop_ack.get("recording", {}) or {}).get("teams", {}) or {}
        for team, body in teams.items():
            for f in body.get("files", []):
                url = f.get("url", "")
                name = Path(url).name
                rel = f"skeleton/{team}/{name}"
                out = Path(dest_dir) / rel
                out.parent.mkdir(parents=True, exist_ok=True)
                try:
                    with urllib.request.urlopen(url, timeout=120) as r:
                        data = r.read()
                    out.write_bytes(data)
                    result.files.append(PulledFile(rel, len(data), True))
                except Exception as exc:  # noqa: BLE001 - report per-file
                    result.files.append(PulledFile(rel, 0, False, str(exc)))
        return result


# --------------------------------------------------------------------------
# Raspberry Pi microphone (OSC control + SFTP pull)
# --------------------------------------------------------------------------

# Pi-side root under which every session is stored (see the Pi docs).
_PI_SESSION_ROOT = "/home/pi/SPEECH_RECORD_ANALYSIS/SESSION_LOGS"
# CSVs converted to Parquet on pull; audio.json/audio.flac travel as-is.
_CSV_KINDS = ("opensmile_lld", "vad")


class PiMicClient:
    """Real client for one Pi microphone process (voiceAnonymizer_PI protocol)."""

    def __init__(self, target: MicTarget, cfg: ExternalMediaConfig,
                 ssh_creds: dict):
        self.t = target
        self.cfg = cfg
        self.ssh_creds = ssh_creds
        self.device_key = target.device_key
        # voiceAnonymizer_PI's speech_control provides the OSC helpers.
        import sys
        voice_repo = str(_voice_repo_path())
        if voice_repo not in sys.path:
            sys.path.insert(0, voice_repo)

    def _endpoint(self):
        from speech_control import parse_target_endpoint
        return parse_target_endpoint(f"{self.t.host}:{self.t.ctrl_port}")

    def ping(self) -> bool:
        try:
            from speech_control import send_ctrl
            ack = send_ctrl(self._endpoint(), "query_state", [],
                            ack_timeout=self.cfg.ack_timeout_s)
            return bool(ack.ok)
        except Exception:
            return False

    def start(self, date_str: str, time_str: str) -> bool:
        from speech_control import send_ctrl
        ep = self._endpoint()
        try:
            # Clear any stale open session so a crashed prior run never blocks us.
            pre = send_ctrl(ep, "query_state", [], ack_timeout=2.0)
            if pre.ok and "log_open=1" in (pre.message or ""):
                send_ctrl(ep, "log_discard_stop", [], ack_timeout=5.0)
            ack = send_ctrl(ep, "log_start",
                            [date_str, time_str, str(self.cfg.max_minutes),
                             "1" if self.cfg.keep_raw_audio else "0"],
                            ack_timeout=self.cfg.ack_timeout_s)
            return bool(ack.ok)
        except Exception:
            return False

    def stop(self) -> bool:
        # Async on the Pi: returns an immediate "saving" ack; the actual file
        # write finishes shortly after. We pull after a brief settle delay.
        try:
            from speech_control import send_ctrl
            ack = send_ctrl(self._endpoint(), "log_save_stop", [],
                            ack_timeout=30.0)
            return bool(ack.ok)
        except Exception:
            return False

    def pull(self, dest_dir: Path, date_str: str, time_str: str) -> PullResult:
        """SFTP this mic's files into ``<dest_dir>/audio/<player>/``.

        CSVs are converted to Parquet on the way in (and the local CSV
        discarded); audio.json/audio.flac are copied verbatim.
        """
        import paramiko

        result = PullResult(device_key=self.device_key)
        mic_dir = f"{_PI_SESSION_ROOT}/{date_str}/{time_str}/{self.t.mic_label}"
        out_dir = Path(dest_dir) / "audio" / self.t.player_label
        out_dir.mkdir(parents=True, exist_ok=True)

        names = [f"{k}.csv" for k in _CSV_KINDS]
        if self.cfg.emotion:
            names.append("emotion.csv")
        if self.cfg.keep_raw_audio:
            names += ["audio.flac", "audio.json"]

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(self.t.host, username=self.ssh_creds["username"],
                        password=self.ssh_creds["password"], timeout=10)
            sftp = ssh.open_sftp()
            for name in names:
                remote = f"{mic_dir}/{name}"
                try:
                    if name.endswith(".csv"):
                        rel = f"audio/{self.t.player_label}/{name[:-4]}.parquet"
                        out = out_dir / (name[:-4] + ".parquet")
                        data = sftp.open(remote).read()
                        n = _csv_bytes_to_parquet(data, out)
                    else:
                        rel = f"audio/{self.t.player_label}/{name}"
                        out = out_dir / name
                        sftp.get(remote, str(out))
                        n = out.stat().st_size
                    result.files.append(PulledFile(rel, n, True))
                except Exception as exc:  # noqa: BLE001
                    rel = f"audio/{self.t.player_label}/{name}"
                    result.files.append(PulledFile(rel, 0, False, str(exc)))
            sftp.close()
        except Exception as exc:  # noqa: BLE001
            result.files.append(PulledFile(
                f"audio/{self.t.player_label}/", 0, False, f"ssh: {exc}"))
        finally:
            ssh.close()
        return result


def _csv_bytes_to_parquet(data: bytes, out: Path) -> int:
    """Convert one semicolon-delimited CSV (Pi prosody/VAD) to Parquet (zstd).

    Returns the written Parquet size in bytes. The source CSV is not kept.
    """
    import io
    import pyarrow.csv as pcsv
    import pyarrow.parquet as pq

    table = pcsv.read_csv(io.BytesIO(data),
                          parse_options=pcsv.ParseOptions(delimiter=";"))
    pq.write_table(table, out, compression="zstd")
    return out.stat().st_size


def _voice_repo_path() -> Path:
    """Absolute path to the cloned voiceAnonymizer_PI repo (OSC helpers)."""
    return Path("C:/Users/yck01/GitHub/voiceAnonymizer_PI")


# --------------------------------------------------------------------------
# Sim clients (no hardware) — write small dummy files locally
# --------------------------------------------------------------------------

class SimJetsonClient:
    """No-hardware Jetson stand-in. Writes tiny dummy files on pull."""

    def __init__(self, cfg: ExternalMediaConfig, sim_dir: Path):
        self.device_key = "jetson"
        self.cfg = cfg
        self.sim_dir = Path(sim_dir)
        self._recording = False
        self._date = ""
        self._time = ""

    def ping(self) -> bool:
        return True

    def start(self, date_str: str, time_str: str) -> bool:
        self._recording, self._date, self._time = True, date_str, time_str
        return True

    def stop(self) -> bool:
        self._recording = False
        return True

    def pull(self, dest_dir: Path) -> PullResult:
        result = PullResult(device_key=self.device_key)
        for team in ("a", "b"):
            for kind in ("skeleton", "frames"):
                rel = f"skeleton/{team}/{kind}.parquet"
                out = Path(dest_dir) / rel
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_bytes(b"PAR1-sim")  # placeholder bytes
                result.files.append(PulledFile(rel, out.stat().st_size, True))
            if self.cfg.keep_raw_video:
                rel = f"skeleton/{team}/video.mp4"
                out = Path(dest_dir) / rel
                out.write_bytes(b"\x00\x00\x00\x18ftypmp42-sim")
                result.files.append(PulledFile(rel, out.stat().st_size, True))
        return result


class SimPiMicClient:
    """No-hardware Pi mic stand-in. Writes a tiny CSV converted to Parquet."""

    def __init__(self, target: MicTarget, cfg: ExternalMediaConfig):
        self.t = target
        self.cfg = cfg
        self.device_key = target.device_key
        self._recording = False

    def ping(self) -> bool:
        return True

    def start(self, date_str: str, time_str: str) -> bool:
        self._recording = True
        return True

    def stop(self) -> bool:
        self._recording = False
        return True

    def pull(self, dest_dir: Path, date_str: str, time_str: str) -> PullResult:
        result = PullResult(device_key=self.device_key)
        out_dir = Path(dest_dir) / "audio" / self.t.player_label
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_bytes = b"time_ms;vad\n0;0\n10;1\n20;0\n"
        n = _csv_bytes_to_parquet(csv_bytes, out_dir / "vad.parquet")
        result.files.append(
            PulledFile(f"audio/{self.t.player_label}/vad.parquet", n, True))
        if self.cfg.keep_raw_audio:
            flac = out_dir / "audio.flac"
            flac.write_bytes(b"fLaC-sim")
            result.files.append(PulledFile(
                f"audio/{self.t.player_label}/audio.flac",
                flac.stat().st_size, True))
        return result
