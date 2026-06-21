"""On-disk format for recording and replaying the display-broadcast feed.

A *session recording* is the verbatim stream of ``state.full`` bodies that the
``state_broadcaster`` fans out to the player-display Raspberry Pis. Capturing
it lets a whole session -- spanning many games and every stage transition
(idle / daydreaming / tutorial / play / reset / conclusion, plus operator
e-stops and pauses) -- be replayed offline over UDP with ``state_replayer`` so
the display UI can be developed without the live rig.

File format (gzip-compressed newline-delimited JSON, ``.jsonl.gz``)
------------------------------------------------------------------
The file is a gzip stream of UTF-8 JSON lines (typically ~10x smaller than the
uncompressed text). Decompressed, the lines are:

* Line 1 is a **header** object: ``{"type": "header", ...metadata...}``.
* Each following line is a **frame**: ``{"seq": int, "ts_wall_ns": int,
  "state": {...state.full body...}}``. ``seq`` / ``ts_wall_ns`` are copied from
  the game controller's own envelope so replay reproduces the original cadence.
* The final line is a **footer**: ``{"type": "footer", "ended_wall_ns": int,
  "frame_count": int}``, written by ``RecordingWriter.close``.

The writer flushes periodically so an abrupt kill loses at most one buffer's
worth of frames; the gzip stream is still readable up to the last flushed
frame. The footer (and a clean close) is written on the normal shutdown path
-- e.g. when ESC in the dashboard tears the launcher down.

This module is the single source of truth shared by the recorder (inside
``apps.state_broadcaster``) and the player (``apps.state_replayer``).
"""

from __future__ import annotations

import gzip
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Iterator

# Bump when the on-disk line schema (header/frame/footer) changes in a
# backward-incompatible way so a reader can refuse a recording it cannot parse.
RECORDING_FORMAT_VERSION = 1


def default_recording_path(directory: str | Path, profile_name: str) -> Path:
    """Return a timestamped output path under ``directory`` for one session.

    Parameters
    ----------
    directory:
        Folder the recording is written into (created later by the writer).
    profile_name:
        Loaded profile name, embedded in the filename so captures from
        different profiles are easy to tell apart on disk.

    Returns
    -------
    Path
        ``<directory>/<YYYYmmdd_HHMMSS>_<profile>.jsonl.gz``. The profile name is
        sanitized to keep the filename filesystem-safe.
    """

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_profile = "".join(
        ch if (ch.isalnum() or ch in "-_") else "_" for ch in (profile_name or "session")
    )
    return Path(directory) / f"{stamp}_{safe_profile}.jsonl.gz"


class RecordingWriter:
    """Append-only writer for one display-broadcast session recording.

    The writer owns a file handle for its whole lifetime. Construct it when the
    broadcaster starts (recording enabled), call :meth:`append` once per
    broadcast frame, and :meth:`close` on shutdown to seal the file with a
    footer. :meth:`close` is idempotent so it is safe to call from both a normal
    teardown and a ``finally`` block.

    Parameters
    ----------
    path:
        Destination ``.jsonl`` file. Parent directories are created as needed.
    meta:
        Extra metadata merged into the header line (e.g. ``dest``, ``port``,
        ``target_hz``, ``profile``, ``protocol_version``). Keys are written
        verbatim, so keep them JSON-serializable.
    flush_every:
        Flush the OS buffer to disk every N appended frames. Lower = less data
        lost on a hard kill, higher = fewer syscalls. 30 frames at 60 Hz is a
        ~0.5 s worst-case loss window.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        meta: dict[str, Any] | None = None,
        flush_every: int = 30,
    ) -> None:
        self.path = Path(path)
        self._flush_every = max(1, int(flush_every))
        self._frame_count = 0  # number of frames appended so far
        self._closed = False

        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Buffered gzip text handle; we flush() explicitly on the cadence above
        # so the stream stays readable up to the last flushed frame.
        self._fh = gzip.open(self.path, "wt", encoding="utf-8")

        header = {
            "type": "header",
            "format_version": RECORDING_FORMAT_VERSION,
            "started_wall_ns": time.time_ns(),
        }
        if meta:
            header.update(meta)
        self._write_line(header)
        self._fh.flush()

    @property
    def frame_count(self) -> int:
        """Number of frames written so far (excludes header/footer)."""

        return self._frame_count

    def append(self, state: dict[str, Any], seq: int, ts_wall_ns: int) -> None:
        """Append one broadcast frame to the recording.

        Parameters
        ----------
        state:
            The ``state.full`` body being broadcast (recorded verbatim).
        seq:
            The game controller's monotonic state sequence number for this
            frame (used by the player to drop duplicates / reorder).
        ts_wall_ns:
            The frame's source wall-clock time in nanoseconds; the player uses
            successive values to reproduce the original inter-frame timing.
        """

        if self._closed:
            return
        self._write_line(
            {"seq": int(seq), "ts_wall_ns": int(ts_wall_ns), "state": state}
        )
        self._frame_count += 1
        if self._frame_count % self._flush_every == 0:
            self._fh.flush()

    def close(self) -> None:
        """Write the footer and close the file. Safe to call more than once."""

        if self._closed:
            return
        self._closed = True
        try:
            self._write_line(
                {
                    "type": "footer",
                    "ended_wall_ns": time.time_ns(),
                    "frame_count": self._frame_count,
                }
            )
            self._fh.flush()
        finally:
            self._fh.close()

    def __enter__(self) -> "RecordingWriter":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def _write_line(self, obj: dict[str, Any]) -> None:
        """Serialize one object as a compact JSON line."""

        self._fh.write(json.dumps(obj, separators=(",", ":")) + "\n")


def read_header(path: str | Path) -> dict[str, Any]:
    """Return the header object of a recording, or ``{}`` if absent/empty."""

    with gzip.open(Path(path), "rt", encoding="utf-8") as fh:
        first = fh.readline()
    if not first.strip():
        return {}
    try:
        obj = json.loads(first)
    except json.JSONDecodeError:
        return {}
    return obj if isinstance(obj, dict) else {}


def iter_frames(path: str | Path) -> Iterator[dict[str, Any]]:
    """Yield each frame object of a recording in file order.

    The header (first line) and footer (``type == "footer"``) are skipped, as
    are blank or malformed lines, so a recording truncated by a hard kill still
    replays every complete frame it managed to flush. A truncated gzip trailer
    (no clean close) is tolerated: iteration simply stops at the last fully
    decompressed line.
    """

    with gzip.open(Path(path), "rt", encoding="utf-8") as fh:
        index = 0
        while True:
            try:
                line = fh.readline()
            except (EOFError, OSError):
                # Truncated/unfinished gzip stream from an abrupt kill; stop at
                # the last frame that decompressed cleanly.
                break
            if not line:
                break
            current_index = index
            index += 1
            if not line.strip():
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(obj, dict):
                continue
            if current_index == 0 or obj.get("type") in ("header", "footer"):
                # Header is line 0; footer carries an explicit type tag.
                continue
            if not isinstance(obj.get("state"), dict):
                continue
            yield obj
