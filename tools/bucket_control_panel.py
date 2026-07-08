"""Tkinter bucket motor control panel for manual door testing.

Typical real-hardware workflow:
    $env:PYTHONPATH = "src"
    & C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe tools\\bucket_control_panel.py

Attach to already-running broker/controller processes instead:
    $env:PYTHONPATH = "src"
    & C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe tools\\bucket_control_panel.py --no-autostart

Direct command-line options:
    python tools\\bucket_control_panel.py --profile config\\profiles\\dev_bucket_integration.yaml
    python tools\\bucket_control_panel.py --stale-s 2.0
    python tools\\bucket_control_panel.py --startup-delay-s 1.0
    python tools\\bucket_control_panel.py --xsub tcp://127.0.0.1:5550 --xpub tcp://127.0.0.1:5551

This panel deliberately reuses the existing BucketController bus contract:
    - publishes sparse commands to ``cmd.bucket``
    - subscribes to live status on ``telem.bucket``

The bucket RS-485 port should stay owned by ``apps.bucket_controller`` so this
manual test exercises the same path used by the game.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from subsystems.bucket.common import BUCKET_LABELS  # noqa: E402


CMD_TOPIC = "cmd.bucket"  # Topic consumed by apps.bucket_controller for open/close/stop requests.
TELEM_TOPIC = "telem.bucket"  # Topic published by apps.bucket_controller with decoded motor status.
BUCKET_HEARTBEAT_TOPIC = "heartbeat.bucket_controller"  # Liveness topic from the bucket controller process.
BROKER_HEARTBEAT_TOPIC = "heartbeat.bus_broker"  # Liveness topic from the bus broker process.
PRODUCER = "bucket_control_panel"  # Producer name inserted into command envelopes for traceability.
DEFAULT_UI_REFRESH_MS = 100  # Tk refresh cadence; lower for snappier UI, higher for less CPU churn.
DEFAULT_STALE_TELEM_S = 2.0  # Seconds after the last telem.bucket before the UI marks data stale.
DEFAULT_PUB_GRACE_MS = 250  # Initial PUB warm-up for ZMQ slow-joiner mitigation.
DEFAULT_PROFILE = REPO_ROOT / "config" / "profiles" / "dev_bucket_integration.yaml"
DEFAULT_STARTUP_DELAY_S = 0.75  # Seconds between starting broker and controller; tune if startup races appear.
SHUTDOWN_GRACE_S = 3.0  # Seconds to wait for graceful child shutdown before forceful termination.
STATE_COLORS = {
    "limit": "#216e39",
    "moving": "#9a6700",
    "stopped": "#57606a",
    "unknown": "#8250df",
    "missing": "#cf222e",
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse CLI options that control bus endpoints and stale thresholds."""

    parser = argparse.ArgumentParser(description="Manual bucket door control panel")
    parser.add_argument(
        "--profile",
        default=str(DEFAULT_PROFILE),
        help="Profile YAML used when autostarting bus_broker and bucket_controller.",
    )
    parser.add_argument(
        "--no-autostart",
        action="store_true",
        help="Do not spawn bus_broker or bucket_controller; attach to existing processes.",
    )
    parser.add_argument(
        "--startup-delay-s",
        type=float,
        default=DEFAULT_STARTUP_DELAY_S,
        help="Seconds to wait between starting bus_broker and bucket_controller.",
    )
    parser.add_argument(
        "--xsub",
        default=bus.BUS_XSUB_ENDPOINT,
        help="Bus XSUB endpoint used for publishing cmd.bucket.",
    )
    parser.add_argument(
        "--xpub",
        default=bus.BUS_XPUB_ENDPOINT,
        help="Bus XPUB endpoint used for subscribing to telem.bucket.",
    )
    parser.add_argument(
        "--stale-s",
        type=float,
        default=DEFAULT_STALE_TELEM_S,
        help="Seconds before the latest bucket telemetry is considered stale.",
    )
    parser.add_argument(
        "--ui-ms",
        type=int,
        default=DEFAULT_UI_REFRESH_MS,
        help="Tkinter refresh period in milliseconds.",
    )
    return parser.parse_args(argv)


def short_text(value: object, *, max_chars: int = 64) -> str:
    """Return a compact single-line string for status/result cells."""

    text = "" if value is None else str(value)
    text = " ".join(text.split())
    if len(text) <= max_chars:
        return text
    return text[: max(0, max_chars - 1)] + "..."


def status_color(status: dict[str, Any] | None) -> str:
    """Return a stable color for the decoded motor status state."""

    if status is None:
        return STATE_COLORS["missing"]
    state = str(status.get("state") or "unknown")
    return STATE_COLORS.get(state, STATE_COLORS["unknown"])


def status_summary(status: dict[str, Any] | None) -> str:
    """Format one bucket status dict as a concise human-readable string."""

    if status is None:
        return "no status"
    state = str(status.get("state") or "unknown")
    raw = status.get("raw")
    raw_text = f"0x{int(raw):02X}" if isinstance(raw, int) else "?"
    description = short_text(status.get("description"), max_chars=42)
    return f"{state} ({raw_text}) {description}".strip()


def active_summary(active: dict[str, Any] | None) -> str:
    """Format the active watchdog command state for one bucket."""

    if not active:
        return "-"
    action = short_text(active.get("action"), max_chars=12)
    direction = short_text(active.get("direction"), max_chars=12)
    timeout = active.get("timeout_s")
    timeout_text = f"{float(timeout):.1f}s" if isinstance(timeout, (int, float)) else "?s"
    return f"{action} {direction}, timeout {timeout_text}"


def result_summary(result: dict[str, Any] | None) -> str:
    """Format the latest command result for one bucket."""

    if not result:
        return "-"
    ok = "OK" if result.get("ok") else "FAIL"
    action = short_text(result.get("action"), max_chars=12)
    message = short_text(result.get("message"), max_chars=52)
    return f"{ok} {action}: {message}"


def resolve_profile_path(raw_path: str) -> Path:
    """Resolve a profile path from either the caller's cwd or the repo root."""

    path = Path(raw_path)
    if path.is_absolute():
        return path
    cwd_candidate = Path.cwd() / path
    if cwd_candidate.exists():
        return cwd_candidate.resolve()
    return (REPO_ROOT / path).resolve()


class ChildProcessManager:
    """Start and stop the broker/controller processes needed by this panel."""

    def __init__(self, *, profile_path: Path, autostart: bool, startup_delay_s: float) -> None:
        self.profile_path = profile_path  # Profile passed to child process --profile arguments.
        self.autostart = bool(autostart)  # False means the panel only attaches to existing processes.
        self.startup_delay_s = max(0.0, float(startup_delay_s))  # Delay between broker and controller starts.
        self.children: list[tuple[str, subprocess.Popen]] = []  # Owned child processes to stop on close.
        self.startup_error: str | None = None  # Last process-spawn error shown in the GUI.

    def start(self) -> None:
        """Autostart bus_broker and bucket_controller when requested."""

        if not self.autostart:
            return
        if not self.profile_path.exists():
            self.startup_error = f"profile not found: {self.profile_path}"
            return
        try:
            self._spawn("bus_broker", "apps.bus_broker")
            if self.startup_delay_s > 0.0:
                time.sleep(self.startup_delay_s)
            self._spawn("bucket_controller", "apps.bucket_controller")
        except Exception as exc:
            self.startup_error = str(exc)

    def status_text(self) -> str:
        """Return a compact summary of owned child process state."""

        if not self.autostart:
            return "autostart off"
        if self.startup_error:
            return f"autostart error: {self.startup_error}"
        if not self.children:
            return "autostart pending"
        parts: list[str] = []
        for name, child in self.children:
            code = child.poll()
            if code is None:
                parts.append(f"{name} pid={child.pid}")
            else:
                parts.append(f"{name} exited rc={code}")
        return "; ".join(parts)

    def stop(self) -> None:
        """Stop owned children in reverse startup order."""

        for _, child in reversed(self.children):
            _terminate_child(child, grace_s=SHUTDOWN_GRACE_S)
        self.children.clear()

    def _spawn(self, proc_name: str, module_name: str) -> None:
        """Start one child module with the standard process CLI."""

        env = os.environ.copy()
        existing_pythonpath = env.get("PYTHONPATH")
        env["PYTHONPATH"] = (
            str(SRC)
            if not existing_pythonpath
            else f"{SRC}{os.pathsep}{existing_pythonpath}"
        )
        argv = [
            sys.executable,
            "-m",
            module_name,
            "--profile",
            str(self.profile_path),
            "--proc",
            proc_name,
        ]
        kwargs: dict[str, Any] = {
            "cwd": str(REPO_ROOT),
            "env": env,
        }
        if sys.platform == "win32":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]
        child = subprocess.Popen(argv, **kwargs)
        self.children.append((proc_name, child))


def _terminate_child(child: subprocess.Popen, *, grace_s: float) -> None:
    """Ask one child to exit, then terminate it if it does not stop."""

    if child.poll() is not None:
        return
    try:
        if sys.platform == "win32":
            child.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        else:
            child.terminate()
        child.wait(timeout=grace_s)
    except Exception:
        try:
            child.terminate()
            child.wait(timeout=1.0)
        except Exception:
            child.kill()


class BucketBusClient:
    """Own the ZMQ sockets used by the Tk panel."""

    def __init__(self, *, xsub_endpoint: str, xpub_endpoint: str, pub_grace_ms: int) -> None:
        self.ctx = zmq.Context.instance()  # Shared ZMQ context for this tool process.
        self.pub = bus.make_pub(self.ctx, endpoint=xsub_endpoint)  # Command publisher for cmd.bucket.
        self.sub = bus.make_sub(
            self.ctx,
            topics=[TELEM_TOPIC, BUCKET_HEARTBEAT_TOPIC, BROKER_HEARTBEAT_TOPIC],
            endpoint=xpub_endpoint,
        )  # Status subscriber for live bucket telemetry and process heartbeats.
        self.xsub_endpoint = xsub_endpoint  # Display-only publisher endpoint string.
        self.xpub_endpoint = xpub_endpoint  # Display-only subscriber endpoint string.
        self.ready_at = time.perf_counter() + (pub_grace_ms / 1000.0)  # Earliest safe command publish time.
        self.latest_telem: dict[str, Any] | None = None  # Most recent telem.bucket body.
        self.latest_telem_mono = 0.0  # Local receive timestamp for stale-data checks.
        self.latest_bucket_heartbeat: dict[str, Any] | None = None  # Most recent heartbeat.bucket_controller body.
        self.latest_bucket_heartbeat_mono = 0.0  # Local receive timestamp for bucket-controller age checks.
        self.latest_broker_heartbeat: dict[str, Any] | None = None  # Most recent heartbeat.bus_broker body.
        self.latest_broker_heartbeat_mono = 0.0  # Local receive timestamp for broker age checks.
        self.command_seq = 0  # Monotonic sequence for request_id generation.

    def drain(self) -> int:
        """Read every queued status message and return the count drained."""

        count = 0
        while True:
            try:
                topic, body = bus.recv(self.sub, flags=zmq.NOBLOCK)
            except zmq.Again:
                return count
            count += 1
            now = time.perf_counter()
            if topic == TELEM_TOPIC:
                self.latest_telem = body
                self.latest_telem_mono = now
            elif topic == BUCKET_HEARTBEAT_TOPIC:
                self.latest_bucket_heartbeat = body
                self.latest_bucket_heartbeat_mono = now
            elif topic == BROKER_HEARTBEAT_TOPIC:
                self.latest_broker_heartbeat = body
                self.latest_broker_heartbeat_mono = now

    def publish_bucket_command(self, action: str, *, bucket_label: str | None = None) -> str:
        """Publish one bucket command and return its generated request id."""

        wait_s = self.ready_at - time.perf_counter()
        if wait_s > 0.0:
            raise RuntimeError(f"publisher warming up; try again in {wait_s:.1f}s")
        self.command_seq += 1
        target = bucket_label or "all"
        request_id = f"manual-{target}-{action}-{self.command_seq}"
        body = bus.make_envelope(PRODUCER, with_wall=True)
        body.update({
            "action": action,
            "request_id": request_id,
            "reason": "manual_bucket_control_panel",
        })
        if bucket_label is not None:
            body["bucket_label"] = bucket_label
        bus.publish(self.pub, CMD_TOPIC, body)
        return request_id

    def close(self) -> None:
        """Close sockets and terminate this tool's ZMQ context."""

        self.pub.close(0)
        self.sub.close(0)
        self.ctx.destroy(linger=0)


class BucketRow:
    """Widget bundle and Tk variables for one bucket row."""

    def __init__(self, parent: ttk.Frame, *, row: int, label: str, command_fn: Any) -> None:
        self.label = label  # Logical bucket label, one of A1..B3.
        self.address_var = tk.StringVar(value="-")  # Modbus address reported by telem.bucket.
        self.status_var = tk.StringVar(value="waiting")  # Decoded status summary.
        self.motion_var = tk.StringVar(value="-")  # Direction/speed/moving/limit quick view.
        self.active_var = tk.StringVar(value="-")  # Active command watched by controller timeout.
        self.result_var = tk.StringVar(value="-")  # Latest command result for this bucket.
        self.error_var = tk.StringVar(value="-")  # Latest per-bucket error reported by controller.
        self.indicator = tk.Canvas(parent, width=18, height=18, highlightthickness=0)
        self.indicator.grid(row=row, column=0, padx=(0, 6), pady=3)
        self.indicator_dot = self.indicator.create_oval(3, 3, 15, 15, fill=STATE_COLORS["missing"], outline="")

        ttk.Label(parent, text=label, width=4).grid(row=row, column=1, sticky="w", pady=3)
        ttk.Label(parent, textvariable=self.address_var, width=7).grid(row=row, column=2, sticky="w", pady=3)
        ttk.Label(parent, textvariable=self.status_var, width=34).grid(row=row, column=3, sticky="w", pady=3)
        ttk.Label(parent, textvariable=self.motion_var, width=24).grid(row=row, column=4, sticky="w", pady=3)
        ttk.Label(parent, textvariable=self.active_var, width=28).grid(row=row, column=5, sticky="w", pady=3)
        ttk.Label(parent, textvariable=self.result_var, width=44).grid(row=row, column=6, sticky="w", pady=3)
        ttk.Label(parent, textvariable=self.error_var, width=24).grid(row=row, column=7, sticky="w", pady=3)

        buttons = ttk.Frame(parent)
        buttons.grid(row=row, column=8, sticky="e", padx=(8, 0), pady=3)
        ttk.Button(buttons, text="Open", width=7, command=lambda: command_fn("open", label)).grid(row=0, column=0)
        ttk.Button(buttons, text="Close", width=7, command=lambda: command_fn("close", label)).grid(row=0, column=1, padx=4)
        ttk.Button(buttons, text="Stop", width=7, command=lambda: command_fn("stop", label)).grid(row=0, column=2)

    def update_from_snapshot(self, snapshot: dict[str, Any] | None) -> None:
        """Refresh this row from one bucket's telemetry snapshot."""

        if snapshot is None:
            self.address_var.set("-")
            self.status_var.set("no telemetry")
            self.motion_var.set("-")
            self.active_var.set("-")
            self.result_var.set("-")
            self.error_var.set("-")
            self.indicator.itemconfigure(self.indicator_dot, fill=STATE_COLORS["missing"])
            return

        status = snapshot.get("status") if isinstance(snapshot.get("status"), dict) else None
        direction = status.get("direction") if status else None
        speed = status.get("speed") if status else None
        moving = bool(status.get("is_moving")) if status else False
        limit = bool(status.get("at_limit")) if status else False
        motion = f"dir={direction or '-'} speed={speed if speed is not None else '-'} moving={moving} limit={limit}"

        self.address_var.set(str(snapshot.get("address") or "-"))
        self.status_var.set(status_summary(status))
        self.motion_var.set(motion)
        self.active_var.set(active_summary(snapshot.get("active_command")))
        self.result_var.set(result_summary(snapshot.get("last_result")))
        self.error_var.set(short_text(snapshot.get("last_error") or "-", max_chars=24))
        self.indicator.itemconfigure(self.indicator_dot, fill=status_color(status))


class BucketControlPanel:
    """Tkinter app that displays bucket telemetry and sends manual commands."""

    def __init__(
        self,
        root: tk.Tk,
        *,
        client: BucketBusClient,
        process_manager: ChildProcessManager,
        stale_s: float,
        ui_ms: int,
    ) -> None:
        self.root = root  # Tk top-level window.
        self.client = client  # ZMQ bus client used for commands and telemetry.
        self.process_manager = process_manager  # Owned child process manager for autostart mode.
        self.stale_s = max(0.1, float(stale_s))  # Telemetry stale threshold in seconds.
        self.ui_ms = max(50, int(ui_ms))  # Refresh period in milliseconds.
        self.status_var = tk.StringVar(value="Waiting for telem.bucket...")  # Bottom status line text.
        self.summary_var = tk.StringVar(value="No telemetry yet")  # Top telemetry summary text.
        self.process_var = tk.StringVar(value=self.process_manager.status_text())  # Owned child process summary.
        self.endpoint_var = tk.StringVar(
            value=f"PUB {client.xsub_endpoint} / SUB {client.xpub_endpoint}"
        )  # Visible bus endpoint summary.
        self.rows: dict[str, BucketRow] = {}  # Bucket label -> row widget bundle.
        self._after_id: str | None = None  # Tk after handle for the update loop.
        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._schedule_update()

    def _build_layout(self) -> None:
        """Build the complete Tkinter layout."""

        self.root.title("Bucket Door Control Panel")
        self.root.minsize(1240, 390)
        outer = ttk.Frame(self.root, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="Bucket Door Manual Test", font=("Segoe UI", 14, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(header, textvariable=self.endpoint_var).grid(row=1, column=0, sticky="w", pady=(2, 0))
        ttk.Label(header, textvariable=self.process_var).grid(row=2, column=0, sticky="w", pady=(2, 0))
        ttk.Label(header, textvariable=self.summary_var).grid(row=3, column=0, sticky="w", pady=(2, 0))

        all_buttons = ttk.Frame(header)
        all_buttons.grid(row=0, column=1, rowspan=4, sticky="e")
        ttk.Button(all_buttons, text="Open All", command=lambda: self._send_all("open_all")).grid(row=0, column=0, padx=3)
        ttk.Button(all_buttons, text="Close All", command=lambda: self._send_all("close_all")).grid(row=0, column=1, padx=3)
        ttk.Button(all_buttons, text="Stop All", command=lambda: self._send_all("stop_all")).grid(row=0, column=2, padx=3)

        table = ttk.LabelFrame(outer, text="Live Bucket Motor Status", padding=8)
        table.grid(row=1, column=0, sticky="nsew", pady=(12, 8))
        for col in range(9):
            table.columnconfigure(col, weight=1 if col in (3, 5, 6) else 0)

        headings = ["", "Bucket", "Address", "Status", "Motion", "Active", "Last Result", "Error", "Command"]
        for col, text in enumerate(headings):
            ttk.Label(table, text=text, font=("Segoe UI", 9, "bold")).grid(row=0, column=col, sticky="w", pady=(0, 5))
        for row_index, label in enumerate(BUCKET_LABELS, start=1):
            self.rows[label] = BucketRow(table, row=row_index, label=label, command_fn=self._send_one)

        ttk.Label(outer, textvariable=self.status_var).grid(row=2, column=0, sticky="ew")

    def _schedule_update(self) -> None:
        """Schedule the next GUI refresh using Tk's event loop."""

        self._after_id = self.root.after(self.ui_ms, self._update_loop)

    def _update_loop(self) -> None:
        """Drain bus messages, refresh labels, and reschedule itself."""

        self.client.drain()
        self.process_var.set(self.process_manager.status_text())
        self._refresh_from_latest_telem()
        self._schedule_update()

    def _refresh_from_latest_telem(self) -> None:
        """Render the latest controller telemetry into the GUI."""

        telemetry = self.client.latest_telem
        now = time.perf_counter()
        telem_age = now - self.client.latest_telem_mono if self.client.latest_telem_mono else None
        bucket_heartbeat_age = (
            now - self.client.latest_bucket_heartbeat_mono
            if self.client.latest_bucket_heartbeat_mono
            else None
        )
        broker_heartbeat_age = (
            now - self.client.latest_broker_heartbeat_mono
            if self.client.latest_broker_heartbeat_mono
            else None
        )

        if telemetry is None:
            for row in self.rows.values():
                row.update_from_snapshot(None)
            self.summary_var.set(self._startup_hint(bucket_heartbeat_age, broker_heartbeat_age))
            self.status_var.set(self._heartbeat_text(bucket_heartbeat_age, broker_heartbeat_age))
            return

        stale = telem_age is not None and telem_age > self.stale_s
        buckets = telemetry.get("buckets") if isinstance(telemetry.get("buckets"), dict) else {}
        for label, row in self.rows.items():
            row.update_from_snapshot(buckets.get(label) if isinstance(buckets, dict) else None)

        connected = bool(telemetry.get("connected"))
        active_count = telemetry.get("active_count", 0)
        scan_hz = float(telemetry.get("observed_status_scan_hz") or 0.0)
        scan_ms = float(telemetry.get("last_scan_duration_ms") or 0.0)
        age_text = "stale" if stale else f"{telem_age:.2f}s old" if telem_age is not None else "age ?"
        self.summary_var.set(
            f"connected={connected} active={active_count} scan={scan_hz:.2f} Hz "
            f"scan_ms={scan_ms:.1f} telemetry={age_text}"
        )
        self.status_var.set(self._heartbeat_text(bucket_heartbeat_age, broker_heartbeat_age))

    def _startup_hint(self, bucket_heartbeat_age: float | None, broker_heartbeat_age: float | None) -> str:
        """Return a specific startup hint when bucket telemetry has not arrived."""

        if bucket_heartbeat_age is not None:
            return "bucket_controller heartbeat is alive, but no telem.bucket has arrived yet."
        if broker_heartbeat_age is not None:
            return "bus_broker is alive, but bucket_controller is not running or failed to start."
        return "No bus heartbeat received. Start bus_broker, then start bucket_controller, then launch this panel."

    def _heartbeat_text(self, bucket_heartbeat_age: float | None, broker_heartbeat_age: float | None) -> str:
        """Return the bottom status text describing controller liveness."""

        broker_text = (
            f"bus_broker heartbeat {broker_heartbeat_age:.2f}s old"
            if broker_heartbeat_age is not None
            else "no bus_broker heartbeat"
        )
        if bucket_heartbeat_age is None:
            return f"{broker_text}; no heartbeat.bucket_controller received."
        heartbeat = self.client.latest_bucket_heartbeat or {}
        loop_hz = float(heartbeat.get("loop_hz") or 0.0)
        return f"{broker_text}; bucket_controller heartbeat {bucket_heartbeat_age:.2f}s old, loop_hz={loop_hz:.1f}"

    def _send_one(self, action: str, label: str) -> None:
        """Publish one open/close/stop command for a single bucket."""

        try:
            request_id = self.client.publish_bucket_command(action, bucket_label=label)
        except Exception as exc:
            self.status_var.set(f"Command failed: {exc}")
            return
        self.status_var.set(f"Sent {action} for {label}: {request_id}")

    def _send_all(self, action: str) -> None:
        """Publish one all-bucket command."""

        try:
            request_id = self.client.publish_bucket_command(action)
        except Exception as exc:
            self.status_var.set(f"Command failed: {exc}")
            return
        self.status_var.set(f"Sent {action}: {request_id}")

    def close(self) -> None:
        """Stop the refresh loop, close ZMQ sockets, and destroy the window."""

        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        self.client.close()
        self.process_manager.stop()
        self.root.destroy()


def main(argv: list[str] | None = None) -> int:
    """Launch the bucket control panel."""

    args = parse_args(argv)
    process_manager = ChildProcessManager(
        profile_path=resolve_profile_path(args.profile),
        autostart=not args.no_autostart,
        startup_delay_s=args.startup_delay_s,
    )
    process_manager.start()
    root = tk.Tk()
    client = BucketBusClient(
        xsub_endpoint=args.xsub,
        xpub_endpoint=args.xpub,
        pub_grace_ms=DEFAULT_PUB_GRACE_MS,
    )
    try:
        BucketControlPanel(
            root,
            client=client,
            process_manager=process_manager,
            stale_s=args.stale_s,
            ui_ms=args.ui_ms,
        )
        root.mainloop()
    finally:
        process_manager.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
