"""Throwaway connectivity probe for the Jetson skeleton-tracker recorder node.

Exercises the WebSocket command protocol documented in
C:/Users/yck01/GitHub/jetson_camera_experiments/skeleton-tracker/jetson/tracker_api.md:
ping -> get_status -> short skeleton-only recording -> stop -> verify HTTP
file listing on :9100. Does NOT download files (that's tested separately).

Run:
    $env:PYTHONPATH = "src"
    & C:/Users/yck01/miniconda3/envs/game/python.exe tools/probe_jetson_recorder.py
    & C:/Users/yck01/miniconda3/envs/game/python.exe tools/probe_jetson_recorder.py --seconds 10 --video
"""

from __future__ import annotations

import argparse
import asyncio
import json
import urllib.request

import websockets

WS_URL = "ws://192.168.0.101:9000"   # Jetson recorder_node WebSocket command port
HTTP_BASE = "http://192.168.0.101:9100"  # Jetson read-only file server root
PROBE_DATE = "2099-01-01"            # clearly-fake probe session identity
PROBE_TIME = "99-99-99"


async def send_cmd(ws, command: str, args: dict | None = None) -> dict:
    """Send one command with a unique request_id and return its ack/status reply."""
    req_id = f"probe-{command}"
    msg = {"type": "command", "command": command, "request_id": req_id}
    if args:
        msg["args"] = args
    await ws.send(json.dumps(msg))
    # Wait for the reply matching our request_id (skip unsolicited events).
    while True:
        reply = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
        if reply.get("request_id") == req_id:
            return reply
        print(f"  (event: {reply})")


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="How long to record before stopping (default 5 s).")
    parser.add_argument("--video", action="store_true",
                        help="Also record raw video.mp4 (default: skeleton only).")
    ns = parser.parse_args()

    print(f"Connecting to {WS_URL} ...")
    async with websockets.connect(WS_URL) as ws:
        print("ping ->", json.dumps(await send_cmd(ws, "ping")))

        status = await send_cmd(ws, "get_status")
        print("get_status ->", json.dumps(status, indent=2))

        args = {"date": PROBE_DATE, "time": PROBE_TIME,
                "record_skeleton": True, "record_video": ns.video}
        print(f"start_recording {args} ->",
              json.dumps(await send_cmd(ws, "start_recording", args)))

        print(f"Recording for {ns.seconds} s ...")
        await asyncio.sleep(ns.seconds)

        status = await send_cmd(ws, "get_status")
        print("get_status (mid-recording) ->", json.dumps(status, indent=2))

        stop = await send_cmd(ws, "stop_recording")
        print("stop_recording ->", json.dumps(stop, indent=2))

    # Verify the files are visible over the HTTP file server.
    for team in ("a", "b"):
        url = f"{HTTP_BASE}/recordings/{PROBE_DATE}/{PROBE_TIME}/{team}/"
        try:
            with urllib.request.urlopen(url, timeout=10) as resp:
                listing = resp.read().decode(errors="replace")
            print(f"HTTP listing {url}:\n{listing[:800]}")
        except Exception as exc:  # noqa: BLE001 - probe: report anything
            print(f"HTTP listing {url} FAILED: {exc}")


if __name__ == "__main__":
    asyncio.run(main())
