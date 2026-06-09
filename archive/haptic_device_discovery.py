# device_discovery.py
# Auto-discovers and identifies ESP32 haptic controllers on serial ports.
#
# Each controller stores one persistent dial ID in NVS flash (set via the I command).
# This module probes all available COM ports, confirms firmware presence via the
# V (version) command, and reads motor identity via the I (identity) command.
#
# The devices are already streaming telemetry (T,...) lines at 50 Hz the moment
# the port is opened, so the probe logic reads through that noise looking for
# the specific V or I response line.
#
# Usage from application code:
#   from device_discovery import discover_devices, build_device_map, assign_identity
#
#   devices = discover_devices()
#   # [{"port": "COM4", "fw_version": "0.3.0", "dial_id": 11}, ...]
#
#   device_map = build_device_map()
#   # {11: "COM4", 12: "COM5"}
#
#   # One-time provisioning (writes to NVS flash, survives reboot):
#   assign_identity("COM4", dial_id=11)

import serial
import serial.tools.list_ports
import time

BAUD = 115200
PROBE_TIMEOUT = 1.5  # seconds to wait for a response per command
DRAIN_DELAY = 0.2  # seconds to wait after opening port to let telemetry flow

# Known USB VID/PID pairs for common ESP32 USB-serial chips.
ESP32_VID_PIDS = {
    (0x1A86, 0x7523),  # CH340
    (0x1A86, 0x55D4),  # CH9102
    (0x10C4, 0xEA60),  # CP2102
    (0x303A, 0x1001),  # ESP32-S2/S3 native USB
    (0x303A, 0x0002),  # ESP32-S3 JTAG
}

DEFAULT_DEPLOY_VID = 0x1A86
DEFAULT_DEPLOY_PID = 0x7523


def list_candidate_ports(filter_by_vid_pid=True):
    """Return list of serial port info dicts for likely ESP32 devices.

    Each entry: {"port": "COMx", "description": "...", "vid_pid": "1A86:7523" or None}
    If filter_by_vid_pid is True, only ports matching known ESP32 chips are returned.
    Falls back to all ports if no VID/PID matches are found.
    """
    ports = serial.tools.list_ports.comports()

    def port_info(p):
        vid_pid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else None
        return {"port": p.device, "description": p.description, "vid_pid": vid_pid}

    if not filter_by_vid_pid:
        return [port_info(p) for p in ports]

    candidates = [
        port_info(p)
        for p in ports
        if p.vid is not None and (p.vid, p.pid) in ESP32_VID_PIDS
    ]
    # If VID/PID filtering yields nothing, fall back to all ports
    if not candidates:
        candidates = [port_info(p) for p in ports]
    return candidates


def list_ports_by_exact_vid_pid(vid, pid):
    """Return ports whose USB serial bridge matches one exact VID/PID pair."""

    ports = serial.tools.list_ports.comports()
    matches = []
    for p in ports:
        if p.vid == vid and p.pid == pid:
            matches.append(
                {
                    "port": p.device,
                    "description": p.description,
                    "vid_pid": f"{vid:04X}:{pid:04X}",
                }
            )
    return matches


def _send_and_wait(ser, command, expected_prefix, timeout=PROBE_TIMEOUT):
    """Send a command and wait for a line starting with expected_prefix.

    Returns the full response line (stripped) or None on timeout.
    The device is constantly streaming telemetry (T,...) lines, so this
    reads and discards any lines that don't match the expected prefix.
    """
    ser.reset_input_buffer()
    ser.write((command + "\n").encode())
    deadline = time.monotonic() + timeout
    buf = ""
    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buf += data.decode(errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line.startswith(expected_prefix):
                    return line
    return None


def probe_port(port, baud=BAUD):
    """Probe a single serial port to check if it's one of our controllers.

    Opens the port, sends a V (version) command and an I (identity) command,
    reads through streaming telemetry to find the responses.

    Returns a dict with keys: port, fw_version, dial_id (int),
    or None if the port doesn't respond correctly.
    """
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except (serial.SerialException, OSError):
        return None

    try:
        # Wait briefly — the device is already streaming telemetry.
        # This gives time for any partial line in the buffer to complete.
        time.sleep(DRAIN_DELAY)
        ser.reset_input_buffer()

        # 1. Version check — confirms this is our firmware
        resp = _send_and_wait(ser, "V,1", "V,1,")
        if resp is None:
            return None

        parts = resp.split(",")
        fw_version = parts[2] if len(parts) >= 3 else "unknown"

        # 2. Identity query — get the persistent dial ID from NVS
        resp = _send_and_wait(ser, "I,2", "I,2,")
        if resp is None:
            return None

        parts = resp.split(",")
        if len(parts) >= 3:
            dial_id = int(parts[2])
        else:
            dial_id = 0

        return {
            "port": port,
            "fw_version": fw_version,
            "dial_id": dial_id,
        }
    finally:
        ser.close()


def discover_devices(baud=BAUD, filter_by_vid_pid=True):
    """Scan all candidate serial ports and return a list of discovered devices.

    Each entry is a dict: {"port": "COMx", "fw_version": "...", "dial_id": id}
    Devices with dial_id 0 have not been provisioned yet.
    """
    candidates = list_candidate_ports(filter_by_vid_pid)
    devices = []
    for candidate in candidates:
        info = probe_port(candidate["port"], baud)
        if info is not None:
            devices.append(info)
    return devices


def discover_devices_on_ports(ports, baud=BAUD):
    """Probe a caller-specified port list and return devices that respond."""

    devices = []
    for port in ports:
        info = probe_port(port, baud)
        if info is not None:
            devices.append(info)
    return devices


def discover_devices_by_exact_vid_pid(vid=DEFAULT_DEPLOY_VID, pid=DEFAULT_DEPLOY_PID, baud=BAUD):
    """Probe only ports that match one exact VID/PID pair."""

    ports = [candidate["port"] for candidate in list_ports_by_exact_vid_pid(vid, pid)]
    return discover_devices_on_ports(ports, baud=baud)


def assign_identity(port, dial_id, baud=BAUD):
    """Write dial identity to a device's NVS flash (persistent across reboots).

    Returns the confirmed dial_id read back from the device.
    """
    ser = serial.Serial(port, baud, timeout=0.1)
    try:
        time.sleep(DRAIN_DELAY)
        ser.reset_input_buffer()
        cmd = f"I,1,{dial_id}"
        resp = _send_and_wait(ser, cmd, "I,1,")
        if resp is None:
            raise RuntimeError(f"No response from {port} after identity assignment")
        parts = resp.split(",")
        if len(parts) >= 3:
            return int(parts[2])
        return None
    finally:
        ser.close()


def build_device_map(baud=BAUD):
    """Discover all devices and return a dict mapping dial_id -> port name.

    Example return: {11: "COM4", 12: "COM5"}
    """
    devices = discover_devices(baud)
    device_map = {}
    for dev in devices:
        current_dial_id = dev["dial_id"]
        if current_dial_id == 0:
            print(f"  WARNING: Unconfigured device on {dev['port']} (dial_id=0)")
        if current_dial_id in device_map:
            print(
                f"  WARNING: Duplicate dial_id {current_dial_id} on {dev['port']} and {device_map[current_dial_id]}"
            )
        device_map[current_dial_id] = dev["port"]
    return device_map


# =========================
# Test / demo — just run this file directly
# =========================
if __name__ == "__main__":

    print("=== Candidate COM ports (VID/PID filtered) ===")
    candidates = list_candidate_ports(filter_by_vid_pid=True)
    if not candidates:
        print("  (none found)")
    for c in candidates:
        print(f"  {c['port']:8s}  {c['vid_pid'] or '----:----'}  {c['description']}")

    print()
    print("=== Probing for haptic controllers ===")
    devices = discover_devices()
    if not devices:
        print("  No controllers found.")
    for dev in devices:
        status = "UNCONFIGURED" if dev["dial_id"] == 0 else "ok"
        print(
            f"  {dev['port']:8s}  fw={dev['fw_version']}  "
            f"dial_id={dev['dial_id']}  [{status}]"
        )

    print()
    print("=== Device map (dial_id -> port) ===")
    device_map = build_device_map()
    if not device_map:
        print("  (empty)")
    for current_dial_id, port in device_map.items():
        print(f"  dial_id={current_dial_id} -> {port}")
