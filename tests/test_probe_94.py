"""Test probe using function 0x94 (query device address).

Sends the 0x94 command to each device address 1–8 on the first
detected RS485 port, retrying a few times per address to handle
lossy bus conditions.  Prints raw hex + decoded ASCII for every
response.

Expected reply from a present controller (two lines):
    RecvEnd\r\n
    0001\r\n          (the 4-digit device address)
"""

import time
import serial
import serial.tools.list_ports

# ── Config ──────────────────────────────────────────────────────────────────
BAUDRATE = 921600
MAX_ADDR = 8
RETRIES = 3            # attempts per address
RETRY_DELAY_S = 0.010  # 10 ms between retries
READ_TIMEOUT_S = 0.3   # how long to wait for reply bytes

# VID/PID for CH343 adapter (add others if needed)
VID_PIDS = {
    (0x1A86, 0x55D3),  # CH343
    (0x1A86, 0x7522),  # CH340
    (0x1A86, 0x7523),  # CH341
    (0x0403, 0x6001),  # FTDI FT232R
    (0x0403, 0x6015),  # FTDI FT-X
    (0x10C4, 0xEA60),  # CP210x
    (0x067B, 0x2303),  # PL2303
}

HEADER = bytes([0xDD, 0x55, 0xEE])
TAIL   = bytes([0xAA, 0xBB])


def build_query_address(device_addr: int) -> bytes:
    """Build a function-0x94 'query device address' command (21 bytes)."""
    return (
        HEADER
        + bytes([0x00, 0x00])                        # group addr (broadcast)
        + bytes([(device_addr >> 8) & 0xFF,
                  device_addr & 0xFF])               # device addr
        + bytes([0x00])                              # port
        + bytes([0x94])                              # function: query device addr
        + bytes([0x02])                              # LED type (WS2811)
        + bytes([0x00, 0x00])                        # reserved
        + bytes([0x00, 0x03])                        # data length = 3
        + bytes([0x00, 0x01])                        # repeat = 1
        + bytes([0x00, 0x00, 0x00])                  # colour placeholder
        + TAIL
    )


def find_port() -> str | None:
    for info in serial.tools.list_ports.comports():
        if info.vid is not None and (info.vid, info.pid) in VID_PIDS:
            return info.device
    return None


def probe_address(ser: serial.Serial, addr: int) -> bytes | None:
    """Send function 0x94 up to RETRIES times.  Return first non-empty reply."""
    cmd = build_query_address(addr)
    for attempt in range(1, RETRIES + 1):
        ser.reset_input_buffer()
        ser.write(cmd)
        ser.flush()

        # Wait for reply
        time.sleep(RETRY_DELAY_S)
        saved_timeout = ser.timeout
        ser.timeout = READ_TIMEOUT_S
        response = ser.read(256)
        ser.timeout = saved_timeout

        if response:
            return response
        # no reply — retry
    return None


def main():
    port = find_port()
    if not port:
        print("No RS485 adapter found.")
        return

    print(f"Using port: {port}  baud: {BAUDRATE}")
    print(f"Retries per address: {RETRIES}  retry delay: {RETRY_DELAY_S*1000:.0f} ms")
    print(f"Read timeout: {READ_TIMEOUT_S*1000:.0f} ms")
    print("=" * 60)

    ser = serial.Serial(
        port=port,
        baudrate=BAUDRATE,
        timeout=0.1,
        write_timeout=1.0,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    ser.reset_input_buffer()

    for addr in range(1, MAX_ADDR + 1):
        response = probe_address(ser, addr)
        if response:
            hex_str = response.hex(" ")
            text = response.decode("ascii", errors="replace").strip()
            lines = text.split("\n")
            print(f"  Addr {addr}: {len(response)} byte(s)  hex=[{hex_str}]")
            for i, line in enumerate(lines):
                print(f"           line {i}: {line.strip()!r}")
        else:
            print(f"  Addr {addr}: (no reply after {RETRIES} attempts)")

    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
