"""Enumerate all USB serial devices and print their VID/PID information.

Run directly:
    python src/enumerate_usb.py
"""

import serial.tools.list_ports


def enumerate_usb_devices():
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No COM ports found.")
        return

    print(f"{'Port':<10} {'VID':>6} {'PID':>6}  {'VID:PID Hex':<14} {'Description'}")
    print("-" * 80)

    for p in sorted(ports, key=lambda x: x.device):
        vid_str = f"{p.vid}" if p.vid is not None else "—"
        pid_str = f"{p.pid}" if p.pid is not None else "—"
        if p.vid is not None and p.pid is not None:
            hex_str = f"0x{p.vid:04X}:0x{p.pid:04X}"
        else:
            hex_str = "—"
        print(f"{p.device:<10} {vid_str:>6} {pid_str:>6}  {hex_str:<14} {p.description}")

        # Extra detail
        if p.manufacturer:
            print(f"{'':10}   manufacturer : {p.manufacturer}")
        if p.product:
            print(f"{'':10}   product      : {p.product}")
        if p.serial_number:
            print(f"{'':10}   serial       : {p.serial_number}")
        if p.hwid:
            print(f"{'':10}   hwid         : {p.hwid}")
        print()


if __name__ == "__main__":
    enumerate_usb_devices()
