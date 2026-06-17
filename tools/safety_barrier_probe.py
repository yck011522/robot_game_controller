"""CLI probe for Modbus RTU safety barrier input modules.

The probe polls four RS485 IO units by default. Each unit is expected to expose
two discrete input channels via Modbus function 02 starting at input address 0.

Example:
    python tools/safety_barrier_probe.py
    python tools/safety_barrier_probe.py --port COM43 --baud 115200 --addresses 1 2 3 4
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on machines without pyserial.
    serial = None

DEFAULT_PORT = "COM43"  # fallback COM port when config/com_ports.yaml has no safety_barrier entry
DEFAULT_BAUD = 115200  # UART baud rate configured on the RS485 IO units
DEFAULT_TIMEOUT_MS = 70.0  # read timeout per device; lower values raise max FPS but punish slow replies
DEFAULT_ADDRESSES = (1, 2, 3, 4)  # Modbus slave addresses currently assigned to the four IO units
MODBUS_READ_DISCRETE_INPUTS = 0x02  # Modbus function code used by the provided working frame
DEFAULT_START_ADDRESS = 0  # first discrete input address to read from each module
DEFAULT_INPUT_COUNT = 2  # number of input bits to read from each module
DEFAULT_INTER_REQUEST_DELAY_MS = 6.0  # measured stable RS485 quiet gap between module polls on COM43
COM_PORTS_PATH = Path(__file__).resolve().parent.parent / "config" / "com_ports.yaml"  # local serial map


@dataclass(frozen=True)
class ReadResult:
    """One Modbus read result for a single safety barrier IO unit."""

    address: int  # Modbus slave address that was polled
    channel_1: int | None  # input channel 1 value; None means the read failed
    channel_2: int | None  # input channel 2 value; None means the read failed
    raw_value: int | None  # raw response data byte containing both input bits
    error: str | None = None  # short error description for CLI display
    response_hex: str = ""  # raw response frame captured for debug output
    elapsed_ms: float = 0.0  # measured request-to-response duration for this module poll


@dataclass
class RateMeter:
    """Rolling cycle-rate estimator for the overwrite-one-line display."""

    update_period_s: float = 0.5  # minimum time between visible FPS recalculations
    last_update_t: float = 0.0  # monotonic timestamp for the previous FPS recalculation
    last_cycles: int = 0  # completed polling cycles at the previous FPS recalculation
    fps: float = 0.0  # most recent cycles-per-second estimate across all devices

    def update(self, now_t: float, cycles: int) -> float:
        """Return the latest FPS estimate, refreshing it every update period."""

        elapsed_s = now_t - self.last_update_t  # seconds since the last FPS recalculation
        if self.last_update_t <= 0.0:
            self.last_update_t = now_t
            self.last_cycles = cycles
            return self.fps
        if elapsed_s >= self.update_period_s:
            cycle_delta = cycles - self.last_cycles  # completed polling loops in the sample window
            self.fps = cycle_delta / elapsed_s if elapsed_s > 0.0 else 0.0
            self.last_update_t = now_t
            self.last_cycles = cycles
        return self.fps


def modbus_crc16(frame: bytes) -> int:
    """Compute the Modbus RTU CRC16 value for a request or response frame."""

    crc = 0xFFFF  # Modbus RTU CRC seed value
    for byte in frame:
        crc ^= byte
        for _bit_index in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame_without_crc: bytes) -> bytes:
    """Return a Modbus RTU frame with its little-endian CRC bytes appended."""

    crc = modbus_crc16(frame_without_crc)  # integer CRC before Modbus little-endian byte packing
    return frame_without_crc + crc.to_bytes(2, byteorder="little")


def build_read_inputs_request(address: int, start_address: int, input_count: int) -> bytes:
    """Build a Modbus function-02 request for a module's discrete inputs."""

    body = bytes(
        [
            address & 0xFF,
            MODBUS_READ_DISCRETE_INPUTS,
            (start_address >> 8) & 0xFF,
            start_address & 0xFF,
            (input_count >> 8) & 0xFF,
            input_count & 0xFF,
        ]
    )
    return append_crc(body)


def read_exact(ser: serial.Serial, byte_count: int) -> bytes:
    """Read exactly byte_count bytes unless the serial timeout expires first."""

    chunks = bytearray()  # accumulated response bytes from the serial driver
    while len(chunks) < byte_count:
        chunk = ser.read(byte_count - len(chunks))
        if not chunk:
            break
        chunks.extend(chunk)
    return bytes(chunks)


def parse_read_inputs_response(address: int, response: bytes, elapsed_ms: float = 0.0) -> ReadResult:
    """Validate and decode a Modbus function-02 discrete-input response."""

    response_hex = response.hex(" ")  # compact response dump used when --debug is enabled
    minimum_normal_length = 6  # address + function + byte_count + one data byte + two CRC bytes
    if len(response) < minimum_normal_length:
        return ReadResult(address, None, None, None, f"short:{response_hex}", response_hex, elapsed_ms)

    payload = response[:-2]  # response bytes covered by the CRC
    received_crc = int.from_bytes(response[-2:], byteorder="little")  # CRC supplied by the IO unit
    expected_crc = modbus_crc16(payload)  # CRC computed locally from the response payload
    if received_crc != expected_crc:
        return ReadResult(address, None, None, None, "crc", response_hex, elapsed_ms)

    response_address = response[0]  # slave address echoed by the IO unit
    response_function = response[1]  # Modbus function code or exception code
    if response_address != address:
        return ReadResult(address, None, None, None, f"addr:{response_address}", response_hex, elapsed_ms)
    if response_function == (MODBUS_READ_DISCRETE_INPUTS | 0x80):
        exception_code = response[2]  # Modbus exception reason from the slave
        return ReadResult(address, None, None, None, f"exc:{exception_code}", response_hex, elapsed_ms)
    if response_function != MODBUS_READ_DISCRETE_INPUTS:
        return ReadResult(address, None, None, None, f"func:{response_function}", response_hex, elapsed_ms)

    byte_count = response[2]  # number of data bytes in the function-02 response
    if byte_count < 1:
        return ReadResult(address, None, None, None, "empty", response_hex, elapsed_ms)

    raw_value = response[3]  # first data byte; bit 0 is channel 1 and bit 1 is channel 2
    channel_1 = 1 if (raw_value & 0x01) else 0
    channel_2 = 1 if (raw_value & 0x02) else 0
    return ReadResult(address, channel_1, channel_2, raw_value, None, response_hex, elapsed_ms)


def read_module(
    ser: serial.Serial,
    *,
    address: int,
    start_address: int,
    input_count: int,
    flush_per_request: bool,
) -> ReadResult:
    """Poll one IO unit and return its two decoded input values."""

    request = build_read_inputs_request(address, start_address, input_count)  # RTU request sent on RS485
    if flush_per_request:
        ser.reset_input_buffer()
    started_t = time.perf_counter()  # high-resolution timestamp for this one Modbus transaction
    ser.write(request)
    ser.flush()

    header = read_exact(ser, 3)  # address + function + byte_count/exception code
    elapsed_ms = (time.perf_counter() - started_t) * 1000.0  # duration through the header read
    if len(header) < 3:
        return ReadResult(address, None, None, None, "timeout", header.hex(" "), elapsed_ms)

    response_function = header[1]  # normal function code or Modbus exception marker
    if response_function == (MODBUS_READ_DISCRETE_INPUTS | 0x80):
        tail_length = 2  # exception response only needs CRC after the exception code in header[2]
    else:
        tail_length = header[2] + 2  # data byte count plus CRC bytes

    tail = read_exact(ser, tail_length)  # remaining response body and CRC
    elapsed_ms = (time.perf_counter() - started_t) * 1000.0  # duration through the complete response read
    return parse_read_inputs_response(address, header + tail, elapsed_ms)


def configured_safety_barrier_port() -> str:
    """Return the configured safety_barrier COM port, or the probe fallback."""

    if COM_PORTS_PATH.exists():
        config_text = COM_PORTS_PATH.read_text(encoding="utf-8")  # installation-local COM-port YAML text
        match = re.search(r"^\s*safety_barrier:\s*[\"']?([^\"'\s#]+)", config_text, flags=re.MULTILINE)
        if match:
            return match.group(1).strip()
    return DEFAULT_PORT


def parse_addresses(values: Iterable[int]) -> list[int]:
    """Validate Modbus addresses from the CLI."""

    addresses = list(values)  # concrete list so it can be reused during the polling loop
    if not addresses:
        raise argparse.ArgumentTypeError("at least one address is required")
    for address in addresses:
        if address < 1 or address > 247:
            raise argparse.ArgumentTypeError("Modbus addresses must be in the range 1..247")
    return addresses


def format_result(result: ReadResult) -> str:
    """Format one device result as a compact CLI field."""

    if result.error is not None:
        return f"A{result.address}:??({result.error})"
    return f"A{result.address}:{result.channel_1}{result.channel_2}"


def format_status_line(results: list[ReadResult], fps: float, cycles: int, errors: int) -> str:
    """Build the one-line timestamp, input states, and refresh-rate display."""

    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # wall-clock timestamp with milliseconds
    values_text = " ".join(format_result(result) for result in results)  # compact per-address values
    return f"{timestamp} | {values_text} | FPS={fps:7.1f} | cycles={cycles} | errors={errors}"


def print_debug_cycle(cycles: int, results: list[ReadResult]) -> None:
    """Print one verbose debug line per module for the completed polling cycle."""

    for result in results:
        state_text = "??" if result.error is not None else f"{result.channel_1}{result.channel_2}"
        print(
            f"cycle={cycles} A{result.address} state={state_text} "
            f"elapsed_ms={result.elapsed_ms:6.2f} error={result.error or '-'} rx={result.response_hex}",
            flush=True,
        )


def run_probe(args: argparse.Namespace) -> int:
    """Open the serial port and continuously poll the configured IO units."""

    if serial is None:
        print("ERROR: pyserial is not installed. Install requirements.txt first.", file=sys.stderr)
        return 2

    addresses = parse_addresses(args.addresses)  # ordered Modbus slave addresses to poll each cycle
    timeout_s = max(0.001, args.timeout_ms / 1000.0)  # serial read timeout in seconds
    write_timeout_s = max(0.001, args.write_timeout_ms / 1000.0)  # serial write timeout in seconds
    inter_request_delay_s = max(0.0, args.inter_request_delay_ms / 1000.0)  # optional pause after each device
    max_cycles = max(0, args.cycles)  # finite sweep count; 0 keeps polling until Ctrl+C
    rate = RateMeter()  # rolling FPS estimator for full four-device polling cycles
    cycles = 0  # number of complete address sweeps finished
    errors = 0  # cumulative failed module reads
    last_line_length = 0  # previous line width so shorter updates erase cleanly

    print(
        "Opening safety barrier probe "
        f"port={args.port} baud={args.baud} addresses={addresses} "
        f"timeout_ms={args.timeout_ms:g} inter_request_delay_ms={args.inter_request_delay_ms:g}",
        flush=True,
    )
    print("Press Ctrl+C to stop.", flush=True)

    with serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout_s,
        write_timeout=write_timeout_s,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        try:
            while True:
                results: list[ReadResult] = []  # results collected during this complete polling cycle
                for address in addresses:
                    result = read_module(
                        ser,
                        address=address,
                        start_address=args.start_address,
                        input_count=args.input_count,
                        flush_per_request=not args.no_flush_per_request,
                    )
                    if result.error is not None:
                        errors += 1
                    results.append(result)
                    if inter_request_delay_s > 0.0:
                        time.sleep(inter_request_delay_s)

                cycles += 1
                now_t = time.monotonic()  # monotonic timestamp used only for rate measurement
                fps = rate.update(now_t, cycles)
                line = format_status_line(results, fps, cycles, errors)
                padding = " " * max(0, last_line_length - len(line))  # erase leftovers from longer lines
                print(f"\r{line}{padding}", end="", flush=True)
                last_line_length = len(line)
                if args.debug:
                    print()
                    print_debug_cycle(cycles, results)
                    last_line_length = 0
                if max_cycles and cycles >= max_cycles:
                    print()
                    return 0
        except KeyboardInterrupt:
            print()
            return 0


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser for the safety barrier probe."""

    parser = argparse.ArgumentParser(description="Poll Modbus RTU safety barrier input modules.")
    parser.add_argument("--port", default=configured_safety_barrier_port(), help="Serial COM port.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate.")
    parser.add_argument(
        "--addresses",
        nargs="+",
        type=int,
        default=list(DEFAULT_ADDRESSES),
        help="Modbus slave addresses to poll in order.",
    )
    parser.add_argument("--timeout-ms", type=float, default=DEFAULT_TIMEOUT_MS, help="Read timeout per module.")
    parser.add_argument("--write-timeout-ms", type=float, default=DEFAULT_TIMEOUT_MS, help="Write timeout.")
    parser.add_argument("--start-address", type=int, default=DEFAULT_START_ADDRESS, help="First input address.")
    parser.add_argument("--input-count", type=int, default=DEFAULT_INPUT_COUNT, help="Input bits to read.")
    parser.add_argument("--cycles", type=int, default=0, help="Number of full polling cycles to run; 0 runs forever.")
    parser.add_argument("--debug", action="store_true", help="Print per-address response hex and timing.")
    parser.add_argument(
        "--inter-request-delay-ms",
        type=float,
        default=DEFAULT_INTER_REQUEST_DELAY_MS,
        help="Optional delay after each module poll.",
    )
    parser.add_argument(
        "--no-flush-per-request",
        action="store_true",
        help="Do not clear pending input bytes before each request; useful for max-rate experiments.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Parse command-line arguments and run the safety barrier probe."""

    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run_probe(args)
    except Exception as exc:  # noqa: BLE001
        if serial is None or not isinstance(exc, serial.SerialException):
            raise
        print(f"\nERROR: serial failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
