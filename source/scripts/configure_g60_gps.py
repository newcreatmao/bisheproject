#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Iterable
from typing import Optional

import serial


BAUD_TO_PCAS_CODE = {
    4800: 0,
    9600: 1,
    19200: 2,
    38400: 3,
    57600: 4,
    115200: 5,
}

RATE_HZ_TO_FIX_INTERVAL_MS = {
    1: 1000,
    2: 500,
    4: 250,
    5: 200,
    10: 100,
}


def checksum_ok(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    body, checksum_text = line[1:].split("*", 1)
    checksum_text = checksum_text[:2]
    if len(checksum_text) != 2:
        return False

    checksum = 0
    for ch in body.encode("ascii", errors="ignore"):
        checksum ^= ch
    return f"{checksum:02X}" == checksum_text.upper()


def build_command(body: str) -> bytes:
    checksum = 0
    for ch in body.encode("ascii"):
        checksum ^= ch
    return f"${body}*{checksum:02X}\r\n".encode("ascii")


def open_serial(port: str, baud: int, timeout_sec: float) -> serial.Serial:
    return serial.Serial(port=port, baudrate=baud, timeout=timeout_sec)


def probe_baud(port: str, baud: int, probe_seconds: float) -> int:
    valid_lines = 0
    try:
        with open_serial(port, baud, 0.1) as ser:
            time.sleep(0.2)
            ser.reset_input_buffer()
            end_time = time.time() + probe_seconds
            while time.time() < end_time:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if checksum_ok(line):
                    valid_lines += 1
    except serial.SerialException:
        return 0
    return valid_lines


def detect_baud(port: str, probe_bauds: Iterable[int], probe_seconds: float) -> Optional[int]:
    ranked = []
    for baud in probe_bauds:
        valid_lines = probe_baud(port, baud, probe_seconds)
        print(f"[gps-config] probe baud={baud} valid_lines={valid_lines}", flush=True)
        if valid_lines > 0:
            ranked.append((valid_lines, baud))

    if not ranked:
        return None

    ranked.sort(reverse=True)
    return ranked[0][1]


def send_command(ser: serial.Serial, body: str, settle_sec: float = 0.25) -> None:
    command = build_command(body)
    print(f"[gps-config] send {command.decode('ascii').strip()}", flush=True)
    ser.write(command)
    ser.flush()
    time.sleep(settle_sec)


def verify_rate_hz(port: str, baud: int, verify_seconds: float) -> float:
    gga_count = 0
    with open_serial(port, baud, 0.1) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        end_time = time.time() + verify_seconds
        while time.time() < end_time:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if checksum_ok(line) and line.startswith("$") and "GGA," in line:
                gga_count += 1

    if verify_seconds <= 0.0:
        return 0.0
    return gga_count / verify_seconds


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Configure WHEELTEC G60 GPS runtime parameters")
    parser.add_argument("--port", default="/dev/gps")
    parser.add_argument("--probe-baud", dest="probe_bauds", action="append", type=int)
    parser.add_argument("--target-baud", type=int, default=115200)
    parser.add_argument("--rate-hz", type=int, default=10)
    parser.add_argument("--probe-seconds", type=float, default=1.2)
    parser.add_argument("--verify-seconds", type=float, default=1.4)
    parser.add_argument("--persist", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.target_baud not in BAUD_TO_PCAS_CODE:
        print(f"[gps-config] unsupported target baud: {args.target_baud}", file=sys.stderr, flush=True)
        return 2
    if args.rate_hz not in RATE_HZ_TO_FIX_INTERVAL_MS:
        print(f"[gps-config] unsupported rate: {args.rate_hz}Hz", file=sys.stderr, flush=True)
        return 2

    probe_bauds = args.probe_bauds or [9600, args.target_baud]
    seen = set()
    probe_bauds = [baud for baud in probe_bauds if not (baud in seen or seen.add(baud))]

    detected_baud = detect_baud(args.port, probe_bauds, args.probe_seconds)
    if detected_baud is None:
        print("[gps-config] failed to detect a valid NMEA baud on the GPS port", file=sys.stderr, flush=True)
        return 1

    print(f"[gps-config] detected baud={detected_baud}", flush=True)

    minimal_sentence_profile = "PCAS03,1,0,0,0,1,1,0,0,0,0,,,0,0,,,,0"
    target_baud_body = f"PCAS01,{BAUD_TO_PCAS_CODE[args.target_baud]}"
    target_rate_body = f"PCAS02,{RATE_HZ_TO_FIX_INTERVAL_MS[args.rate_hz]}"

    try:
        if detected_baud != args.target_baud:
            with open_serial(args.port, detected_baud, 0.2) as ser:
                time.sleep(0.3)
                ser.reset_input_buffer()
                send_command(ser, minimal_sentence_profile)
                send_command(ser, target_baud_body)
            time.sleep(0.6)

        with open_serial(args.port, args.target_baud, 0.2) as ser:
            time.sleep(0.5)
            ser.reset_input_buffer()
            send_command(ser, target_rate_body, settle_sec=0.5)
            send_command(ser, minimal_sentence_profile, settle_sec=0.5)
            if args.persist:
                send_command(ser, "PCAS00", settle_sec=0.5)
    except serial.SerialException as exc:
        print(f"[gps-config] serial error: {exc}", file=sys.stderr, flush=True)
        return 1

    measured_rate = verify_rate_hz(args.port, args.target_baud, args.verify_seconds)
    print(
        f"[gps-config] verified GGA rate ~= {measured_rate:.2f}Hz at baud={args.target_baud}",
        flush=True,
    )

    if measured_rate < max(1.0, args.rate_hz * 0.7):
        print(
            f"[gps-config] warning: expected about {args.rate_hz}Hz but measured {measured_rate:.2f}Hz; "
            "keeping the best-effort runtime configuration",
            flush=True,
        )
        return 0

    print("[gps-config] runtime configuration applied", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
