#!/usr/bin/env python3

import argparse
import sys
import time

import serial


RATE_TO_COMMAND = {
    1: bytes.fromhex("59 53 03 0A 00 01 0E 2B"),
    2: bytes.fromhex("59 53 03 0A 00 02 0F 2C"),
    5: bytes.fromhex("59 53 03 0A 00 03 10 2D"),
    10: bytes.fromhex("59 53 03 0A 00 04 11 2E"),
    20: bytes.fromhex("59 53 03 0A 00 05 12 2F"),
    25: bytes.fromhex("59 53 03 0A 00 06 13 30"),
    50: bytes.fromhex("59 53 03 0A 00 07 14 31"),
    100: bytes.fromhex("59 53 03 0A 00 08 15 32"),
    200: bytes.fromhex("59 53 03 0A 00 09 16 33"),
}

SUCCESS_RESPONSE = bytes.fromhex("59 53 03 0A 00 00 0D 2A")
FAIL_RESPONSE = bytes.fromhex("59 53 03 0A 00 FF 0C 29")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Configure WHEELTEC H30 IMU output frequency")
    parser.add_argument("--port", default="/dev/imu")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--rate-hz", type=int, default=50, choices=sorted(RATE_TO_COMMAND.keys()))
    parser.add_argument("--settle-seconds", type=float, default=0.2)
    parser.add_argument("--ack-timeout", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    command = RATE_TO_COMMAND[args.rate_hz]

    print(
        f"[imu-config] configure H30 port={args.port} baud={args.baud} target_rate={args.rate_hz}Hz",
        flush=True,
    )

    try:
        with serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1) as ser:
            time.sleep(args.settle_seconds)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(command)
            ser.flush()
            print(f"[imu-config] send {' '.join(f'{b:02X}' for b in command)}", flush=True)

            deadline = time.time() + args.ack_timeout
            received = bytearray()
            while time.time() < deadline:
                chunk = ser.read(256)
                if chunk:
                    received.extend(chunk)
                    if SUCCESS_RESPONSE in received:
                        print("[imu-config] device acknowledged output rate change", flush=True)
                        return 0
                    if FAIL_RESPONSE in received:
                        print("[imu-config] device returned failure response", file=sys.stderr, flush=True)
                        return 1
    except serial.SerialException as exc:
        print(f"[imu-config] serial error: {exc}", file=sys.stderr, flush=True)
        return 1

    print(
        "[imu-config] no explicit ack received; continuing with best-effort runtime configuration",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
