#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Optional

import serial


RATE_TO_CODE = {
    1: 0x01,
    2: 0x02,
    5: 0x03,
    10: 0x04,
    20: 0x05,
    25: 0x06,
    50: 0x07,
    100: 0x08,
    200: 0x09,
}

MODE_TO_CODE = {
    "ahrs": 0x01,
    "vru": 0x02,
    "imu": 0x03,
}

OUTPUT_PROFILE_TO_MASK = {
    # Use the vendor's full attitude output profile so the project keeps access
    # to acc/gyro/mag/euler/quaternion data with a deterministic startup config.
    "project": 0x00F8,
    "all_attitude": 0x00F8,
    "minimal": 0x00D8,
    "imu_only": 0x00C0,
}

CLASS_OUTPUT_RATE = 0x03
CLASS_OUTPUT_CONTENT = 0x04
CLASS_FUNCTION_MODE = 0x4D
SUBTYPE_ALGORITHM_MODE = 0x02
SERIAL_TIMEOUT_SEC = 0.1


def parse_mask(value: str) -> int:
    mask = int(value, 0)
    if mask < 0 or mask > 0xFFFF:
        raise argparse.ArgumentTypeError("output mask must be between 0x0000 and 0xFFFF")
    return mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Configure WHEELTEC H30 IMU mode, output content, and output frequency"
    )
    parser.add_argument("--port", default="/dev/imu")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--rate-hz", type=int, default=50, choices=sorted(RATE_TO_CODE.keys()))
    parser.add_argument(
        "--heading-mode",
        default="vru",
        choices=sorted(MODE_TO_CODE.keys()),
        help="H30 algorithm mode: vru is the safest default around magnetic interference",
    )
    parser.add_argument(
        "--output-profile",
        default="project",
        choices=sorted(OUTPUT_PROFILE_TO_MASK.keys()),
        help="Predefined H30 output content mask",
    )
    parser.add_argument(
        "--output-mask",
        type=parse_mask,
        help="Override output profile with an explicit 16-bit output content mask",
    )
    parser.add_argument("--settle-seconds", type=float, default=0.2)
    parser.add_argument("--ack-timeout", type=float, default=1.0)
    parser.set_defaults(persist=True)
    parser.add_argument("--persist", dest="persist", action="store_true")
    parser.add_argument("--no-persist", dest="persist", action="store_false")
    return parser.parse_args()


def checksum_bytes(payload: bytes) -> bytes:
    ck1 = 0
    ck2 = 0
    for value in payload:
        ck1 = (ck1 + value) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return bytes([ck1, ck2])


def build_frame(class_id: int, payload: bytes, persist: bool) -> bytes:
    op = 0x02 if persist else 0x01
    length = len(payload)
    control = (length << 3) | op
    body = bytes([class_id, control & 0xFF, (control >> 8) & 0xFF]) + payload
    return b"\x59\x53" + body + checksum_bytes(body)


def build_mode_payload(mode: str) -> bytes:
    return bytes([SUBTYPE_ALGORITHM_MODE, MODE_TO_CODE[mode]])


def build_output_mask_payload(mask: int) -> bytes:
    return mask.to_bytes(2, byteorder="little", signed=False)


def build_success_ack(class_id: int, persist: bool) -> bytes:
    return build_frame(class_id, b"\x00", persist)


def build_failure_ack(class_id: int, persist: bool) -> bytes:
    return build_frame(class_id, b"\xFF", persist)


def send_command(
    ser: serial.Serial,
    label: str,
    class_id: int,
    payload: bytes,
    persist: bool,
    ack_timeout: float,
) -> Optional[bool]:
    command = build_frame(class_id, payload, persist)
    success_ack = build_success_ack(class_id, persist)
    failure_ack = build_failure_ack(class_id, persist)

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(command)
    ser.flush()
    print(f"[imu-config] send {label}: {' '.join(f'{b:02X}' for b in command)}", flush=True)

    deadline = time.time() + ack_timeout
    received = bytearray()
    while time.time() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        received.extend(chunk)
        if success_ack in received:
            print(f"[imu-config] {label} acknowledged", flush=True)
            return True
        if failure_ack in received:
            print(f"[imu-config] {label} rejected by device", file=sys.stderr, flush=True)
            return False

    print(
        f"[imu-config] no explicit ack for {label}; keeping best-effort runtime configuration",
        flush=True,
    )
    return None


def main() -> int:
    args = parse_args()
    output_mask = (
        args.output_mask
        if args.output_mask is not None
        else OUTPUT_PROFILE_TO_MASK[args.output_profile]
    )

    print(
        "[imu-config] configure H30 "
        f"port={args.port} baud={args.baud} mode={args.heading_mode} "
        f"rate={args.rate_hz}Hz output_mask=0x{output_mask:04X} "
        f"persist={'yes' if args.persist else 'no'}",
        flush=True,
    )

    try:
        with serial.Serial(port=args.port, baudrate=args.baud, timeout=SERIAL_TIMEOUT_SEC) as ser:
            time.sleep(args.settle_seconds)
            mode_result = send_command(
                ser,
                f"algorithm_mode={args.heading_mode}",
                CLASS_FUNCTION_MODE,
                build_mode_payload(args.heading_mode),
                args.persist,
                args.ack_timeout,
            )
            if mode_result is False:
                return 1

            output_result = send_command(
                ser,
                f"output_mask=0x{output_mask:04X}",
                CLASS_OUTPUT_CONTENT,
                build_output_mask_payload(output_mask),
                args.persist,
                args.ack_timeout,
            )
            if output_result is False:
                return 1

            rate_result = send_command(
                ser,
                f"output_rate={args.rate_hz}Hz",
                CLASS_OUTPUT_RATE,
                bytes([RATE_TO_CODE[args.rate_hz]]),
                args.persist,
                args.ack_timeout,
            )
            if rate_result is False:
                return 1
    except serial.SerialException as exc:
        print(f"[imu-config] serial error: {exc}", file=sys.stderr, flush=True)
        return 1

    print("[imu-config] H30 runtime configuration applied", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
