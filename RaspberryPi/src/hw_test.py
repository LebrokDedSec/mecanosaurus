#!/usr/bin/env python3
"""Quick hardware checks for UART and optional GPIO library."""

from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Hardware test for Raspberry Pi")
    parser.add_argument("--uart", default="/dev/ttyUSB0", help="UART device path")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    return parser.parse_args()


def test_uart(port: str, baud: int) -> None:
    try:
        import serial  # type: ignore

        with serial.Serial(port=port, baudrate=baud, timeout=0.2) as connection:
            print(f"UART OK: opened {connection.port} at {baud} baud")
    except Exception as exc:  # pragma: no cover - diagnostic helper
        print(f"UART ERROR: {exc}")


def test_gpio() -> None:
    try:
        import RPi.GPIO as GPIO  # type: ignore

        GPIO.setmode(GPIO.BCM)
        GPIO.cleanup()
        print("GPIO OK: RPi.GPIO imported and initialized")
    except Exception as exc:  # pragma: no cover - diagnostic helper
        print(f"GPIO WARNING: {exc}")


def main() -> None:
    args = parse_args()
    test_uart(args.uart, args.baud)
    test_gpio()


if __name__ == "__main__":
    main()
