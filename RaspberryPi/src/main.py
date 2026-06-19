#!/usr/bin/env python3
"""Simple Raspberry Pi entrypoint for Mecanosaurus."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mecanosaurus Raspberry Pi runner")
    parser.add_argument(
        "--config",
        default="config/settings.example.json",
        help="Path to JSON config file",
    )
    return parser.parse_args()


def load_config(config_path: str) -> dict:
    path = Path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def main() -> None:
    args = parse_args()
    config = load_config(args.config)

    robot_name = config.get("robot_name", "mecanosaurus")
    control_hz = config.get("control_hz", 20)
    uart_port = config.get("uart_port", "/dev/ttyUSB0")

    print("Mecanosaurus Raspberry Pi module")
    print(f"Robot: {robot_name}")
    print(f"Control loop: {control_hz} Hz")
    print(f"UART port: {uart_port}")


if __name__ == "__main__":
    main()
