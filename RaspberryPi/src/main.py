#!/usr/bin/env python3
"""Simple Raspberry Pi entrypoint for Mecanosaurus."""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path

from uart_controller import UARTController

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mecanosaurus Raspberry Pi runner")
    parser.add_argument(
        "--config",
        default="config/settings.example.json",
        help="Path to JSON config file",
    )
    parser.add_argument(
        "--forward",
        type=float,
        default=0.5,
        help="Forward speed (0.0-1.0)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Duration to run in seconds",
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
    uart_baud = config.get("uart_baud", 115200)

    logger.info("=" * 60)
    logger.info("Mecanosaurus Raspberry Pi module")
    logger.info(f"Robot: {robot_name}")
    logger.info(f"Control loop: {control_hz} Hz")
    logger.info(f"UART port: {uart_port} @ {uart_baud} baud")
    logger.info("=" * 60)

    # Connect to ESP32
    controller = UARTController(port=uart_port, baudrate=uart_baud)
    if not controller.is_connected():
        logger.error("Failed to connect to ESP32!")
        return

    try:
        # Give ESP32 time to initialize
        time.sleep(0.5)

        logger.info(f"Sending forward command: y={args.forward} for {args.duration}s")
        controller.send_command(x=0.0, y=args.forward, omega=0.0)

        # Run for specified duration
        start_time = time.time()
        while time.time() - start_time < args.duration:
            # Read any telemetry from ESP32
            telemetry = controller.read_telemetry()
            if telemetry:
                logger.info(f"ESP32: {telemetry}")

            time.sleep(0.05)

        # Stop
        logger.info("Stopping")
        controller.send_stop()
        time.sleep(0.2)

        logger.info("Done!")

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        controller.send_stop()
    finally:
        controller.close()


if __name__ == "__main__":
    main()

