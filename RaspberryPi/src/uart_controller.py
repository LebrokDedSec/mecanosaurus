#!/usr/bin/env python3
"""UART controller for communicating with ESP32."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path

import serial

logger = logging.getLogger(__name__)


class UARTController:
    """Controls ESP32 via UART connection."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        """Initialize UART connection.
        
        Args:
            port: Serial port (e.g., /dev/ttyUSB0, /dev/ttyAMA0 for RPi GPIO)
            baudrate: Communication speed
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: serial.Serial | None = None
        self._connect()

    def _connect(self) -> bool:
        """Establish serial connection."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
            )
            logger.info(f"Connected to ESP32 on {self.port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            return False

    def is_connected(self) -> bool:
        """Check if serial port is open."""
        return self.serial is not None and self.serial.is_open

    def send_command(self, x: float = 0.0, y: float = 0.0, omega: float = 0.0) -> bool:
        """Send drive command to ESP32.
        
        Args:
            x: Lateral velocity (-1.0 to 1.0), positive = right
            y: Forward velocity (-1.0 to 1.0), positive = forward
            omega: Angular velocity (-1.0 to 1.0), positive = CCW
            
        Returns:
            True if sent successfully
        """
        if not self.is_connected():
            logger.error("Not connected to ESP32")
            return False

        # Clamp values to [-1, 1]
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        omega = max(-1.0, min(1.0, omega))

        command = f"DRIVE:{x:.3f},{y:.3f},{omega:.3f}\n"
        try:
            self.serial.write(command.encode("utf-8"))
            logger.debug(f"Sent: {command.strip()}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send command: {e}")
            return False

    def send_stop(self) -> bool:
        """Send STOP command to ESP32."""
        if not self.is_connected():
            logger.error("Not connected to ESP32")
            return False

        command = "STOP\n"
        try:
            self.serial.write(command.encode("utf-8"))
            logger.debug(f"Sent: {command.strip()}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send STOP: {e}")
            return False

    def read_telemetry(self) -> str | None:
        """Read telemetry from ESP32 (non-blocking)."""
        if not self.is_connected():
            return None

        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    return line
        except serial.SerialException as e:
            logger.error(f"Failed to read telemetry: {e}")

        return None

    def close(self) -> None:
        """Close serial connection."""
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            logger.info("Closed UART connection")

    def __del__(self) -> None:
        """Cleanup on deletion."""
        self.close()
