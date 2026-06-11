# ESP32 firmware (PlatformIO)

This folder contains firmware for ESP32-S3-DEVKITC-1-N16R8V.

## What is included

- PlatformIO project config in `platformio.ini`
- Starter Arduino firmware in `src/main.cpp`
- Current firmware: LED blink (500 ms ON, 500 ms OFF)

## Quick start

1. Install VS Code extension: PlatformIO IDE.
2. Open this repository in VS Code.
3. Open folder `ESP32` in PlatformIO project tasks.
4. Build firmware:
   - `pio run`
5. Upload firmware (connected board):
   - `pio run -t upload`
6. Open serial monitor:
   - `pio device monitor`

## Blink behavior

- LED pin: `LED_BUILTIN` (fallback: GPIO 2)
- Interval: 500 ms ON, 500 ms OFF

If your board does not expose a built-in LED, connect an external LED with resistor and set the pin in `src/main.cpp`.
