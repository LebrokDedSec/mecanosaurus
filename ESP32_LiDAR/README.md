# ESP32 LiDAR firmware (PlatformIO)

This folder contains firmware for the second ESP32 board responsible for LiDAR handling.

## What is included

- PlatformIO project config in `platformio.ini`
- Starter Arduino firmware in `src/main.cpp`
- Placeholder serial loop for LiDAR integration

## Quick start

1. Install VS Code extension: PlatformIO IDE.
2. Open this repository in VS Code.
3. Open folder `ESP32_LiDAR` in PlatformIO project tasks.
4. Build firmware:
   - `pio run`
5. Upload firmware (connected board):
   - `pio run -t upload`
6. Open serial monitor:
   - `pio device monitor`

## Next step

Replace the placeholder logic in `src/main.cpp` with the specific LiDAR driver and pin mapping for your sensor.
