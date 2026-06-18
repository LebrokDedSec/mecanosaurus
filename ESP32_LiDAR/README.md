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

## Windows 2D viewer (top view)

Simple desktop viewer for the serial LiDAR stream is available in:
- `lidar_viewer_windows.py`

The script expects serial lines in this format:
- `PT,angle_deg,distance_mm,confidence,speed`

### Install dependencies

```powershell
cd ESP32_LiDAR
python -m pip install -r viewer_requirements.txt
```

### Run

```powershell
cd ESP32_LiDAR
python lidar_viewer_windows.py --port COM17 --baud 115200
```

Useful options:
- `--range 4000` view range in mm
- `--min-confidence 60` filter weak points
- `--max-age 1.2` drop stale points
