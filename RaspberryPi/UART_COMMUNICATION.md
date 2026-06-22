# ESP32 ↔ RPi UART Communication Guide

## Hardware Setup

### Wiring (3-wire UART connection)

**ESP32-S3 pins:**
- Pin 19 (UART1 TX) → RPi RX
- Pin 20 (UART1 RX) → RPi TX  
- GND → RPi GND

**RPi UART pins** (GPIO):
- GPIO 14 (TXD) → ESP32 Pin 20
- GPIO 15 (RXD) → ESP32 Pin 19
- GND → ESP32 GND

**Or via USB adapter:**
- If using USB-to-UART adapter, connect adapter to USB and configure port accordingly

## ESP32 Firmware

### Build and Upload

```bash
cd ESP32
pio run -t upload
```

The firmware now listens on UART1 for commands from RPi.

### Test (optional - from computer)

```bash
# Monitor serial output
pio device monitor

# In another terminal, send test command via Python
python3 << 'EOF'
import serial
import time

with serial.Serial('/dev/ttyUSB0', 115200) as s:
    time.sleep(0.5)
    s.write(b'DRIVE:0.0,0.5,0.0\n')  # Forward at 50% speed
    time.sleep(2)
    s.write(b'STOP\n')
EOF
```

## Raspberry Pi

### 1. Identify UART Port

Check which port the ESP32 is connected to:

```bash
# List serial devices
ls -la /dev/tty*

# If connected via USB:
# /dev/ttyUSB0 (adapter), /dev/ttyUSB1, etc.

# If connected to GPIO UART:
# /dev/ttyAMA0 (hardware UART) or /dev/ttyS0
```

### 2. Update Configuration

Edit [config/settings.example.json](config/settings.example.json):

```json
{
  "robot_name": "mecanosaurus-rpi",
  "control_hz": 20,
  "uart_port": "/dev/ttyUSB0",
  "uart_baud": 115200
}
```

Change `uart_port` to match your connection:
- USB adapter: `/dev/ttyUSB0` 
- GPIO UART: `/dev/ttyAMA0`

### 3. Run Motor Test

Make wheels go forward for 5 seconds at 50% speed:

```bash
cd RaspberryPi
python3 src/main.py --config config/settings.example.json --forward 0.5 --duration 5
```

Options:
- `--forward`: Speed 0.0-1.0 (default: 0.5)
- `--duration`: How long in seconds (default: 5.0)

### 4. Examples

**Go forward full speed for 10 seconds:**
```bash
python3 src/main.py --forward 1.0 --duration 10
```

**Go forward slow (20% speed) for 3 seconds:**
```bash
python3 src/main.py --forward 0.2 --duration 3
```

## Command Protocol

### Format: `DRIVE:x,y,omega`

- `x`: Lateral (-1.0 left to 1.0 right)
- `y`: Forward (-1.0 back to 1.0 forward)  
- `omega`: Rotation (-1.0 CCW to 1.0 CW)

### Examples

```
DRIVE:0.0,1.0,0.0      # Forward only
DRIVE:1.0,0.0,0.0      # Strafe right only
DRIVE:0.0,0.0,1.0      # Rotate CCW only
DRIVE:0.5,0.5,0.0      # Forward + right strafe
STOP                    # Emergency stop
```

## Troubleshooting

### ESP32 not responding

1. Check USB connection or UART wiring
2. Verify port: `ls -la /dev/tty*`
3. Check permissions: `sudo usermod -a -G dialout $USER`
4. Monitor ESP32 serial: `pio device monitor`

### Permission denied on `/dev/ttyUSB0`

```bash
# Add user to dialout group
sudo usermod -a -G dialout $(whoami)

# Re-login or:
newgrp dialout

# Then test:
python3 src/main.py
```

### Check UART communication

```bash
# Install screen/minicom
sudo apt install screen

# Monitor incoming data
screen /dev/ttyUSB0 115200

# Exit: Ctrl+A then Ctrl+D
```

## Debug Mode

Add logging to see what's happening:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Then run main.py
```

## Next Steps

- Integrate with ROS2 for autonomous control
- Add encoder feedback for accurate odometry  
- Implement AprilTag following (see [RaspberryPi/README.md](../README.md))
- Add obstacle avoidance
