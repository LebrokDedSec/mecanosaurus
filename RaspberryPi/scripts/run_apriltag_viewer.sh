#!/usr/bin/env bash
set -euo pipefail

APP_DIR="/home/jan/Mecanozaurus/RaspberryPi"
PYTHON_BIN="$APP_DIR/.venv/bin/python"
SCRIPT="$APP_DIR/src/camera_viewer.py"
CALIB="$APP_DIR/config/camera_calib.npz"

CAMERAS=("$@")
if [[ ${#CAMERAS[@]} -eq 0 ]]; then
  CAMERAS=(0 1 2 3 4 5)
fi

for cam in "${CAMERAS[@]}"; do
  if "$PYTHON_BIN" "$SCRIPT" \
    --camera "$cam" \
    --tag-family 36h11 \
    --tag-id 0 \
    --calib-file "$CALIB" \
    --distance-scale 1.5; then
    exit 0
  fi
done

echo "Nie udalo sie uruchomic podgladu: brak dostepnej kamery." >&2
exit 1
