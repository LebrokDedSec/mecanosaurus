#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${ROOT_DIR}"

if [[ ! -d .venv ]]; then
  echo "Missing virtual environment. Run: bash scripts/setup.sh"
  exit 1
fi

source .venv/bin/activate

CMD=(
  python src/apriltag_ble_follow.py
  --camera "${CAMERA_INDEX:-0}"
  --tag-family "${TAG_FAMILY:-36h11}"
  --tag-id "${TAG_ID:-0}"
  --tag-size "${TAG_SIZE_M:-0.12}"
  --calib-file "${CALIB_FILE:-config/camera_calib.npz}"
  --distance-scale "${DISTANCE_SCALE:-1.52}"
  --target-distance "${TARGET_DISTANCE_M:-0.50}"
  --max-wheel-rpm "${MAX_WHEEL_RPM:-220}"
  --wheel-diameter "${WHEEL_DIAMETER_M:-0.10}"
  --wheelbase-length "${WHEELBASE_LENGTH_M:-0.260}"
  --track-width "${TRACK_WIDTH_M:-0.486}"
  --kp-forward "${KP_FORWARD:-1.20}"
  --kp-strafe "${KP_STRAFE:-1.60}"
  --kp-turn "${KP_TURN:-2.00}"
  --max-x "${MAX_X_NORM:-0.35}"
  --max-y "${MAX_Y_NORM:-0.40}"
  --max-omega "${MAX_OMEGA_NORM:-0.35}"
  --command-timeout "${COMMAND_TIMEOUT_S:-0.40}"
  --loop-hz "${LOOP_HZ:-20}"
  --ble-ready-timeout "${BLE_READY_TIMEOUT_S:-8.0}"
  --ble-telemetry-uuid "${BLE_TELEMETRY_UUID:-12345678-1234-1234-1234-1234567890ad}"
)

if [[ -n "${BLE_ADDRESS:-}" ]]; then
  CMD+=(--ble-address "${BLE_ADDRESS}")
else
  CMD+=(--ble-name "${BLE_NAME:-ESP32-S3-DEVKITC-1-N16R8V}")
fi

if [[ "${SHOW_PREVIEW:-0}" == "1" ]]; then
  CMD+=(--show-preview)
fi

echo "Starting AprilTag BLE follower (headless by default)..."
exec "${CMD[@]}"
