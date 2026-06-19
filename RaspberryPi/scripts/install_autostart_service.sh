#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="mecanosaurus-apriltag.service"
ENV_FILE="/etc/default/mecanosaurus-apriltag"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

RUN_USER="${SUDO_USER:-$USER}"
RUN_GROUP="$(id -gn "${RUN_USER}")"

SERVICE_TMP="$(mktemp)"
trap 'rm -f "${SERVICE_TMP}"' EXIT

cat >"${SERVICE_TMP}" <<EOF
[Unit]
Description=Mecanosaurus AprilTag BLE follow
After=network-online.target bluetooth.target
Wants=network-online.target bluetooth.target

[Service]
Type=simple
User=${RUN_USER}
Group=${RUN_GROUP}
WorkingDirectory=${ROOT_DIR}
EnvironmentFile=-${ENV_FILE}
ExecStart=${ROOT_DIR}/scripts/start_apriltag_ble_follow.sh
Restart=always
RestartSec=2
StartLimitIntervalSec=0

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo install -m 755 "${ROOT_DIR}/scripts/start_apriltag_ble_follow.sh" "${ROOT_DIR}/scripts/start_apriltag_ble_follow.sh"
sudo install -m 644 "${SERVICE_TMP}" "/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -f "${ENV_FILE}" ]]; then
  sudo tee "${ENV_FILE}" >/dev/null <<'EOF'
# Mecanosaurus AprilTag autostart settings
# Fill BLE_ADDRESS for fastest and most reliable reconnect.
BLE_ADDRESS=
BLE_NAME=ESP32-S3-DEVKITC-1-N16R8V
BLE_TELEMETRY_UUID=12345678-1234-1234-1234-1234567890ad
BLE_READY_TIMEOUT_S=8.0

# Camera/Tag
CAMERA_INDEX=0
TAG_FAMILY=36h11
TAG_ID=0
TAG_SIZE_M=0.12
CALIB_FILE=config/camera_calib.npz
DISTANCE_SCALE=1.52
TARGET_DISTANCE_M=0.50

# Robot dynamics assumptions
MAX_WHEEL_RPM=220
WHEEL_DIAMETER_M=0.10
WHEELBASE_LENGTH_M=0.260
TRACK_WIDTH_M=0.486

# Controller tuning and normalized limits
KP_FORWARD=1.20
KP_STRAFE=1.60
KP_TURN=2.00
MAX_X_NORM=0.35
MAX_Y_NORM=0.40
MAX_OMEGA_NORM=0.35
COMMAND_TIMEOUT_S=0.40
LOOP_HZ=20

# Keep disabled for onboard headless mode
SHOW_PREVIEW=0
EOF
fi

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"

echo "Installed and enabled ${SERVICE_NAME}."
echo "Edit settings: ${ENV_FILE}"
echo "Start now: sudo systemctl start ${SERVICE_NAME}"
echo "Check logs: sudo journalctl -u ${SERVICE_NAME} -f"
