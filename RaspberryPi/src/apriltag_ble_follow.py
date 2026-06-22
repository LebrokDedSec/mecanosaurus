#!/usr/bin/env python3
"""Follow AprilTag and send DRIVE commands to ESP32 over BLE.

This script keeps the robot moving toward a selected AprilTag by publishing
BLE control payloads in the existing firmware format:
- DRIVE:x,y,omega
- STOP

Coordinate/sign convention expected by ESP32 firmware:
- +x: strafe right
- +y: forward
- +omega: CCW (left turn)
"""

from __future__ import annotations

import argparse
import asyncio
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - runtime dependency check
    BleakClient = None
    BleakScanner = None


APRILTAG_FAMILIES = {
    "16h5": "DICT_APRILTAG_16h5",
    "25h9": "DICT_APRILTAG_25h9",
    "36h10": "DICT_APRILTAG_36h10",
    "36h11": "DICT_APRILTAG_36h11",
}


@dataclass
class Config:
    camera: int
    width: int
    height: int
    tag_family: str
    tag_id: int
    tag_size: float
    calib_file: str
    fx: float
    fy: float
    cx: float
    cy: float
    loop_hz: float
    command_timeout: float
    target_distance: float
    max_wheel_rpm: float
    wheel_diameter_m: float
    wheelbase_length_m: float
    track_width_m: float
    kp_forward: float
    kp_strafe: float
    kp_turn: float
    max_x: float
    max_y: float
    max_omega: float
    deadband_x: float
    deadband_y: float
    deadband_omega: float
    smooth_alpha: float
    distance_scale: float
    search_enabled: bool
    search_omega: float
    search_phase_sec: float
    center_only: bool
    ble_name: str
    ble_address: str
    ble_service_uuid: str
    ble_control_uuid: str
    ble_telemetry_uuid: str
    ble_ready_timeout: float
    show_preview: bool


def compute_robot_limits(
    max_wheel_rpm: float,
    wheel_diameter_m: float,
    wheelbase_length_m: float,
    track_width_m: float,
) -> tuple[float, float, float]:
    wheel_radius = max(wheel_diameter_m * 0.5, 1e-6)
    wheelbase_radius = max((wheelbase_length_m * 0.5) + (track_width_m * 0.5), 1e-6)
    wheel_ang_radps = (max_wheel_rpm * 2.0 * math.pi) / 60.0

    max_vx = wheel_ang_radps * wheel_radius
    max_vy = wheel_ang_radps * wheel_radius
    max_omega = max_vx / wheelbase_radius
    return max_vx, max_vy, max_omega


def parse_args() -> Config:
    parser = argparse.ArgumentParser(description="AprilTag BLE follower for ESP32 drive board")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--tag-family", default="36h11", choices=sorted(APRILTAG_FAMILIES.keys()))
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.12, help="Tag size in meters")

    parser.add_argument("--calib-file", default="config/camera_calib.npz")
    parser.add_argument("--fx", type=float, default=0.0)
    parser.add_argument("--fy", type=float, default=0.0)
    parser.add_argument("--cx", type=float, default=0.0)
    parser.add_argument("--cy", type=float, default=0.0)

    parser.add_argument("--loop-hz", type=float, default=20.0)
    parser.add_argument("--command-timeout", type=float, default=0.40)
    parser.add_argument("--target-distance", type=float, default=0.50)
    parser.add_argument("--max-wheel-rpm", type=float, default=220.0)
    parser.add_argument("--wheel-diameter", type=float, default=0.10, help="Wheel diameter in meters")
    parser.add_argument("--wheelbase-length", type=float, default=0.260, help="Front-back axle spacing in meters")
    parser.add_argument("--track-width", type=float, default=0.486, help="Left-right wheel spacing in meters")

    parser.add_argument("--kp-forward", type=float, default=1.2)
    parser.add_argument("--kp-strafe", type=float, default=1.6)
    parser.add_argument("--kp-turn", type=float, default=2.0)

    parser.add_argument("--max-x", type=float, default=0.60)
    parser.add_argument("--max-y", type=float, default=0.65)
    parser.add_argument("--max-omega", type=float, default=0.70)

    parser.add_argument("--deadband-x", type=float, default=0.04)
    parser.add_argument("--deadband-y", type=float, default=0.04)
    parser.add_argument("--deadband-omega", type=float, default=0.05)

    parser.add_argument("--smooth-alpha", type=float, default=0.35)
    parser.add_argument("--distance-scale", type=float, default=1.52)
    parser.add_argument(
        "--search-enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When tag is not visible, sweep left/right with omega to search for it",
    )
    parser.add_argument(
        "--search-omega",
        type=float,
        default=0.40,
        help="Normalized omega used while sweeping for a tag",
    )
    parser.add_argument(
        "--search-phase-sec",
        type=float,
        default=1.2,
        help="Duration of one sweep direction before switching sign",
    )
    parser.add_argument(
        "--center-only",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When tag is visible, only center it in camera axis (no forward/strafe)",
    )

    parser.add_argument("--ble-name", default="ESP32-S3-DEVKITC-1-N16R8V")
    parser.add_argument("--ble-address", default="", help="Optional BLE MAC address to skip scan")
    parser.add_argument("--ble-service-uuid", default="12345678-1234-1234-1234-1234567890ab")
    parser.add_argument("--ble-control-uuid", default="12345678-1234-1234-1234-1234567890ac")
    parser.add_argument("--ble-telemetry-uuid", default="12345678-1234-1234-1234-1234567890ad")
    parser.add_argument(
        "--ble-ready-timeout",
        type=float,
        default=8.0,
        help="Seconds to wait for ESP ready/telemetry frame after BLE connect",
    )

    parser.add_argument("--show-preview", action="store_true", help="Show camera preview with debug text")

    args = parser.parse_args()
    return Config(
        camera=args.camera,
        width=args.width,
        height=args.height,
        tag_family=args.tag_family,
        tag_id=args.tag_id,
        tag_size=args.tag_size,
        calib_file=args.calib_file,
        fx=args.fx,
        fy=args.fy,
        cx=args.cx,
        cy=args.cy,
        loop_hz=max(args.loop_hz, 1.0),
        command_timeout=max(args.command_timeout, 0.10),
        target_distance=max(args.target_distance, 0.05),
        max_wheel_rpm=max(args.max_wheel_rpm, 1.0),
        wheel_diameter_m=max(args.wheel_diameter, 0.01),
        wheelbase_length_m=max(args.wheelbase_length, 0.01),
        track_width_m=max(args.track_width, 0.01),
        kp_forward=max(args.kp_forward, 0.0),
        kp_strafe=max(args.kp_strafe, 0.0),
        kp_turn=max(args.kp_turn, 0.0),
        max_x=max(args.max_x, 0.0),
        max_y=max(args.max_y, 0.0),
        max_omega=max(args.max_omega, 0.0),
        deadband_x=max(args.deadband_x, 0.0),
        deadband_y=max(args.deadband_y, 0.0),
        deadband_omega=max(args.deadband_omega, 0.0),
        smooth_alpha=float(np.clip(args.smooth_alpha, 0.0, 1.0)),
        distance_scale=max(args.distance_scale, 0.01),
        search_enabled=bool(args.search_enabled),
        search_omega=float(np.clip(args.search_omega, 0.0, 1.0)),
        search_phase_sec=max(args.search_phase_sec, 0.10),
        center_only=bool(args.center_only),
        ble_name=args.ble_name,
        ble_address=args.ble_address,
        ble_service_uuid=args.ble_service_uuid,
        ble_control_uuid=args.ble_control_uuid,
        ble_telemetry_uuid=args.ble_telemetry_uuid,
        ble_ready_timeout=max(args.ble_ready_timeout, 0.5),
        show_preview=args.show_preview,
    )


def load_calibration(cfg: Config):
    if cfg.calib_file:
        path = Path(cfg.calib_file)
        if not path.exists():
            raise FileNotFoundError(f"Calibration file not found: {path}")
        data = np.load(str(path))
        camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
        dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
        return camera_matrix, dist_coeffs

    fx = cfg.fx if cfg.fx > 0 else float(cfg.width)
    fy = cfg.fy if cfg.fy > 0 else float(cfg.height)
    cx = cfg.cx if cfg.cx > 0 else float(cfg.width) * 0.5
    cy = cfg.cy if cfg.cy > 0 else float(cfg.height) * 0.5
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist_coeffs = np.zeros((4, 1), dtype=np.float64)
    return camera_matrix, dist_coeffs


def clamp_unit(v: float) -> float:
    return float(np.clip(v, -1.0, 1.0))


def apply_deadband(v: float, deadband: float) -> float:
    return 0.0 if abs(v) < deadband else v


def build_dictionary(tag_family: str):
    dict_name = APRILTAG_FAMILIES[tag_family]
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))


def select_stable_pose(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    previous_pose: Optional[tuple[np.ndarray, np.ndarray]],
):
    result = cv2.solvePnPGeneric(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if len(result) < 3 or not bool(result[0]):
        return None

    rvecs = result[1]
    tvecs = result[2]
    reproj = result[3] if len(result) > 3 else None

    prev_rvec = previous_pose[0] if previous_pose is not None else None
    prev_tvec = previous_pose[1] if previous_pose is not None else None

    best_idx = None
    best_score = None

    for idx, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        score = 0.0
        z_val = float(tvec.reshape(-1)[2])
        if z_val <= 0.0:
            score += 1e6
        else:
            score += 10.0 / max(z_val, 1e-6)

        if reproj is not None:
            err = float(np.array(reproj[idx]).reshape(-1)[0])
            score += err * 100.0

        if prev_rvec is not None and prev_tvec is not None:
            score += float(np.linalg.norm(rvec - prev_rvec)) * 25.0
            score += float(np.linalg.norm(tvec - prev_tvec)) * 250.0
        else:
            score += 2.0 / max(z_val, 1e-6)

        if best_score is None or score < best_score:
            best_score = score
            best_idx = idx

    if best_idx is None:
        return None
    return rvecs[best_idx], tvecs[best_idx]


def build_drive_command(
    z_m: float,
    x_m: float,
    cfg: Config,
) -> tuple[float, float, float]:
    z_corr = z_m * cfg.distance_scale
    x_corr = x_m * cfg.distance_scale

    distance_error = z_corr - cfg.target_distance
    y_cmd = 0.0 if cfg.center_only else (cfg.kp_forward * max(distance_error, 0.0))
    x_cmd = 0.0 if cfg.center_only else (cfg.kp_strafe * x_corr)
    bearing_rad = math.atan2(x_corr, max(z_corr, 1e-6))
    omega_cmd = -cfg.kp_turn * bearing_rad

    x_cmd = float(np.clip(x_cmd, -cfg.max_x, cfg.max_x))
    y_cmd = float(np.clip(y_cmd, -cfg.max_y, cfg.max_y))
    omega_cmd = float(np.clip(omega_cmd, -cfg.max_omega, cfg.max_omega))

    x_cmd = clamp_unit(apply_deadband(x_cmd, cfg.deadband_x))
    y_cmd = clamp_unit(apply_deadband(y_cmd, cfg.deadband_y))
    omega_cmd = clamp_unit(apply_deadband(omega_cmd, cfg.deadband_omega))
    return x_cmd, y_cmd, omega_cmd


def build_search_command(now_s: float, cfg: Config) -> tuple[float, float, float]:
    if not cfg.search_enabled:
        return 0.0, 0.0, 0.0

    phase = int(now_s / cfg.search_phase_sec)
    sign = -1.0 if (phase % 2 == 0) else 1.0
    omega_cmd = clamp_unit(apply_deadband(sign * cfg.search_omega, cfg.deadband_omega))
    return 0.0, 0.0, omega_cmd


def format_drive_payload(x: float, y: float, omega: float) -> str:
    return f"DRIVE:{x:.3f},{y:.3f},{omega:.3f}"


async def discover_device(cfg: Config) -> str:
    if cfg.ble_address:
        return cfg.ble_address

    if BleakScanner is None:
        raise RuntimeError("Missing dependency bleak. Install with: pip install bleak")

    print(f"Scanning BLE for device name: {cfg.ble_name}")
    devices = await BleakScanner.discover(timeout=6.0)
    for dev in devices:
        if dev.name == cfg.ble_name:
            print(f"Found BLE device: {dev.name} [{dev.address}]")
            return dev.address

    raise RuntimeError(f"BLE device not found: {cfg.ble_name}")


async def wait_for_esp_ready(client: BleakClient, cfg: Config) -> None:
    telemetry_event = asyncio.Event()
    telemetry_payload = {"value": ""}

    def _on_telemetry(_: int, data: bytearray) -> None:
        text = bytes(data).decode("utf-8", errors="ignore").strip()
        telemetry_payload["value"] = text
        if text:
            telemetry_event.set()

    await client.start_notify(cfg.ble_telemetry_uuid, _on_telemetry)
    try:
        deadline = time.time() + cfg.ble_ready_timeout
        while time.time() < deadline:
            raw = await client.read_gatt_char(cfg.ble_control_uuid)
            control_text = bytes(raw).decode("utf-8", errors="ignore").strip().lower()
            if control_text == "ready":
                print("ESP ready confirmed via control characteristic")
                return

            remain = max(deadline - time.time(), 0.0)
            if remain <= 0.0:
                break

            try:
                await asyncio.wait_for(telemetry_event.wait(), timeout=min(0.8, remain))
                print(f"ESP ready confirmed via telemetry: {telemetry_payload['value']}")
                return
            except asyncio.TimeoutError:
                continue

        raise RuntimeError("ESP ready handshake timeout after BLE connect")
    finally:
        await client.stop_notify(cfg.ble_telemetry_uuid)


async def run_follow(cfg: Config) -> None:
    if BleakClient is None:
        raise RuntimeError("Missing dependency bleak. Install with: pip install bleak")

    camera_matrix, dist_coeffs = load_calibration(cfg)
    max_vx_mps, max_vy_mps, max_omega_radps = compute_robot_limits(
        cfg.max_wheel_rpm,
        cfg.wheel_diameter_m,
        cfg.wheelbase_length_m,
        cfg.track_width_m,
    )

    tag_obj_pts = np.array(
        [
            [-cfg.tag_size / 2.0, -cfg.tag_size / 2.0, 0.0],
            [cfg.tag_size / 2.0, -cfg.tag_size / 2.0, 0.0],
            [cfg.tag_size / 2.0, cfg.tag_size / 2.0, 0.0],
            [-cfg.tag_size / 2.0, cfg.tag_size / 2.0, 0.0],
        ],
        dtype=np.float32,
    )

    dictionary = build_dictionary(cfg.tag_family)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        detector_params = cv2.aruco.DetectorParameters_create()
    else:
        detector_params = cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    address = await discover_device(cfg)

    cap = cv2.VideoCapture(cfg.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {cfg.camera}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)

    pose_state: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    cmd_state: Optional[tuple[float, float, float]] = None
    last_seen = 0.0
    has_seen_tag = False
    last_print = 0.0

    print("Connecting BLE client...")
    async with BleakClient(address) as client:
        print("BLE connected")
        print("Waiting for ESP ready handshake...")
        await wait_for_esp_ready(client, cfg)
        print("ESP handshake OK -> starting AprilTag follow")
        print(
            "Kinematic limits: "
            f"vx_max={max_vx_mps:.3f} m/s, "
            f"vy_max={max_vy_mps:.3f} m/s, "
            f"omega_max={max_omega_radps:.3f} rad/s"
        )
        print(
            "Command caps (normalized): "
            f"x={cfg.max_x:.2f}, y={cfg.max_y:.2f}, omega={cfg.max_omega:.2f}"
        )
        print(
            "Effective caps: "
            f"vx={cfg.max_x * max_vx_mps:.3f} m/s, "
            f"vy={cfg.max_y * max_vy_mps:.3f} m/s, "
            f"omega={cfg.max_omega * max_omega_radps:.3f} rad/s"
        )
        mode_text = "center-only" if cfg.center_only else "follow+center"
        print(f"Waiting for tag id={cfg.tag_id} | mode={mode_text}")

        period = 1.0 / cfg.loop_hz
        try:
            while True:
                ok, frame = cap.read()
                now = time.time()

                if not ok:
                    await client.write_gatt_char(cfg.ble_control_uuid, b"STOP", response=False)
                    await asyncio.sleep(period)
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners_list, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector_params)

                best_tvec = None
                best_id = None
                best_corners = None
                if ids is not None:
                    for i, marker_id in enumerate(ids.flatten()):
                        if cfg.tag_id >= 0 and int(marker_id) != cfg.tag_id:
                            continue

                        corners_f = np.array(corners_list[i], dtype=np.float32).reshape(4, 2)
                        pose = select_stable_pose(
                            tag_obj_pts,
                            corners_f,
                            camera_matrix,
                            dist_coeffs,
                            pose_state.get(int(marker_id)),
                        )
                        if pose is None:
                            continue

                        rvec, tvec = pose
                        if int(marker_id) in pose_state:
                            prev_rvec, prev_tvec = pose_state[int(marker_id)]
                            rvec = cfg.smooth_alpha * prev_rvec + (1.0 - cfg.smooth_alpha) * rvec
                            tvec = cfg.smooth_alpha * prev_tvec + (1.0 - cfg.smooth_alpha) * tvec
                        pose_state[int(marker_id)] = (rvec, tvec)

                        z_m = float(tvec.reshape(-1)[2])
                        if z_m <= 0.0:
                            continue

                        if best_tvec is None or z_m < float(best_tvec.reshape(-1)[2]):
                            best_tvec = tvec
                            best_id = int(marker_id)
                            best_corners = corners_f

                if best_tvec is not None:
                    x_m = float(best_tvec.reshape(-1)[0])
                    z_m = float(best_tvec.reshape(-1)[2])
                    x_cmd, y_cmd, omega_cmd = build_drive_command(z_m, x_m, cfg)

                    if cmd_state is not None:
                        px, py, po = cmd_state
                        x_cmd = cfg.smooth_alpha * px + (1.0 - cfg.smooth_alpha) * x_cmd
                        y_cmd = cfg.smooth_alpha * py + (1.0 - cfg.smooth_alpha) * y_cmd
                        omega_cmd = cfg.smooth_alpha * po + (1.0 - cfg.smooth_alpha) * omega_cmd
                    cmd_state = (x_cmd, y_cmd, omega_cmd)

                    payload = format_drive_payload(x_cmd, y_cmd, omega_cmd)
                    await client.write_gatt_char(cfg.ble_control_uuid, payload.encode("utf-8"), response=False)
                    last_seen = now
                    has_seen_tag = True

                    if now - last_print > 0.2:
                        last_print = now
                        z_corr = z_m * cfg.distance_scale
                        bearing = math.degrees(math.atan2(x_m, max(z_m, 1e-6)))
                        print(
                            f"id={best_id} z={z_corr:.2f}m x={x_m * cfg.distance_scale:+.2f}m "
                            f"bearing={bearing:+.1f}deg cmd={payload}"
                        )

                    if cfg.show_preview and best_corners is not None:
                        corners_i = best_corners.astype(np.int32)
                        cv2.polylines(frame, [corners_i.reshape(-1, 1, 2)], True, (0, 255, 255), 2)
                        cv2.putText(
                            frame,
                            payload,
                            tuple(corners_i[0].tolist()),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
                else:
                    if (not has_seen_tag) or (now - last_seen > cfg.command_timeout):
                        sx, sy, so = build_search_command(now, cfg)
                        if sx == 0.0 and sy == 0.0 and so == 0.0:
                            await client.write_gatt_char(cfg.ble_control_uuid, b"STOP", response=False)
                        else:
                            payload = format_drive_payload(sx, sy, so)
                            await client.write_gatt_char(cfg.ble_control_uuid, payload.encode("utf-8"), response=False)
                    if now - last_print > 0.5:
                        last_print = now
                        if has_seen_tag:
                            print("tag lost -> searching sweep")
                        else:
                            print("waiting for tag -> searching sweep")

                if cfg.show_preview:
                    cv2.imshow("AprilTag BLE Follow", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break

                await asyncio.sleep(period)
        finally:
            await client.write_gatt_char(cfg.ble_control_uuid, b"STOP", response=False)
            cap.release()
            if cfg.show_preview:
                cv2.destroyAllWindows()
            print("Follower stopped, STOP sent")


def main() -> None:
    cfg = parse_args()
    asyncio.run(run_follow(cfg))


if __name__ == "__main__":
    main()
