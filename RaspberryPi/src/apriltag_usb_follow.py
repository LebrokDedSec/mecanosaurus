#!/usr/bin/env python3
"""Follow AprilTag and send DRIVE commands to ESP32 over USB serial.

Payload format sent to ESP32 (line-based):
- DRIVE:x,y,omega
- STOP
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import serial

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
    turn_only_angle_deg: float
    serial_port: str
    serial_baud: int
    show_preview: bool


def compute_robot_limits(max_wheel_rpm: float, wheel_diameter_m: float, wheelbase_length_m: float, track_width_m: float):
    wheel_radius = max(wheel_diameter_m * 0.5, 1e-6)
    wheelbase_radius = max((wheelbase_length_m * 0.5) + (track_width_m * 0.5), 1e-6)
    wheel_ang_radps = (max_wheel_rpm * 2.0 * math.pi) / 60.0
    max_vx = wheel_ang_radps * wheel_radius
    max_vy = wheel_ang_radps * wheel_radius
    max_omega = max_vx / wheelbase_radius
    return max_vx, max_vy, max_omega


def parse_args() -> Config:
    parser = argparse.ArgumentParser(description="AprilTag USB follower for ESP32 drive board")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--tag-family", default="36h11", choices=sorted(APRILTAG_FAMILIES.keys()))
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.12)
    parser.add_argument("--calib-file", default="config/camera_calib.npz")
    parser.add_argument("--fx", type=float, default=0.0)
    parser.add_argument("--fy", type=float, default=0.0)
    parser.add_argument("--cx", type=float, default=0.0)
    parser.add_argument("--cy", type=float, default=0.0)
    parser.add_argument("--loop-hz", type=float, default=20.0)
    parser.add_argument("--command-timeout", type=float, default=0.40)
    parser.add_argument("--target-distance", type=float, default=0.50)
    parser.add_argument("--max-wheel-rpm", type=float, default=220.0)
    parser.add_argument("--wheel-diameter", type=float, default=0.10)
    parser.add_argument("--wheelbase-length", type=float, default=0.260)
    parser.add_argument("--track-width", type=float, default=0.486)
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
        "--turn-only-angle-deg",
        type=float,
        default=20.0,
        help="If |bearing| exceeds this value, rotate in place without forward motion",
    )
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--serial-baud", type=int, default=115200)
    parser.add_argument("--show-preview", action="store_true")

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
        turn_only_angle_deg=max(args.turn_only_angle_deg, 1.0),
        serial_port=args.serial_port,
        serial_baud=max(args.serial_baud, 1200),
        show_preview=args.show_preview,
    )


def load_calibration(cfg: Config):
    if cfg.calib_file:
        path = Path(cfg.calib_file)
        if not path.exists():
            raise FileNotFoundError(f"Calibration file not found: {path}")
        data = np.load(str(path))
        return np.array(data["camera_matrix"], dtype=np.float64), np.array(data["dist_coeffs"], dtype=np.float64)

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


def select_stable_pose(object_points, image_points, camera_matrix, dist_coeffs, previous_pose):
    result = cv2.solvePnPGeneric(object_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if len(result) < 3 or not bool(result[0]):
        return None

    rvecs, tvecs = result[1], result[2]
    reproj = result[3] if len(result) > 3 else None
    prev_rvec = previous_pose[0] if previous_pose is not None else None
    prev_tvec = previous_pose[1] if previous_pose is not None else None

    best_idx = None
    best_score = None
    for idx, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        z_val = float(tvec.reshape(-1)[2])
        score = 10.0 / max(z_val, 1e-6) if z_val > 0.0 else 1e6
        if reproj is not None:
            score += float(np.array(reproj[idx]).reshape(-1)[0]) * 100.0
        if prev_rvec is not None and prev_tvec is not None:
            score += float(np.linalg.norm(rvec - prev_rvec)) * 25.0
            score += float(np.linalg.norm(tvec - prev_tvec)) * 250.0
        if best_score is None or score < best_score:
            best_score = score
            best_idx = idx

    if best_idx is None:
        return None
    return rvecs[best_idx], tvecs[best_idx]


def build_drive_command(z_m: float, x_m: float, cfg: Config):
    z_corr = z_m * cfg.distance_scale
    x_corr = x_m * cfg.distance_scale

    # Angle between camera forward axis and detected tag (right is positive).
    bearing_rad = math.atan2(x_corr, max(z_corr, 1e-6))
    abs_bearing = abs(bearing_rad)

    # Move toward the tag only when heading error is reasonably small.
    distance_error = z_corr - cfg.target_distance
    turn_only_angle_rad = math.radians(cfg.turn_only_angle_deg)
    if abs_bearing >= turn_only_angle_rad:
        y_cmd = 0.0
    else:
        heading_scale = 1.0 - (abs_bearing / turn_only_angle_rad)
        y_cmd = cfg.kp_forward * max(distance_error, 0.0) * heading_scale

    # Lateral strafe is disabled: steering is based on angular deviation.
    x_cmd = 0.0
    omega_cmd = -cfg.kp_turn * bearing_rad

    x_cmd = float(np.clip(x_cmd, -cfg.max_x, cfg.max_x))
    y_cmd = float(np.clip(y_cmd, -cfg.max_y, cfg.max_y))
    omega_cmd = float(np.clip(omega_cmd, -cfg.max_omega, cfg.max_omega))

    x_cmd = clamp_unit(apply_deadband(x_cmd, cfg.deadband_x))
    y_cmd = clamp_unit(apply_deadband(y_cmd, cfg.deadband_y))
    omega_cmd = clamp_unit(apply_deadband(omega_cmd, cfg.deadband_omega))
    return x_cmd, y_cmd, omega_cmd


def send_payload(ser: serial.Serial, payload: str) -> None:
    ser.write((payload + "\n").encode("utf-8"))


def open_camera_with_fallback(preferred_index: int, width: int, height: int):
    candidates = [preferred_index] + [i for i in range(0, 10) if i != preferred_index]
    for index in candidates:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, _ = cap.read()
        if ok:
            return cap, index

        cap.release()

    return None, None


def main() -> None:
    cfg = parse_args()
    camera_matrix, dist_coeffs = load_calibration(cfg)
    max_vx_mps, max_vy_mps, max_omega_radps = compute_robot_limits(
        cfg.max_wheel_rpm, cfg.wheel_diameter_m, cfg.wheelbase_length_m, cfg.track_width_m
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
    detector_params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") else cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    ser = serial.Serial(cfg.serial_port, cfg.serial_baud, timeout=0.05, write_timeout=0.2)
    cap, camera_index = open_camera_with_fallback(cfg.camera, cfg.width, cfg.height)
    if cap is None:
        ser.close()
        raise RuntimeError(f"Could not open camera index {cfg.camera}")

    pose_state: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    cmd_state: Optional[tuple[float, float, float]] = None
    last_seen = 0.0
    has_seen_tag = False
    last_print = 0.0
    period = 1.0 / cfg.loop_hz

    print(f"USB serial connected: {cfg.serial_port} @ {cfg.serial_baud}")
    print(f"Camera opened on index: {camera_index}")
    print(
        f"Kinematic limits: vx_max={max_vx_mps:.3f} m/s, vy_max={max_vy_mps:.3f} m/s, omega_max={max_omega_radps:.3f} rad/s"
    )
    print(f"Waiting for tag id={cfg.tag_id} at target distance {cfg.target_distance:.2f} m")

    try:
        while True:
            ok, frame = cap.read()
            now = time.time()

            if not ok:
                send_payload(ser, "STOP")
                time.sleep(period)
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
                    pose = select_stable_pose(tag_obj_pts, corners_f, camera_matrix, dist_coeffs, pose_state.get(int(marker_id)))
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

                payload = f"DRIVE:{x_cmd:.3f},{y_cmd:.3f},{omega_cmd:.3f}"
                send_payload(ser, payload)
                last_seen = now
                has_seen_tag = True

                if now - last_print > 0.2:
                    last_print = now
                    z_corr = z_m * cfg.distance_scale
                    bearing = math.degrees(math.atan2(x_m, max(z_m, 1e-6)))
                    print(f"id={best_id} z={z_corr:.2f}m x={x_m * cfg.distance_scale:+.2f}m bearing={bearing:+.1f}deg cmd={payload}")

                if cfg.show_preview and best_corners is not None:
                    corners_i = best_corners.astype(np.int32)
                    cv2.polylines(frame, [corners_i.reshape(-1, 1, 2)], True, (0, 255, 255), 2)
                    cv2.putText(frame, payload, tuple(corners_i[0].tolist()), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                if (not has_seen_tag) or (now - last_seen > cfg.command_timeout):
                    send_payload(ser, "STOP")
                if now - last_print > 0.5:
                    last_print = now
                    print("tag lost -> STOP" if has_seen_tag else "waiting for tag -> STOP")

            if cfg.show_preview:
                cv2.imshow("AprilTag USB Follow", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            time.sleep(period)
    finally:
        try:
            send_payload(ser, "STOP")
        except Exception:
            pass
        cap.release()
        ser.close()
        if cfg.show_preview:
            cv2.destroyAllWindows()
        print("Follower stopped, STOP sent")


if __name__ == "__main__":
    main()
