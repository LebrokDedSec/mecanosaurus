#!/usr/bin/env python3
"""Calibrate camera intrinsics using a chessboard pattern and save to NPZ."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Camera calibration with chessboard")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=640, help="Requested frame width")
    parser.add_argument("--height", type=int, default=480, help="Requested frame height")
    parser.add_argument("--cols", type=int, default=10, help="Inner corners per chessboard row")
    parser.add_argument("--rows", type=int, default=7, help="Inner corners per chessboard column")
    parser.add_argument("--square-mm", type=float, default=25.0, help="Chessboard square size in mm")
    parser.add_argument("--samples", type=int, default=25, help="Number of good views to collect")
    parser.add_argument("--out", default="config/camera_calib.npz", help="Output NPZ path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    pattern_size = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
    objp *= args.square_mm / 1000.0

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    print("Calibration started")
    print("Move chessboard around the frame: center, edges, and different tilt angles.")
    print("Press SPACE to capture current frame when corners are visible.")
    print("Press q to finish early.")

    frame_size = None
    while len(objpoints) < args.samples:
        ok, frame = cap.read()
        if not ok:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        display = frame.copy()
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, pattern_size, corners2, found)
            status = f"Corners found. SPACE to save ({len(objpoints)}/{args.samples})"
        else:
            corners2 = None
            status = f"Corners not found ({len(objpoints)}/{args.samples})"

        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Camera Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" ") and found and corners2 is not None:
            objpoints.append(objp.copy())
            imgpoints.append(corners2)
            print(f"Captured sample {len(objpoints)}/{args.samples}")

    cap.release()
    cv2.destroyAllWindows()

    if frame_size is None or len(objpoints) < 8:
        raise RuntimeError("Not enough valid samples for calibration (need at least 8)")

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        frame_size,
        None,
        None,
    )

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(out_path),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        rms=np.array([rms], dtype=np.float64),
        image_width=np.array([frame_size[0]], dtype=np.int32),
        image_height=np.array([frame_size[1]], dtype=np.int32),
        pattern_cols=np.array([args.cols], dtype=np.int32),
        pattern_rows=np.array([args.rows], dtype=np.int32),
        square_mm=np.array([args.square_mm], dtype=np.float64),
    )

    print(f"Saved calibration: {out_path}")
    print(f"RMS reprojection error: {rms:.4f}")
    print("Use this file with camera_viewer.py --calib-file")


if __name__ == "__main__":
    main()
