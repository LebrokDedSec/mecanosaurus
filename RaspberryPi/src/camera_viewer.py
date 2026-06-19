#!/usr/bin/env python3
"""Webcam viewer with AprilTag pose visualization using OpenCV."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None
try:
    import apriltag as apriltag_py
except ImportError:
    apriltag_py = None


APRILTAG_FAMILIES = {
    "16h5": "tag16h5",
    "25h9": "tag25h9",
    "36h10": "tag36h10",
    "36h11": "tag36h11",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Show live camera preview")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=640, help="Requested frame width")
    parser.add_argument("--height", type=int, default=480, help="Requested frame height")
    parser.add_argument(
        "--tag-family",
        default="36h11",
        choices=sorted(APRILTAG_FAMILIES.keys()),
        help="AprilTag family",
    )
    parser.add_argument(
        "--tag-id",
        type=int,
        default=-1,
        help="If >=0, draw cube only for this tag id",
    )
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.12,
        help="Tag size in meters",
    )
    parser.add_argument("--fx", type=float, default=0.0, help="Camera focal length fx")
    parser.add_argument("--fy", type=float, default=0.0, help="Camera focal length fy")
    parser.add_argument("--cx", type=float, default=0.0, help="Camera optical center cx")
    parser.add_argument("--cy", type=float, default=0.0, help="Camera optical center cy")
    parser.add_argument(
        "--calib-file",
        default="",
        help="Path to .npz calibration file with camera_matrix and dist_coeffs",
    )
    parser.add_argument(
        "--smooth-alpha",
        type=float,
        default=0.35,
        help="Pose smoothing factor in [0,1], lower means stronger smoothing",
    )
    parser.add_argument(
        "--quad-decimate",
        type=float,
        default=1.0,
        help="AprilTag detector decimation, >1 improves speed, <1 can improve quality",
    )
    parser.add_argument(
        "--quad-sigma",
        type=float,
        default=0.0,
        help="AprilTag detector blur sigma",
    )
    parser.add_argument(
        "--prism-depth",
        type=float,
        default=0.06,
        help="Prism extrusion depth in meters (smaller gives less bulky overlay)",
    )
    parser.add_argument(
        "--simple-overlay",
        action="store_true",
        help="Draw only the detected outline and axes, without any prism geometry",
    )
    parser.add_argument(
        "--distance-scale",
        type=float,
        default=1.0,
        help="Multiplier applied to distance and lateral offset to match real-world scale",
    )
    return parser.parse_args()


def camera_matrix_from_args(args: argparse.Namespace) -> np.ndarray:
    fx = args.fx if args.fx > 0 else float(args.width)
    fy = args.fy if args.fy > 0 else float(args.height)
    cx = args.cx if args.cx > 0 else float(args.width) / 2.0
    cy = args.cy if args.cy > 0 else float(args.height) / 2.0

    return np.array(
        [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def load_calibration(path_str: str):
    path = Path(path_str)
    if not path_str:
        return None, None
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {path}")

    data = np.load(str(path))
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def smooth_pose(
    marker_id: int,
    rvec: np.ndarray,
    tvec: np.ndarray,
    state: dict,
    alpha: float,
):
    if marker_id not in state:
        state[marker_id] = (rvec.copy(), tvec.copy())
        return rvec, tvec

    prev_rvec, prev_tvec = state[marker_id]
    rvec_s = alpha * prev_rvec + (1.0 - alpha) * rvec
    tvec_s = alpha * prev_tvec + (1.0 - alpha) * tvec
    state[marker_id] = (rvec_s, tvec_s)
    return rvec_s, tvec_s


def smooth_metrics(
    marker_id: int,
    metrics: tuple[float, float, float],
    state: dict,
    alpha: float,
) -> tuple[float, float, float]:
    if marker_id not in state:
        state[marker_id] = metrics
        return metrics

    prev = np.array(state[marker_id], dtype=np.float64)
    curr = np.array(metrics, dtype=np.float64)
    smoothed = alpha * prev + (1.0 - alpha) * curr
    result = (float(smoothed[0]), float(smoothed[1]), float(smoothed[2]))
    state[marker_id] = result
    return result


def order_corners_tl_tr_br_bl(corners: np.ndarray) -> np.ndarray:
    pts = np.array(corners, dtype=np.float64).reshape(4, 2)
    s = pts.sum(axis=1)
    d = (pts[:, 0] - pts[:, 1])

    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmax(d)]
    bl = pts[np.argmin(d)]
    return np.array([tl, tr, br, bl], dtype=np.float64)


def select_stable_pose(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    previous_pose: tuple[np.ndarray, np.ndarray] | None,
):
    result = cv2.solvePnPGeneric(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    if len(result) < 3:
        return None

    ok = bool(result[0])
    if not ok:
        return None

    rvecs = result[1]
    tvecs = result[2]
    reproj = result[3] if len(result) > 3 else None

    best_index = None
    best_score = None
    prev_rvec = previous_pose[0] if previous_pose is not None else None
    prev_tvec = previous_pose[1] if previous_pose is not None else None

    for idx, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        score = 0.0

        z_val = float(tvec.reshape(-1)[2])
        if z_val <= 0.0:
            score += 1e6
        else:
            # IPPE can return a geometrically valid but near-zero depth solution.
            # Penalize tiny Z heavily so frontal tags do not collapse to 0 m / 90 deg.
            score += 10.0 / max(z_val, 1e-6)

        if reproj is not None:
            err = float(np.array(reproj[idx]).reshape(-1)[0])
            score += err * 100.0

        if prev_rvec is not None and prev_tvec is not None:
            score += float(np.linalg.norm(rvec - prev_rvec)) * 25.0
            score += float(np.linalg.norm(tvec - prev_tvec)) * 250.0
        else:
            # With no previous pose, prefer the farther physically plausible solution.
            score += 2.0 / max(z_val, 1e-6)

        if best_score is None or score < best_score:
            best_score = score
            best_index = idx

    if best_index is None:
        return None

    return rvecs[best_index], tvecs[best_index]


def draw_simple_orientation_arrow(
    frame: np.ndarray,
    corners: np.ndarray,
) -> None:
    tilt = compute_tilt_indicator(corners)
    if tilt is None:
        return

    pts, centroid, direction, strength, tag_scale, _, _ = tilt
    if strength < 0.02:
        # Near-frontal view: draw only the center point.
        p0 = (int(round(float(centroid[0]))), int(round(float(centroid[1]))))
        cv2.circle(frame, p0, 4, (255, 255, 255), -1)
        return

    arrow_len = float(np.clip(tag_scale, 12.0, 40.0))
    end = centroid + direction * arrow_len

    p0 = (int(round(float(centroid[0]))), int(round(float(centroid[1]))))
    p1 = (int(round(float(end[0]))), int(round(float(end[1]))))
    cv2.circle(frame, p0, 4, (255, 255, 255), -1)
    cv2.arrowedLine(frame, p0, p1, (0, 255, 255), 2, tipLength=0.3)

    # Draw a subtle crosshair to make the center and arrow easier to read.
    cv2.line(frame, (p0[0] - 6, p0[1]), (p0[0] + 6, p0[1]), (255, 255, 255), 1)
    cv2.line(frame, (p0[0], p0[1] - 6), (p0[0], p0[1] + 6), (255, 255, 255), 1)


def draw_tilt_text(frame: np.ndarray, corners: np.ndarray) -> None:
    tilt = compute_tilt_indicator(corners)
    if tilt is None:
        return

    pts, _, _, _, _, pitch_deg, yaw_deg = tilt
    anchor = pts[0]
    text = f"x:{pitch_deg:+.0f} deg  y:{yaw_deg:+.0f} deg"
    x = int(round(float(anchor[0])))
    y = int(round(float(anchor[1]))) - 10
    cv2.putText(
        frame,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def estimate_tag_navigation(
    corners: np.ndarray,
    camera_matrix: np.ndarray,
    tag_size_m: float,
):
    pts = order_corners_tl_tr_br_bl(corners)
    if pts.shape != (4, 2):
        return None

    top_w = float(np.linalg.norm(pts[1] - pts[0]))
    bottom_w = float(np.linalg.norm(pts[2] - pts[3]))
    left_h = float(np.linalg.norm(pts[3] - pts[0]))
    right_h = float(np.linalg.norm(pts[2] - pts[1]))

    avg_w = max(0.5 * (top_w + bottom_w), 1e-6)
    avg_h = max(0.5 * (left_h + right_h), 1e-6)
    fx = float(camera_matrix[0][0])
    fy = float(camera_matrix[1][1])
    cx = float(camera_matrix[0][2])

    # Use the less-foreshortened dimension for a more navigation-friendly range estimate.
    z_from_width = fx * tag_size_m / avg_w
    z_from_height = fy * tag_size_m / avg_h
    forward_m = float(min(z_from_width, z_from_height))

    centroid = np.mean(pts, axis=0)
    bearing_deg = float(np.degrees(np.arctan2(float(centroid[0]) - cx, fx)))
    lateral_m = float(forward_m * np.tan(np.radians(bearing_deg)))
    return forward_m, bearing_deg, lateral_m


def navigation_from_tvec(tvec: np.ndarray) -> tuple[float, float, float]:
    vec = np.array(tvec, dtype=np.float64).reshape(-1)
    if vec.size < 3:
        raise ValueError("tvec must contain at least 3 elements")

    forward_m = float(vec[2])
    bearing_deg = float(np.degrees(np.arctan2(float(vec[0]), max(float(vec[2]), 1e-6))))
    lateral_m = float(vec[0])
    return forward_m, bearing_deg, lateral_m


def navigation_metrics_from_pose_or_outline(
    tvec: np.ndarray | None,
    corners: np.ndarray,
    camera_matrix: np.ndarray,
    tag_size_m: float,
):
    if tvec is not None:
        forward_m, bearing_deg, lateral_m = navigation_from_tvec(tvec)
        if forward_m >= max(tag_size_m * 0.5, 0.08):
            return forward_m, bearing_deg, lateral_m

    return estimate_tag_navigation(corners, camera_matrix, tag_size_m)


def apply_distance_scale(
    metrics: tuple[float, float, float] | None,
    distance_scale: float,
):
    if metrics is None:
        return None

    distance_m, bearing_deg, lateral_m = metrics
    return (
        float(distance_m) * distance_scale,
        float(bearing_deg),
        float(lateral_m) * distance_scale,
    )


def draw_navigation_text(
    frame: np.ndarray,
    anchor_xy: tuple[int, int],
    distance_m: float,
    bearing_deg: float,
    lateral_m: float,
) -> None:
    x = int(anchor_xy[0])
    y = int(anchor_xy[1]) + 20
    line1 = f"dist:{distance_m:.2f} m"
    line2 = f"dx:{lateral_m:+.2f} m  ang:{bearing_deg:+.1f} deg"
    cv2.putText(
        frame,
        line1,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        line2,
        (x, y + 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 220, 0),
        2,
        cv2.LINE_AA,
    )


def compute_tilt_indicator(corners: np.ndarray):
    pts = order_corners_tl_tr_br_bl(corners)
    if pts.shape != (4, 2):
        return None

    centroid = np.mean(pts, axis=0)

    top_w = float(np.linalg.norm(pts[1] - pts[0]))
    bottom_w = float(np.linalg.norm(pts[2] - pts[3]))
    left_h = float(np.linalg.norm(pts[3] - pts[0]))
    right_h = float(np.linalg.norm(pts[2] - pts[1]))

    width_sum = max(top_w + bottom_w, 1e-6)
    height_sum = max(left_h + right_h, 1e-6)

    vx = (left_h - right_h) / height_sum
    vy = (top_w - bottom_w) / width_sum
    direction = np.array([vx, vy], dtype=np.float64)
    strength = float(np.linalg.norm(direction))
    if not np.isfinite(strength):
        return None

    if strength >= 1e-6:
        direction = direction / strength
    else:
        direction = np.array([0.0, 0.0], dtype=np.float64)

    tag_scale = 0.25 * min(top_w + bottom_w, left_h + right_h)

    # Approximate tilt angles from relative shortening of opposite edges.
    pitch_deg = float(np.degrees(np.arctan2(bottom_w - top_w, width_sum)) * 2.0)
    yaw_deg = float(np.degrees(np.arctan2(right_h - left_h, height_sum)) * 2.0)
    return pts, centroid, direction, strength, tag_scale, pitch_deg, yaw_deg


def polygon_area(pts: np.ndarray) -> float:
    p = np.array(pts, dtype=np.float64).reshape(-1, 2)
    x = p[:, 0]
    y = p[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def project_points_safe(
    obj_pts: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
):
    img_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
    pts_f = img_pts.reshape(-1, 2)

    pts = []
    for p in pts_f:
        x = float(p[0])
        y = float(p[1])
        if not np.isfinite(x) or not np.isfinite(y):
            return None
        if abs(x) > 1e6 or abs(y) > 1e6:
            return None
        pts.append((int(round(x)), int(round(y))))
    return np.array(pts, dtype=np.int32)


def draw_prism(
    frame: np.ndarray,
    base_corners: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    tag_size: float,
    prism_depth: float,
) -> None:
    tilt = compute_tilt_indicator(base_corners)
    if tilt is None:
        return

    base_pts, centroid, direction, strength, tag_scale, _, _ = tilt

    base_area = polygon_area(base_pts)
    if base_area < 1.0:
        return

    z_m = max(float(abs(tvec.reshape(-1)[2])), 1e-6)
    depth_ratio = prism_depth / max(tag_size, 1e-6)
    shift_px = tag_scale * 2.0 * depth_ratio * np.clip(strength, 0.0, 1.0)
    if shift_px < 2.0:
        return

    top_pts = base_pts + direction.reshape(1, 2) * shift_px
    if not np.all(np.isfinite(top_pts)):
        return

    top_pts = np.array(top_pts, dtype=np.float64).reshape(4, 2)
    if polygon_area(top_pts) < 1.0:
        return

    top_area = polygon_area(top_pts)
    if top_area > 1.15 * base_area or top_area < 0.85 * base_area:
        return

    base_pts = np.round(base_pts).astype(np.int32)
    top_pts = np.round(top_pts).astype(np.int32)

    for i in range(4):
        b1 = tuple(int(v) for v in base_pts[i])
        b2 = tuple(int(v) for v in base_pts[(i + 1) % 4])
        t1 = tuple(int(v) for v in top_pts[i])
        t2 = tuple(int(v) for v in top_pts[(i + 1) % 4])

        # Bottom face = exact tag border in image.
        cv2.line(frame, b1, b2, (0, 255, 255), 2)
        # Top face = projected parallel face.
        cv2.line(frame, t1, t2, (0, 200, 255), 2)
        # Side edges = perspective cue for orientation.
        cv2.line(frame, b1, t1, (255, 0, 0), 2)


def main() -> None:
    args = parse_args()
    if Detector is None and apriltag_py is None:
        print("Missing dependency: install at least one backend")
        print("python3 -m pip install --user --break-system-packages apriltag")
        print("or")
        print("python3 -m pip install --user --break-system-packages pupil-apriltags")
        raise SystemExit(1)

    if args.smooth_alpha < 0.0 or args.smooth_alpha > 1.0:
        raise ValueError("--smooth-alpha must be in [0,1]")

    loaded_camera_matrix, loaded_dist_coeffs = load_calibration(args.calib_file)
    camera_matrix = loaded_camera_matrix if loaded_camera_matrix is not None else camera_matrix_from_args(args)
    dist_coeffs = loaded_dist_coeffs if loaded_dist_coeffs is not None else np.zeros((4, 1), dtype=np.float64)

    tag_obj_pts = np.array(
        [
            [-args.tag_size / 2.0, -args.tag_size / 2.0, 0.0],
            [args.tag_size / 2.0, -args.tag_size / 2.0, 0.0],
            [args.tag_size / 2.0, args.tag_size / 2.0, 0.0],
            [-args.tag_size / 2.0, args.tag_size / 2.0, 0.0],
        ],
        dtype=np.float32,
    )

    pose_state = {}
    nav_state = {}
    aruco_backend = hasattr(cv2.aruco, "detectMarkers")
    detector_params = None
    detector = None
    fallback_detector = None
    if aruco_backend:
        detector_params = cv2.aruco.DetectorParameters_create()
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector_params.cornerRefinementWinSize = 5
        detector_params.adaptiveThreshWinSizeMin = 3
        detector_params.adaptiveThreshWinSizeMax = 23
        detector_params.adaptiveThreshWinSizeStep = 10
        backend = "opencv_aruco"
    elif Detector is not None:
        detector = Detector(
            families=APRILTAG_FAMILIES[args.tag_family],
            nthreads=2,
            quad_decimate=args.quad_decimate,
            quad_sigma=args.quad_sigma,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        backend = "pupil_apriltags"
    else:
        options = apriltag_py.DetectorOptions(families=APRILTAG_FAMILIES[args.tag_family])
        fallback_detector = apriltag_py.Detector(options)
        backend = "apriltag"

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("Camera viewer started.")
    print(
        "AprilTag overlay enabled: "
        f"family={args.tag_family}, tag_id={'any' if args.tag_id < 0 else args.tag_id}, "
        f"size={args.tag_size}m"
    )
    if args.calib_file:
        print(f"Calibration: {args.calib_file}")
    else:
        print("Calibration: fallback intrinsics from width/height (less accurate)")
    print(f"Detector backend: {backend}")
    if args.simple_overlay:
        print("Overlay mode: simple")
    else:
        print("Overlay mode: prism")
    print("Press 'q' to quit.")

    prev = time.time()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Frame read failed")
                continue

            now = time.time()
            dt = max(now - prev, 1e-6)
            prev = now
            fps = 1.0 / dt

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = 0
            if aruco_backend:
                dict_name = f"DICT_APRILTAG_{args.tag_family}"
                dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
                corners_list, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector_params)

                if ids is not None and len(corners_list) > 0:
                    cv2.aruco.drawDetectedMarkers(frame, corners_list, ids)

                if ids is not None:
                    for i, marker_id in enumerate(ids.flatten()):
                        if args.tag_id >= 0 and marker_id != args.tag_id:
                            continue

                        corners_f = np.array(corners_list[i], dtype=np.float32).reshape(4, 2)
                        corners_i = corners_f.astype(np.int32)

                        if args.simple_overlay:
                            detections += 1
                            draw_simple_orientation_arrow(frame, corners_f)
                            draw_tilt_text(frame, corners_f)

                            anchor = tuple(corners_i[0].tolist())
                            stable_pose = select_stable_pose(
                                tag_obj_pts,
                                corners_f,
                                camera_matrix,
                                dist_coeffs,
                                pose_state.get(int(marker_id)),
                            )
                            nav_metrics = None
                            if stable_pose is not None:
                                rvec, tvec = stable_pose
                                rvec, tvec = smooth_pose(marker_id, rvec, tvec, pose_state, args.smooth_alpha)
                                nav_metrics = navigation_metrics_from_pose_or_outline(
                                    tvec,
                                    corners_f,
                                    camera_matrix,
                                    args.tag_size,
                                )
                            else:
                                nav_metrics = estimate_tag_navigation(corners_f, camera_matrix, args.tag_size)
                            nav_metrics = apply_distance_scale(nav_metrics, args.distance_scale)
                            if nav_metrics is not None:
                                distance_m, bearing_deg, lateral_m = smooth_metrics(
                                    int(marker_id),
                                    nav_metrics,
                                    nav_state,
                                    args.smooth_alpha,
                                )
                                draw_navigation_text(frame, anchor, distance_m, bearing_deg, lateral_m)
                            cv2.putText(
                                frame,
                                f"id={marker_id}",
                                anchor,
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (255, 255, 0),
                                2,
                                cv2.LINE_AA,
                            )
                            continue

                        stable_pose = select_stable_pose(
                            tag_obj_pts,
                            corners_f,
                            camera_matrix,
                            dist_coeffs,
                            pose_state.get(int(marker_id)),
                        )
                        if stable_pose is None:
                            continue

                        rvec, tvec = stable_pose

                        rvec, tvec = smooth_pose(marker_id, rvec, tvec, pose_state, args.smooth_alpha)

                        detections += 1
                        if args.simple_overlay:
                            draw_simple_orientation_arrow(frame, corners_f)
                        else:
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, args.tag_size * 0.6, 2)
                            draw_prism(
                                frame,
                                corners_f,
                                rvec,
                                tvec,
                                camera_matrix,
                                dist_coeffs,
                                args.tag_size,
                                args.prism_depth,
                            )

                        anchor = tuple(corners_i[0].tolist())
                        distance_m, bearing_deg, lateral_m = navigation_metrics_from_pose_or_outline(
                            tvec,
                            corners_f,
                            camera_matrix,
                            args.tag_size,
                        )
                        distance_m, bearing_deg, lateral_m = apply_distance_scale(
                            (distance_m, bearing_deg, lateral_m),
                            args.distance_scale,
                        )
                        distance_m, bearing_deg, lateral_m = smooth_metrics(
                            int(marker_id),
                            (distance_m, bearing_deg, lateral_m),
                            nav_state,
                            args.smooth_alpha,
                        )
                        cv2.putText(
                            frame,
                            f"id={marker_id} z={float(tvec[2][0]):.2f}m",
                            anchor,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
                        draw_navigation_text(frame, anchor, distance_m, bearing_deg, lateral_m)
            elif detector is not None:
                tags = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=(
                        float(camera_matrix[0][0]),
                        float(camera_matrix[1][1]),
                        float(camera_matrix[0][2]),
                        float(camera_matrix[1][2]),
                    ),
                    tag_size=args.tag_size,
                )

                for tag in tags:
                    marker_id = int(tag.tag_id)
                    if args.tag_id >= 0 and marker_id != args.tag_id:
                        continue

                    corners_f = tag.corners.astype(np.float32)
                    corners_i = corners_f.astype(np.int32)
                    cv2.polylines(frame, [corners_i.reshape(-1, 1, 2)], True, (0, 255, 255), 2)

                    if args.simple_overlay:
                        detections += 1
                        draw_simple_orientation_arrow(frame, corners_f)
                        draw_tilt_text(frame, corners_f)
                        anchor = tuple(corners_i[0].tolist())
                        nav_metrics = None
                        rmat = np.array(tag.pose_R, dtype=np.float64)
                        tvec = np.array(tag.pose_t, dtype=np.float64)
                        nav_metrics = navigation_metrics_from_pose_or_outline(
                            tvec,
                            corners_f,
                            camera_matrix,
                            args.tag_size,
                        )
                        nav_metrics = apply_distance_scale(nav_metrics, args.distance_scale)
                        if nav_metrics is not None:
                            distance_m, bearing_deg, lateral_m = smooth_metrics(
                                int(marker_id),
                                nav_metrics,
                                nav_state,
                                args.smooth_alpha,
                            )
                            draw_navigation_text(frame, anchor, distance_m, bearing_deg, lateral_m)
                        cv2.putText(
                            frame,
                            f"id={marker_id}",
                            anchor,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
                        continue

                    rmat = np.array(tag.pose_R, dtype=np.float64)
                    tvec = np.array(tag.pose_t, dtype=np.float64)
                    rvec, _ = cv2.Rodrigues(rmat)

                    rvec, tvec = smooth_pose(marker_id, rvec, tvec, pose_state, args.smooth_alpha)

                    detections += 1
                    if args.simple_overlay:
                        draw_simple_orientation_arrow(frame, corners_f)
                    else:
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, args.tag_size * 0.6, 2)
                        draw_prism(
                            frame,
                            corners_f,
                            rvec,
                            tvec,
                            camera_matrix,
                            dist_coeffs,
                            args.tag_size,
                            args.prism_depth,
                        )

                    z_m = float(tvec[2][0])
                    distance_m, bearing_deg, lateral_m = navigation_metrics_from_pose_or_outline(
                        tvec,
                        corners_f,
                        camera_matrix,
                        args.tag_size,
                    )
                    distance_m, bearing_deg, lateral_m = apply_distance_scale(
                        (distance_m, bearing_deg, lateral_m),
                        args.distance_scale,
                    )
                    distance_m, bearing_deg, lateral_m = smooth_metrics(
                        int(marker_id),
                        (distance_m, bearing_deg, lateral_m),
                        nav_state,
                        args.smooth_alpha,
                    )
                    anchor = tuple(corners_i[0].tolist())
                    cv2.putText(
                        frame,
                        f"id={marker_id} z={z_m:.2f}m",
                        anchor,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    draw_navigation_text(frame, anchor, distance_m, bearing_deg, lateral_m)
            else:
                tags = fallback_detector.detect(gray)
                for tag in tags:
                    marker_id = int(tag.tag_id)
                    if args.tag_id >= 0 and marker_id != args.tag_id:
                        continue

                    corners_f = np.array(tag.corners, dtype=np.float32)
                    corners_i = corners_f.astype(np.int32)
                    cv2.polylines(frame, [corners_i.reshape(-1, 1, 2)], True, (0, 255, 255), 2)

                    if args.simple_overlay:
                        detections += 1
                        draw_simple_orientation_arrow(frame, corners_f)
                        draw_tilt_text(frame, corners_f)
                        anchor = tuple(corners_i[0].tolist())
                        ok_pnp, rvec, tvec = cv2.solvePnP(
                            tag_obj_pts,
                            corners_f,
                            camera_matrix,
                            dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE,
                        )
                        nav_metrics = None
                        if ok_pnp:
                            rvec, tvec = smooth_pose(marker_id, rvec, tvec, pose_state, args.smooth_alpha)
                            nav_metrics = navigation_metrics_from_pose_or_outline(
                                tvec,
                                corners_f,
                                camera_matrix,
                                args.tag_size,
                            )
                        else:
                            nav_metrics = estimate_tag_navigation(corners_f, camera_matrix, args.tag_size)
                        nav_metrics = apply_distance_scale(nav_metrics, args.distance_scale)
                        if nav_metrics is not None:
                            distance_m, bearing_deg, lateral_m = smooth_metrics(
                                int(marker_id),
                                nav_metrics,
                                nav_state,
                                args.smooth_alpha,
                            )
                            draw_navigation_text(frame, anchor, distance_m, bearing_deg, lateral_m)
                        cv2.putText(
                            frame,
                            f"id={marker_id}",
                            anchor,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )
                        continue

                    ok_pnp, rvec, tvec = cv2.solvePnP(
                        tag_obj_pts,
                        corners_f,
                        camera_matrix,
                        dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if not ok_pnp:
                        continue

                    rvec, tvec = smooth_pose(marker_id, rvec, tvec, pose_state, args.smooth_alpha)

                    detections += 1
                    if args.simple_overlay:
                        draw_simple_orientation_arrow(frame, corners_f)
                    else:
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, args.tag_size * 0.6, 2)
                        draw_prism(
                            frame,
                            corners_f,
                            rvec,
                            tvec,
                            camera_matrix,
                            dist_coeffs,
                            args.tag_size,
                            args.prism_depth,
                        )

                    z_m = float(tvec[2][0])
                    distance_m, bearing_deg, lateral_m = navigation_metrics_from_pose_or_outline(
                        tvec,
                        corners_f,
                        camera_matrix,
                        args.tag_size,
                    )
                    distance_m, bearing_deg, lateral_m = apply_distance_scale(
                        (distance_m, bearing_deg, lateral_m),
                        args.distance_scale,
                    )
                    distance_m, bearing_deg, lateral_m = smooth_metrics(
                        int(marker_id),
                        (distance_m, bearing_deg, lateral_m),
                        nav_state,
                        args.smooth_alpha,
                    )
                    anchor = tuple(corners_i[0].tolist())
                    cv2.putText(
                        frame,
                        f"id={marker_id} z={z_m:.2f}m",
                        anchor,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    draw_navigation_text(frame, anchor, distance_m, bearing_deg, lateral_m)

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"tags: {detections}",
                (10, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Mecanosaurus Camera Viewer", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
