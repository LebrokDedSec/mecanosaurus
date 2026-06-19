#!/usr/bin/env python3
"""Generate a printable AprilTag image (36h11) using OpenCV aruco."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


APRILTAG_DICTS = {
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate printable AprilTag PNG")
    parser.add_argument("--tag-id", type=int, default=0, help="AprilTag id")
    parser.add_argument(
        "--family",
        default="36h11",
        choices=sorted(APRILTAG_DICTS.keys()),
        help="AprilTag family",
    )
    parser.add_argument(
        "--size-px",
        type=int,
        default=800,
        help="Tag size in pixels (without white border)",
    )
    parser.add_argument(
        "--border-px",
        type=int,
        default=100,
        help="White border around tag in pixels",
    )
    parser.add_argument(
        "--out",
        default="apriltag_36h11_id0.png",
        help="Output PNG path",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.size_px <= 0:
        raise ValueError("--size-px must be > 0")
    if args.border_px < 0:
        raise ValueError("--border-px must be >= 0")

    dictionary = cv2.aruco.getPredefinedDictionary(APRILTAG_DICTS[args.family])
    tag = cv2.aruco.generateImageMarker(dictionary, args.tag_id, args.size_px)

    total = args.size_px + 2 * args.border_px
    canvas = 255 * (cv2.UMat(total, total, cv2.CV_8UC1).get())
    start = args.border_px
    end = start + args.size_px
    canvas[start:end, start:end] = tag

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_path), canvas):
        raise RuntimeError(f"Could not write output image: {out_path}")

    print(f"Saved: {out_path}")
    print("Print at 100% scale (no fit-to-page) for best geometry accuracy.")


if __name__ == "__main__":
    main()
