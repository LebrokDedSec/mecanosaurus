#!/usr/bin/env python3
"""ROS2 AprilTag follow test node for webcam-based docking experiments."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
from geometry_msgs.msg import Twist
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node


@dataclass
class FollowConfig:
    camera_index: int
    tag_id: int
    tag_size_m: float
    fx: float
    fy: float
    cx: float
    cy: float
    target_distance_m: float
    max_linear_mps: float
    max_angular_radps: float
    kp_linear: float
    kp_angular: float
    lost_timeout_s: float
    topic: str
    loop_hz: float


class AprilTagFollowNode(Node):
    def __init__(self, cfg: FollowConfig) -> None:
        super().__init__("apriltag_follow_test")
        self.cfg = cfg
        self.pub = self.create_publisher(Twist, cfg.topic, 10)
        self.detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self.cap = cv2.VideoCapture(cfg.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {cfg.camera_index}")

        self.last_seen = 0.0
        self.last_print = 0.0

    def stop_robot(self) -> None:
        msg = Twist()
        self.pub.publish(msg)

    def find_target(self, gray: np.ndarray):
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.cfg.fx, self.cfg.fy, self.cfg.cx, self.cfg.cy),
            tag_size=self.cfg.tag_size_m,
        )

        wanted = [d for d in detections if d.tag_id == self.cfg.tag_id]
        if not wanted:
            return None

        return min(wanted, key=lambda d: float(d.pose_t[2][0]))

    def run_once(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Camera frame read failed")
            self.stop_robot()
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detection = self.find_target(gray)
        now = time.time()

        msg = Twist()
        if detection is not None:
            self.last_seen = now

            x_m = float(detection.pose_t[0][0])
            z_m = float(detection.pose_t[2][0])
            bearing_rad = math.atan2(x_m, z_m)
            distance_err = z_m - self.cfg.target_distance_m

            linear = self.cfg.kp_linear * max(distance_err, 0.0)
            angular = -self.cfg.kp_angular * bearing_rad

            msg.linear.x = float(
                np.clip(linear, 0.0, self.cfg.max_linear_mps)
            )
            msg.angular.z = float(
                np.clip(angular, -self.cfg.max_angular_radps, self.cfg.max_angular_radps)
            )

            if now - self.last_print > 0.2:
                self.last_print = now
                print(
                    f"tag={detection.tag_id} z={z_m:.3f}m x={x_m:.3f}m "
                    f"bearing={math.degrees(bearing_rad):.1f}deg "
                    f"cmd_v={msg.linear.x:.3f} cmd_w={msg.angular.z:.3f}"
                )
        else:
            if now - self.last_seen > self.cfg.lost_timeout_s:
                msg = Twist()
            if now - self.last_print > 0.5:
                self.last_print = now
                print("tag not visible -> stop/hold")

        self.pub.publish(msg)

    def close(self) -> None:
        self.stop_robot()
        if self.cap:
            self.cap.release()


def parse_args() -> FollowConfig:
    parser = argparse.ArgumentParser(description="ROS2 AprilTag follow test")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--tag-id", type=int, default=0, help="Target AprilTag id")
    parser.add_argument("--tag-size", type=float, default=0.12, help="Tag size in meters")

    parser.add_argument("--fx", type=float, required=True, help="Camera fx")
    parser.add_argument("--fy", type=float, required=True, help="Camera fy")
    parser.add_argument("--cx", type=float, required=True, help="Camera cx")
    parser.add_argument("--cy", type=float, required=True, help="Camera cy")

    parser.add_argument("--target-distance", type=float, default=0.50)
    parser.add_argument("--max-linear", type=float, default=0.25)
    parser.add_argument("--max-angular", type=float, default=1.2)
    parser.add_argument("--kp-linear", type=float, default=0.8)
    parser.add_argument("--kp-angular", type=float, default=2.0)
    parser.add_argument("--lost-timeout", type=float, default=0.4)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--hz", type=float, default=20.0)

    args = parser.parse_args()

    return FollowConfig(
        camera_index=args.camera,
        tag_id=args.tag_id,
        tag_size_m=args.tag_size,
        fx=args.fx,
        fy=args.fy,
        cx=args.cx,
        cy=args.cy,
        target_distance_m=args.target_distance,
        max_linear_mps=args.max_linear,
        max_angular_radps=args.max_angular,
        kp_linear=args.kp_linear,
        kp_angular=args.kp_angular,
        lost_timeout_s=args.lost_timeout,
        topic=args.topic,
        loop_hz=args.hz,
    )


def main() -> None:
    cfg = parse_args()
    rclpy.init()
    node = AprilTagFollowNode(cfg)

    rate = node.create_rate(cfg.loop_hz)
    try:
        while rclpy.ok():
            node.run_once()
            rclpy.spin_once(node, timeout_sec=0.0)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
