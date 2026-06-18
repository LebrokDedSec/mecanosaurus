import argparse
import math
import threading
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyserial. Install with: pip install -r ESP32_LiDAR/viewer_requirements.txt"
    ) from exc


@dataclass
class BinPoint:
    distance_mm: float = 0.0
    confidence: int = 0
    updated_at: float = 0.0


class PointCloudState:
    def __init__(self, resolution_deg: float, min_confidence: int) -> None:
        self.bin_count = max(90, int(round(360.0 / resolution_deg)))
        self.resolution_deg = 360.0 / self.bin_count
        self.min_confidence = min_confidence
        self.bins = [BinPoint() for _ in range(self.bin_count)]
        self.lock = threading.Lock()
        self.rx_lines = 0
        self.bad_lines = 0

    def _normalize_angle(self, angle_deg: float) -> float:
        while angle_deg < 0.0:
            angle_deg += 360.0
        while angle_deg >= 360.0:
            angle_deg -= 360.0
        return angle_deg

    def mark_line(self) -> None:
        with self.lock:
            self.rx_lines += 1

    def mark_bad_line(self) -> None:
        with self.lock:
            self.bad_lines += 1

    def update_point(self, angle_deg: float, distance_mm: float, confidence: int) -> None:
        if distance_mm < 0:
            return
        angle_deg = self._normalize_angle(angle_deg)
        idx = int(angle_deg / self.resolution_deg) % self.bin_count
        now = time.time()
        with self.lock:
            self.bins[idx].distance_mm = distance_mm
            self.bins[idx].confidence = confidence
            self.bins[idx].updated_at = now

    def snapshot(self, max_age_s: float, max_distance_mm: float):
        now = time.time()
        points = []
        with self.lock:
            rx_lines = self.rx_lines
            bad_lines = self.bad_lines
            for i, point in enumerate(self.bins):
                if point.distance_mm <= 0:
                    continue
                if point.distance_mm > max_distance_mm:
                    continue
                if point.confidence < self.min_confidence:
                    continue
                if now - point.updated_at > max_age_s:
                    continue

                angle_deg = (i + 0.5) * self.resolution_deg
                angle_rad = math.radians(angle_deg)
                # Sensor orientation on this robot is mirrored against screen X.
                x = -math.cos(angle_rad) * point.distance_mm
                y = math.sin(angle_rad) * point.distance_mm
                points.append((x, y))

        return points, rx_lines, bad_lines


def serial_reader(port: str, baud: int, state: PointCloudState, stop_event: threading.Event) -> None:
    try:
        with serial.Serial(port=port, baudrate=baud, timeout=0.02) as ser:
            ser.reset_input_buffer()
            buffer = b""

            while not stop_event.is_set():
                waiting = ser.in_waiting
                if waiting <= 0:
                    time.sleep(0.002)
                    continue

                # If GUI cannot keep up, drop old backlog and keep stream "live".
                if waiting > 16384:
                    ser.reset_input_buffer()
                    buffer = b""
                    continue

                chunk = ser.read(waiting)
                if not chunk:
                    continue

                buffer += chunk
                lines = buffer.split(b"\n")
                buffer = lines[-1][-512:]

                for raw_line in lines[:-1]:
                    line = raw_line.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue

                    state.mark_line()
                    if not line.startswith("PT,"):
                        continue

                    parts = line.split(",")
                    if len(parts) < 5:
                        state.mark_bad_line()
                        continue

                    try:
                        angle_deg = float(parts[1])
                        distance_mm = float(parts[2])
                        confidence = int(parts[3])
                    except ValueError:
                        state.mark_bad_line()
                        continue

                    state.update_point(angle_deg, distance_mm, confidence)
    except serial.SerialException as exc:
        print(f"Serial error on {port}: {exc}")
        stop_event.set()


def build_plot(max_distance_mm: int):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("LiDAR 2D Top View")
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-max_distance_mm, max_distance_mm)
    ax.set_ylim(-max_distance_mm, max_distance_mm)
    ax.grid(True, linestyle=":", linewidth=0.6)

    ring_step = max(250, max_distance_mm // 8)
    for r in range(ring_step, max_distance_mm + 1, ring_step):
        circle = plt.Circle((0, 0), r, color="#b0b0b0", fill=False, linewidth=0.7, alpha=0.7)
        ax.add_patch(circle)

    ax.axhline(0, color="#808080", linewidth=0.8)
    ax.axvline(0, color="#808080", linewidth=0.8)

    scatter = ax.scatter([], [], s=9, c="#00a651", alpha=0.85)
    stats_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.7, "edgecolor": "#cccccc"},
    )
    return fig, ax, scatter, stats_text


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple 2D LiDAR viewer for ESP32 PT output")
    parser.add_argument("--port", default="COM17", help="Serial port, e.g. COM17")
    parser.add_argument("--baud", type=int, default=115200, help="USB serial baud rate")
    parser.add_argument("--range", type=int, default=4000, dest="max_range_mm", help="View range in mm")
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.5,
        help="Angular bin resolution in degrees",
    )
    parser.add_argument(
        "--max-age",
        type=float,
        default=0.35,
        dest="max_age_s",
        help="Discard points older than this many seconds",
    )
    parser.add_argument(
        "--min-confidence",
        type=int,
        default=60,
        dest="min_confidence",
        help="Minimum confidence filter",
    )
    args = parser.parse_args()

    state = PointCloudState(args.resolution, args.min_confidence)
    stop_event = threading.Event()

    reader = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baud, state, stop_event),
        daemon=True,
    )
    reader.start()

    fig, _, scatter, stats_text = build_plot(args.max_range_mm)

    def animate(_frame):
        points, rx_lines, bad_lines = state.snapshot(args.max_age_s, args.max_range_mm)
        if points:
            scatter.set_offsets(points)
        else:
            scatter.set_offsets(np.empty((0, 2)))

        stats_text.set_text(
            f"port={args.port}  points={len(points)}\\n"
            f"rx_lines={rx_lines}  bad_lines={bad_lines}\\n"
            f"min_conf={args.min_confidence}  max_age={args.max_age_s:.1f}s"
        )
        return scatter, stats_text

    def on_close(_event):
        stop_event.set()

    fig.canvas.mpl_connect("close_event", on_close)
    anim = FuncAnimation(fig, animate, interval=30, blit=False, cache_frame_data=False)
    plt.show()

    stop_event.set()
    reader.join(timeout=1.0)


if __name__ == "__main__":
    main()
