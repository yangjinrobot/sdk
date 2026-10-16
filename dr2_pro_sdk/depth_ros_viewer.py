#!/usr/bin/env python3
"""Display raw ROS depth and the exact depth_walk policy preprocessing result.

The default input is the 64x36, 32FC1 (metres) /DEPTH_FRAME published by 106.
Policy preprocessing mirrors common_policy_runner_depth.hpp:
crop (up=0, down=0, left=16, right=16), replace NaNs with 2.5 m,
Gaussian 3x3 sigma=1, clamp to [0, 2.5], then normalize by 2.5.
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


RAW_WIDTH = 64
RAW_HEIGHT = 36
CROP_UP = 0
CROP_DOWN = 0
CROP_LEFT = 16
CROP_RIGHT = 16


def decode_depth_metres(msg: Image) -> np.ndarray:
    """Decode 32FC1 metres or 16UC1 millimetres without cv_bridge."""
    encoding = msg.encoding.lower()
    byte_order = ">" if msg.is_bigendian else "<"

    if encoding == "32fc1":
        wire_dtype = np.dtype(byte_order + "f4")
        scale = 1.0
    elif encoding in ("16uc1", "mono16"):
        wire_dtype = np.dtype(byte_order + "u2")
        scale = 0.001
    else:
        raise ValueError(
            "unsupported encoding {!r}; expected 32FC1 or 16UC1".format(msg.encoding)
        )

    item_size = wire_dtype.itemsize
    required_step = msg.width * item_size
    if msg.step < required_step:
        raise ValueError(
            "invalid image step {} (need at least {})".format(msg.step, required_step)
        )
    required_bytes = msg.step * msg.height
    if len(msg.data) < required_bytes:
        raise ValueError(
            "short image buffer {} (need {})".format(len(msg.data), required_bytes)
        )

    depth = np.ndarray(
        shape=(msg.height, msg.width),
        dtype=wire_dtype,
        buffer=msg.data,
        strides=(msg.step, item_size),
    )
    return depth.astype(np.float32) * scale


def policy_preprocess(depth_metres: np.ndarray, max_depth: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return the 64x36 policy raw frame and normalized 36x32 encoder frame."""
    if depth_metres.shape != (RAW_HEIGHT, RAW_WIDTH):
        policy_raw = cv2.resize(
            depth_metres, (RAW_WIDTH, RAW_HEIGHT), interpolation=cv2.INTER_NEAREST
        )
    else:
        policy_raw = depth_metres.copy()

    cropped = policy_raw[
        CROP_UP : RAW_HEIGHT - CROP_DOWN,
        CROP_LEFT : RAW_WIDTH - CROP_RIGHT,
    ].copy()
    # This order intentionally matches the C++ runner: only NaNs are replaced
    # before filtering; clipping happens after GaussianBlur.
    cropped[np.isnan(cropped)] = max_depth
    processed = cv2.GaussianBlur(cropped, (3, 3), 1.0)
    processed = np.clip(processed, 0.0, max_depth)
    normalized = processed * (1.0 / max_depth)
    return policy_raw, normalized


def finite_range(image: np.ndarray) -> Tuple[float, float]:
    finite = image[np.isfinite(image)]
    if finite.size == 0:
        return float("nan"), float("nan")
    return float(np.min(finite)), float(np.max(finite))


def colour_depth(depth_metres: np.ndarray, max_depth: float, size: Tuple[int, int]) -> np.ndarray:
    display = np.nan_to_num(
        depth_metres, nan=max_depth, posinf=max_depth, neginf=0.0
    )
    display = np.clip(display, 0.0, max_depth)
    # Near objects are warm, far objects are cool.
    intensity = np.uint8(np.rint((1.0 - display / max_depth) * 255.0))
    coloured = cv2.applyColorMap(intensity, cv2.COLORMAP_TURBO)
    return cv2.resize(coloured, size, interpolation=cv2.INTER_NEAREST)


def make_depth_scale(width: int, max_depth: float) -> np.ndarray:
    """Create a horizontal colour ruler using the same mapping as the images."""
    height = 68
    ruler = np.zeros((height, width, 3), dtype=np.uint8)
    depths = np.linspace(0.0, max_depth, width, dtype=np.float32)
    intensity = np.uint8(np.rint((1.0 - depths / max_depth) * 255.0))
    colours = cv2.applyColorMap(intensity.reshape(1, width), cv2.COLORMAP_TURBO)
    ruler[5:27, :] = np.repeat(colours, 22, axis=0)

    tick_count = 5
    for index in range(tick_count + 1):
        value = max_depth * index / tick_count
        x = int(round((width - 1) * index / tick_count))
        cv2.line(ruler, (x, 27), (x, 34), (255, 255, 255), 1)
        label = "{:.1f} m".format(value)
        (text_width, _), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
        )
        text_x = max(0, min(x - text_width // 2, width - text_width))
        cv2.putText(
            ruler,
            label,
            (text_x, 51),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    cv2.putText(
        ruler,
        "Depth scale: near  ->  far",
        (max(0, width // 2 - 92), 66),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (190, 190, 190),
        1,
        cv2.LINE_AA,
    )
    return ruler


class DepthViewer(Node):
    def __init__(self, topic: str, max_depth: float, display_scale: int) -> None:
        super().__init__("depth_raw_policy_viewer")
        self.topic = topic
        self.max_depth = max_depth
        self.panel_size = (RAW_WIDTH * display_scale, RAW_HEIGHT * display_scale)
        self.latest: Optional[Tuple[np.ndarray, np.ndarray, str]] = None
        self.stop_requested = False
        self.received = 0
        self.decode_errors = 0
        self.start_time = time.monotonic()
        self.last_report = self.start_time

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(Image, topic, self.on_depth, qos)
        cv2.namedWindow("ROS depth: raw | policy processed", cv2.WINDOW_NORMAL)
        self.get_logger().info(
            "subscribing to {}; q/Esc=quit, s=save; max_depth={:.2f} m".format(
                topic, max_depth
            )
        )

    def on_depth(self, msg: Image) -> None:
        try:
            received_raw = decode_depth_metres(msg)
            policy_raw, normalized = policy_preprocess(received_raw, self.max_depth)
        except (ValueError, TypeError) as exc:
            self.decode_errors += 1
            if self.decode_errors <= 5 or self.decode_errors % 100 == 0:
                self.get_logger().error("depth decode failed: {}".format(exc))
            return

        self.received += 1
        description = "{}x{}/{}".format(msg.width, msg.height, msg.encoding)
        self.latest = (policy_raw, normalized, description)

    def render(self) -> None:
        if self.latest is None:
            waiting = np.zeros((240, 800, 3), dtype=np.uint8)
            cv2.putText(
                waiting,
                "Waiting for {} ...".format(self.topic),
                (25, 125),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 220, 255),
                2,
                cv2.LINE_AA,
            )
            frame = waiting
        else:
            policy_raw, normalized, description = self.latest
            processed_metres = normalized * self.max_depth
            raw_view = colour_depth(policy_raw, self.max_depth, self.panel_size)
            processed_view = colour_depth(
                processed_metres, self.max_depth, self.panel_size
            )

            raw_min, raw_max = finite_range(policy_raw)
            proc_min, proc_max = finite_range(normalized)
            cv2.putText(
                raw_view,
                "RAW {} -> 64x36  {:.3f}..{:.3f} m".format(
                    description, raw_min, raw_max
                ),
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                processed_view,
                "POLICY 36x32 normalized  {:.3f}..{:.3f}".format(
                    proc_min, proc_max
                ),
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            frame = np.hstack((raw_view, processed_view))
            frame = np.vstack((frame, make_depth_scale(frame.shape[1], self.max_depth)))

        cv2.imshow("ROS depth: raw | policy processed", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.stop_requested = True
        elif key == ord("s") and self.latest is not None:
            self.save_current()

        now = time.monotonic()
        if now - self.last_report >= 5.0:
            hz = self.received / max(now - self.start_time, 1.0e-6)
            self.get_logger().info(
                "received={} ({:.1f} Hz), decode_errors={}".format(
                    self.received, hz, self.decode_errors
                )
            )
            self.last_report = now

    def save_current(self) -> None:
        policy_raw, normalized, _ = self.latest  # type: ignore[misc]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = Path.cwd() / "depth_captures"
        output_dir.mkdir(parents=True, exist_ok=True)
        np.save(str(output_dir / (stamp + "_raw_metres.npy")), policy_raw)
        np.save(str(output_dir / (stamp + "_processed_normalized.npy")), normalized)
        raw_png = colour_depth(policy_raw, self.max_depth, self.panel_size)
        processed_png = colour_depth(
            normalized * self.max_depth, self.max_depth, self.panel_size
        )
        cv2.imwrite(str(output_dir / (stamp + "_raw.png")), raw_png)
        cv2.imwrite(str(output_dir / (stamp + "_processed.png")), processed_png)
        self.get_logger().info("saved frames to {}".format(output_dir))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="View raw ROS depth and depth_walk policy preprocessing"
    )
    parser.add_argument("--topic", default="/DEPTH_FRAME", help="ROS Image topic")
    parser.add_argument(
        "--max-depth", type=float, default=2.5, help="clipping depth in metres"
    )
    parser.add_argument(
        "--scale", type=int, default=10, help="display scale for each 64x36 panel"
    )
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> int:
    args = parse_args()
    if args.max_depth <= 0.0:
        raise SystemExit("--max-depth must be positive")
    if args.scale < 1:
        raise SystemExit("--scale must be at least 1")

    rclpy.init(args=sys.argv)
    node: Optional[DepthViewer] = None
    try:
        node = DepthViewer(args.topic, args.max_depth, args.scale)
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.render()
    except KeyboardInterrupt:
        pass
    except cv2.error as exc:
        print("OpenCV GUI error (is DISPLAY available?): {}".format(exc), file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
