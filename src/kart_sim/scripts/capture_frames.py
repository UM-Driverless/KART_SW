#!/usr/bin/env python3
"""Capture RGB and depth frames from the Gazebo simulation camera.

Subscribes to the bridged camera topics and saves N frames to disk as PNG.

Usage:
    ros2 run kart_sim capture_frames.py --ros-args \
        -p num_frames:=20 \
        -p output_dir:=/tmp/gazebo_frames
"""
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameCaptureNode(Node):
    """@brief Capture RGB and depth frames from Gazebo camera topics to disk.

    Saves a configurable number of frames as PNG files, skipping initial
    frames to allow the renderer to stabilize. Exits when done.
    """

    def __init__(self):
        """@brief Initialize with capture parameters, output directories, and image subscribers."""
        super().__init__("frame_capture")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("output_dir", "/tmp/gazebo_frames")
        self.declare_parameter("num_frames", 20)
        self.declare_parameter("skip_initial", 5)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.num_frames = int(self.get_parameter("num_frames").value)
        self.skip_initial = int(self.get_parameter("skip_initial").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "rgb").mkdir(exist_ok=True)
        (self.output_dir / "depth").mkdir(exist_ok=True)

        self.bridge = CvBridge()
        self.rgb_count = 0
        self.depth_count = 0
        self.rgb_skip = 0
        self.depth_skip = 0

        self.rgb_sub = self.create_subscription(
            Image, self.image_topic, self._on_rgb, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._on_depth, 10
        )

        self.get_logger().info(
            f"Capturing {self.num_frames} frames to {self.output_dir} "
            f"(skipping first {self.skip_initial})"
        )

    def _on_rgb(self, msg: Image):
        """@brief Callback for RGB image messages. Saves frame as PNG after skipping initial frames.

        @param msg Image message in BGR8 encoding.
        """
        # Skip initial frames (rendering may not be ready)
        if self.rgb_skip < self.skip_initial:
            self.rgb_skip += 1
            return

        if self.rgb_count >= self.num_frames:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        path = self.output_dir / "rgb" / f"frame_{self.rgb_count:04d}.png"
        cv2.imwrite(str(path), frame)
        self.rgb_count += 1
        self.get_logger().info(f"RGB [{self.rgb_count}/{self.num_frames}] → {path.name}")

        self._check_done()

    def _on_depth(self, msg: Image):
        """@brief Callback for depth image messages. Saves as 16-bit PNG (millimeters).

        @param msg Image message in 32FC1 encoding (meters).
        """
        if self.depth_skip < self.skip_initial:
            self.depth_skip += 1
            return

        if self.depth_count >= self.num_frames:
            return

        # Depth comes as 32FC1 (meters). Save as 16-bit PNG (mm) for lossless storage.
        depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        depth_mm = (depth_m * 1000.0).astype(np.uint16)
        path = self.output_dir / "depth" / f"frame_{self.depth_count:04d}.png"
        cv2.imwrite(str(path), depth_mm)
        self.depth_count += 1
        self.get_logger().info(f"Depth [{self.depth_count}/{self.num_frames}] → {path.name}")

        self._check_done()

    def _check_done(self):
        if self.rgb_count >= self.num_frames and self.depth_count >= self.num_frames:
            self.get_logger().info(
                f"Done! Captured {self.rgb_count} RGB + {self.depth_count} depth frames "
                f"to {self.output_dir}"
            )
            raise SystemExit(0)


def main():
    rclpy.init()
    node = FrameCaptureNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
