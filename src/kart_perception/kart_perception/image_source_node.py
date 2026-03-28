#!/usr/bin/env python3
import pathlib
from typing import List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import pyzed.sl as sl
    HAS_PYZED = True
except ImportError:
    HAS_PYZED = False

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".webm"}
SVO_EXTS = {".svo", ".svo2"}


class ImageSourceNode(Node):
    """@brief ROS2 node that publishes images from various sources at a fixed rate.

    Supports single images, image directories, video files, and webcam/device
    inputs. Optionally crops stereo side-by-side images to left half.
    """

    def __init__(self) -> None:
        """@brief Initialize the image source node.

        Declares parameters for source path, publish rate, looping behavior,
        frame ID, output topic, and stereo crop mode. Detects source type
        (webcam, device, directory, image, video) and sets up accordingly.
        """
        super().__init__("image_source")

        self.declare_parameter(
            "source", "tests/test_data/driverless_test_media/cones_test.png"
        )
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("stereo_crop", False)

        self.source = pathlib.Path(self.get_parameter("source").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._stereo_crop = bool(self.get_parameter("stereo_crop").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.image_topic, 10)

        self._image_paths: List[pathlib.Path] = []
        self._image_index = 0
        self._video_capture: Optional[cv2.VideoCapture] = None
        self._zed_camera = None
        self._zed_image = None

        source_str = str(self.source)
        # Webcam: integer index (e.g. "0") or device path (e.g. "/dev/video0")
        if source_str.isdigit():
            self._video_capture = cv2.VideoCapture(int(source_str))
            if not self._video_capture.isOpened():
                self.get_logger().error(f"Failed to open webcam {source_str}")
        elif self.source.is_char_device():
            self._video_capture = cv2.VideoCapture(source_str)
            if not self._video_capture.isOpened():
                self.get_logger().error(f"Failed to open device {source_str}")
        elif self.source.is_dir():
            self._image_paths = sorted(
                [p for p in self.source.iterdir() if p.suffix.lower() in IMAGE_EXTS]
            )
            if not self._image_paths:
                self.get_logger().error(f"No images found in {self.source}")
        elif self.source.is_file():
            if self.source.suffix.lower() in SVO_EXTS:
                if not HAS_PYZED:
                    self.get_logger().error("pyzed not installed — cannot play SVO files")
                else:
                    self._init_svo(source_str)
            elif self.source.suffix.lower() in IMAGE_EXTS:
                self._image_paths = [self.source]
            elif self.source.suffix.lower() in VIDEO_EXTS:
                self._video_capture = cv2.VideoCapture(str(self.source))
                if not self._video_capture.isOpened():
                    self.get_logger().error(f"Failed to open video {self.source}")
            else:
                self.get_logger().error(f"Unsupported source: {self.source}")
        else:
            self.get_logger().error(f"Source not found: {self.source}")

        if publish_rate <= 0:
            self.get_logger().error("publish_rate must be > 0")
            return

        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)

    def _init_svo(self, path: str) -> None:
        """@brief Open an SVO file with the ZED SDK for playback."""
        cam = sl.Camera()
        params = sl.InitParameters()
        params.set_from_svo_file(path)
        params.svo_real_time_mode = False
        err = cam.open(params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open SVO: {err}")
            return
        self._zed_camera = cam
        self._zed_image = sl.Mat()
        self.get_logger().info(f"SVO opened: {path} ({cam.get_svo_number_of_frames()} frames)")

    def _next_svo_frame(self):
        """@brief Grab the next frame from the SVO file.

        @return BGR frame as numpy array, or None if no frame available.
        """
        if self._zed_camera is None:
            return None
        err = self._zed_camera.grab()
        if err == sl.ERROR_CODE.SUCCESS:
            self._zed_camera.retrieve_image(self._zed_image, sl.VIEW.LEFT)
            # pyzed returns BGRA, convert to BGR
            return cv2.cvtColor(self._zed_image.get_data(), cv2.COLOR_BGRA2BGR)
        if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            if self.loop:
                self._zed_camera.set_svo_position(0)
                return None
            self.timer.cancel()
        return None

    def _publish_image(self, frame) -> None:
        """@brief Convert a BGR frame to a ROS Image message and publish it.

        Applies stereo crop (left half) if enabled.

        @param frame BGR image as numpy array.
        """
        if self._stereo_crop:
            frame = frame[:, :frame.shape[1] // 2]
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def _next_image(self):
        """@brief Load and return the next image from the image list.

        Advances the index and optionally loops back to the start.

        @return BGR image as numpy array, or None if no images or read failure.
        """
        if not self._image_paths:
            return None
        path = self._image_paths[self._image_index]
        frame = cv2.imread(str(path))
        if frame is None:
            self.get_logger().warning(f"Failed to read image {path}")
            return None
        self._image_index += 1
        if self._image_index >= len(self._image_paths):
            if self.loop:
                self._image_index = 0
            else:
                self.timer.cancel()
        return frame

    def _next_video_frame(self):
        """@brief Read the next frame from the video capture.

        Reopens the video if looping is enabled and the end is reached.

        @return BGR frame as numpy array, or None if no frame available.
        """
        if self._video_capture is None:
            return None
        ok, frame = self._video_capture.read()
        if ok:
            return frame
        if self.loop:
            self._video_capture.release()
            self._video_capture = cv2.VideoCapture(str(self.source))
            return None
        self.timer.cancel()
        return None

    def _on_timer(self) -> None:
        """@brief Timer callback that grabs the next frame and publishes it."""
        frame = None
        if self._zed_camera is not None:
            frame = self._next_svo_frame()
        elif self._video_capture is not None:
            frame = self._next_video_frame()
        else:
            frame = self._next_image()
        if frame is not None:
            self._publish_image(frame)


def main() -> None:
    """@brief Entry point for the image source node."""
    rclpy.init()
    node = ImageSourceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
