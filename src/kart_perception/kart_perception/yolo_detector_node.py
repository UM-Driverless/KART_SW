#!/usr/bin/env python3
import os
import pathlib
import threading
import time
import warnings

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


warnings.filterwarnings(
    "ignore",
    message=".*autocast\\(args...\\).*deprecated.*",
    category=FutureWarning,
)


# Per-class colors (BGR) for debug image rendering
CLASS_COLORS = {
    "blue_cone": (255, 150, 0),
    "yellow_cone": (0, 230, 255),
    "orange_cone": (0, 140, 255),
    "large_orange_cone": (0, 100, 255),
}
DEFAULT_COLOR = (200, 200, 200)

# Repo root: two levels up from this file's installed/source location.
# Handles both `colcon build` installs and direct source execution.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]  # .../src/kart_perception/kart_perception/file.py


def _repo_relative(path_str: str) -> pathlib.Path:
    """@brief Resolve a path relative to the kart_brain repo root if not absolute.

    @param path_str Path string, either absolute or relative to repo root.
    @return Resolved absolute path.
    """
    p = pathlib.Path(path_str)
    if p.is_absolute():
        return p
    candidate = _REPO_ROOT / p
    if candidate.exists():
        return candidate
    # Fallback: ~/kart_brain (works on all our machines)
    return pathlib.Path.home() / "kart_brain" / p


class YoloDetectorNode(Node):
    """@brief ROS2 node that runs YOLO inference on camera images and publishes 2D cone detections.

    Uses ultralytics (YOLOv8/v11) backend, supporting .pt and .engine (TensorRT) weights.
    Inference runs on a dedicated thread to decouple ROS callbacks from GPU work.
    Optionally publishes annotated debug images with bounding boxes.
    """

    def __init__(self) -> None:
        """@brief Initialize the YOLO detector node.

        Declares parameters for topics, model weights, confidence/IOU thresholds,
        image size, and device selection. Loads the YOLO model and starts the
        inference thread.
        """
        super().__init__("yolo_detector")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("detections_topic", "/perception/cones_2d")
        self.declare_parameter("debug_image_topic", "/perception/yolo/annotated")
        self.declare_parameter("weights_path", "models/perception/yolo/ruben_yolov11n_2026_03_320.engine")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("device", "")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("crop_top_ratio", 0.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.weights_path = _repo_relative(self.get_parameter("weights_path").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.crop_top_ratio = float(self.get_parameter("crop_top_ratio").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.debug_publisher = self.create_publisher(Image, self.debug_image_topic, 10)
        self.fps_publisher = self.create_publisher(Float32, "/perception/yolo/fps", 10)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self._on_image, 1
        )

        self._device = "cpu"
        self.model = self._load_model()
        self.class_names = self._get_class_names()

        # Threading: ROS callback stores latest raw msg, inference thread decodes + infers
        self._latest_msg = None
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._shutdown = False

        self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._infer_thread.start()

        # FPS counter
        self._fps_count = 0
        self._fps_time = time.monotonic()

    def _resolve_device(self) -> str:
        """@brief Determine the compute device for inference.

        @return Device string (e.g. "cuda:0" or "cpu").
        """
        device = self.device
        if not device:
            import torch
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        return device

    def _load_model(self):
        """@brief Load the YOLO model via ultralytics (.pt or .engine).

        @return Loaded model object, or None on failure.
        """
        if not self.weights_path.exists():
            self.get_logger().error(f"Weights not found: {self.weights_path}")
            return None
        os.environ.setdefault("MPLBACKEND", "Agg")

        device = self._resolve_device()
        self.get_logger().info(f"Loading YOLO weights: {self.weights_path} on {device}")

        try:
            from ultralytics import YOLO
            model = YOLO(str(self.weights_path))
            self._device = device
            self.get_logger().info(f"Loaded model via ultralytics API (device: {device})")
            return model
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            return None

    # Canonical class names matching our dataset.yaml
    EXPECTED_CLASS_NAMES = {
        0: "blue_cone",
        1: "yellow_cone",
        2: "orange_cone",
        3: "large_orange_cone",
    }

    def _get_class_names(self):
        """@brief Retrieve and validate class names from the loaded model.

        Overrides model names with canonical cone class names if they don't match.

        @return Dictionary mapping class index to class name, or None if no model.
        """
        if self.model is None:
            return None
        names = self.model.names
        # Override generic names (e.g. "class0") with canonical cone class names
        if names and any(v != self.EXPECTED_CLASS_NAMES.get(k) for k, v in names.items()):
            self.get_logger().warn(
                f"Model class names don't match expected; overriding with {self.EXPECTED_CLASS_NAMES}"
            )
            return dict(self.EXPECTED_CLASS_NAMES)
        return names

    def _on_image(self, msg: Image) -> None:
        """@brief ROS callback for incoming camera images.

        Stores the raw message and signals the inference thread. No decoding here
        to keep the callback lightweight.

        @param msg Raw camera image message.
        """
        if self.model is None:
            return
        with self._frame_lock:
            self._latest_msg = msg
        self._frame_event.set()

    def _inference_loop(self) -> None:
        """@brief Dedicated inference thread loop.

        Waits for new frames, decodes them, runs YOLO inference, publishes
        detections and optional debug images, and tracks FPS.
        """
        while not self._shutdown:
            # Wait for a new frame (with timeout so we can check shutdown)
            if not self._frame_event.wait(timeout=1.0):
                continue
            self._frame_event.clear()

            # Grab the latest raw message and decode in this thread
            with self._frame_lock:
                msg = self._latest_msg
                self._latest_msg = None
            if msg is None:
                continue

            header = msg.header
            t_decode = time.monotonic()
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            t_decode = time.monotonic() - t_decode

            want_debug = (
                self.publish_debug_image
                and self.debug_publisher.get_subscription_count() > 0
            )

            # Crop top of image (sky removal) to speed up inference
            crop_y = int(frame_bgr.shape[0] * self.crop_top_ratio)
            if crop_y > 0:
                cropped = frame_bgr[crop_y:]
            else:
                cropped = frame_bgr

            t0 = time.monotonic()
            self._infer(header, cropped, want_debug, frame_bgr, crop_y)
            t_infer = time.monotonic() - t0

            # FPS logging + publish
            self._fps_count += 1
            now = time.monotonic()
            elapsed = now - self._fps_time
            if elapsed >= 2.0:
                fps = self._fps_count / elapsed
                self.get_logger().info(
                    f"YOLO: {fps:.1f} Hz  decode={t_decode*1000:.1f}ms "
                    f"infer={t_infer*1000:.1f}ms  debug={'Y' if want_debug else 'N'}"
                )
                self.fps_publisher.publish(Float32(data=fps))
                self._fps_count = 0
                self._fps_time = now

    def _infer(self, header, cropped_bgr, want_debug: bool,
               full_bgr=None, crop_y: int = 0) -> None:
        """@brief Run YOLO inference and publish detections and optional debug image.

        @param header ROS message header to stamp output messages.
        @param cropped_bgr Cropped BGR image fed to YOLO.
        @param want_debug If True, draw bounding boxes and publish debug image.
        @param full_bgr Original full-size BGR image for debug drawing.
        @param crop_y Number of pixels cropped from the top.
        """
        results = self.model(
            cropped_bgr,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self._device,
            verbose=False,
        )
        detections = Detection2DArray()
        detections.header = header

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                # Offset y-coordinates back to full-image space
                y1 += crop_y
                y2 += crop_y
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                name = self.class_names[cls_id] if self.class_names else str(cls_id)

                bbox = BoundingBox2D()
                bbox.center.position.x = (x1 + x2) / 2.0
                bbox.center.position.y = (y1 + y2) / 2.0
                bbox.center.theta = 0.0
                bbox.size_x = max(0.0, x2 - x1)
                bbox.size_y = max(0.0, y2 - y1)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(name)
                hypothesis.hypothesis.score = conf

                detection = Detection2D()
                detection.bbox = bbox
                detection.results.append(hypothesis)
                detections.detections.append(detection)

                if want_debug:
                    color = CLASS_COLORS.get(name, DEFAULT_COLOR)
                    p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                    cv2.rectangle(full_bgr, p1, p2, color, 3)
                    label = f"{name.replace('_cone', '')} {conf:.0%}"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(full_bgr, (p1[0], p1[1] - th - 8), (p1[0] + tw + 6, p1[1]), color, -1)
                    cv2.putText(full_bgr, label, (p1[0] + 3, p1[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)

        self.publisher.publish(detections)

        if want_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(full_bgr, encoding="bgr8")
            debug_msg.header = header
            self.debug_publisher.publish(debug_msg)


def main() -> None:
    """@brief Entry point for the YOLO detector node."""
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node._shutdown = True
        node._frame_event.set()  # unblock inference thread
        node._infer_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
