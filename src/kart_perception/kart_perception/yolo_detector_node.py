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
    """Resolve a path relative to the kart_brain repo root if not absolute."""
    p = pathlib.Path(path_str)
    if p.is_absolute():
        return p
    candidate = _REPO_ROOT / p
    if candidate.exists():
        return candidate
    # Fallback: ~/kart_brain (works on all our machines)
    return pathlib.Path.home() / "kart_brain" / p


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("detections_topic", "/perception/cones_2d")
        self.declare_parameter("debug_image_topic", "/perception/yolo/annotated")
        self.declare_parameter("weights_path", "models/perception/yolo/nava_yolov11_2026_02.engine")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("publish_debug_image", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.weights_path = _repo_relative(self.get_parameter("weights_path").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.debug_publisher = self.create_publisher(Image, self.debug_image_topic, 10)
        self.fps_publisher = self.create_publisher(Float32, "/perception/yolo/fps", 10)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self._on_image, 1
        )

        self._use_ultralytics = False
        self._device = "cpu"
        self.model = self._load_model()
        self.class_names = self._get_class_names()

        # Threading: ROS callback stores latest frame, inference thread processes it
        self._latest_frame = None  # (header, frame_bgr)
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._shutdown = False

        self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._infer_thread.start()

        # FPS counter
        self._fps_count = 0
        self._fps_time = time.monotonic()

    def _resolve_device(self) -> str:
        device = self.device
        if not device:
            import torch
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        return device

    def _load_model(self):
        if not self.weights_path.exists():
            self.get_logger().error(f"Weights not found: {self.weights_path}")
            return None
        os.environ.setdefault("MPLBACKEND", "Agg")

        device = self._resolve_device()
        self.get_logger().info(f"Loading YOLO weights: {self.weights_path} on {device}")

        # Try ultralytics (YOLOv8/v11) first, fall back to torch.hub (YOLOv5)
        try:
            from ultralytics import YOLO
            model = YOLO(str(self.weights_path))
            self._device = device
            self._use_ultralytics = True
            self.get_logger().info(f"Loaded model via ultralytics API (will use device: {device})")
            return model
        except Exception as exc:
            self.get_logger().warn(f"ultralytics load failed ({exc}), trying torch.hub YOLOv5...")

        try:
            import torch
            model = torch.hub.load(
                "ultralytics/yolov5", "custom", path=str(self.weights_path)
            )
            model.conf = self.conf_threshold
            model.iou = self.iou_threshold
            model.imgsz = self.imgsz
            model.to(device)
            self._use_ultralytics = False
            self.get_logger().info(f"Loaded model via torch.hub YOLOv5 (device: {device})")
            return model
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            return None

    def _get_class_names(self):
        if self.model is None:
            return None
        if self._use_ultralytics:
            return self.model.names  # dict {0: "blue_cone", ...}
        return self.model.names  # same format for YOLOv5

    def _on_image(self, msg: Image) -> None:
        """ROS callback — just grab the frame and signal the inference thread."""
        if self.model is None:
            return
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._frame_lock:
            self._latest_frame = (msg.header, frame_bgr)
        self._frame_event.set()

    def _inference_loop(self) -> None:
        """Dedicated thread: grab latest frame, run inference, publish results."""
        while not self._shutdown:
            # Wait for a new frame (with timeout so we can check shutdown)
            if not self._frame_event.wait(timeout=1.0):
                continue
            self._frame_event.clear()

            # Grab the latest frame
            with self._frame_lock:
                data = self._latest_frame
                self._latest_frame = None
            if data is None:
                continue

            header, frame_bgr = data

            if self._use_ultralytics:
                self._infer_ultralytics(header, frame_bgr)
            else:
                self._infer_yolov5(header, frame_bgr)

            # FPS logging + publish
            self._fps_count += 1
            now = time.monotonic()
            elapsed = now - self._fps_time
            if elapsed >= 2.0:
                fps = self._fps_count / elapsed
                self.get_logger().info(f"YOLO inference: {fps:.1f} Hz")
                self.fps_publisher.publish(Float32(data=fps))
                self._fps_count = 0
                self._fps_time = now

    def _infer_ultralytics(self, header, frame_bgr) -> None:
        results = self.model(
            frame_bgr,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self._device,
            verbose=False,
        )
        detections = Detection2DArray()
        detections.header = header
        parsed = []

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
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

                color = CLASS_COLORS.get(name, DEFAULT_COLOR)
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                parsed.append((p1, p2, color, name, conf))

        self.publisher.publish(detections)

        if self.publish_debug_image:
            debug_img = frame_bgr.copy()
            for p1, p2, color, name, conf in parsed:
                cv2.rectangle(debug_img, p1, p2, color, 3)
            for p1, _, color, name, conf in parsed:
                label = f"{name.replace('_cone', '')} {conf:.0%}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(debug_img, (p1[0], p1[1] - th - 8), (p1[0] + tw + 6, p1[1]), color, -1)
                cv2.putText(debug_img, label, (p1[0] + 3, p1[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header = header
            self.debug_publisher.publish(debug_msg)

    def _infer_yolov5(self, header, frame_bgr) -> None:
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self.model(frame_rgb)
        detections = Detection2DArray()
        detections.header = header

        for det in results.xyxy[0].tolist():
            x1, y1, x2, y2, conf, cls_id = det
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.center.theta = 0.0
            bbox.size_x = max(0.0, x2 - x1)
            bbox.size_y = max(0.0, y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            if self.class_names is not None:
                hypothesis.hypothesis.class_id = str(self.class_names[int(cls_id)])
            else:
                hypothesis.hypothesis.class_id = str(int(cls_id))
            hypothesis.hypothesis.score = float(conf)

            detection = Detection2D()
            detection.bbox = bbox
            detection.results.append(hypothesis)
            detections.detections.append(detection)

        self.publisher.publish(detections)

        if self.publish_debug_image:
            debug_img = frame_bgr.copy()
            parsed = []
            for det in results.xyxy[0].tolist():
                x1, y1, x2, y2, conf, cls_id = det
                name = str(self.class_names[int(cls_id)]) if self.class_names else str(int(cls_id))
                color = CLASS_COLORS.get(name, DEFAULT_COLOR)
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                parsed.append((p1, p2, color, name, conf))
                cv2.rectangle(debug_img, p1, p2, color, 3)
            for p1, _, color, name, conf in parsed:
                label = f"{name.replace('_cone', '')} {conf:.0%}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(debug_img, (p1[0], p1[1] - th - 8), (p1[0] + tw + 6, p1[1]), color, -1)
                cv2.putText(debug_img, label, (p1[0] + 3, p1[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header = header
            self.debug_publisher.publish(debug_msg)


def main() -> None:
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
