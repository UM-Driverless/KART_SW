#!/usr/bin/env python3
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


class ConeDepthLocalizerNode(Node):
    """@brief ROS2 node that fuses 2D cone detections with a depth map to produce 3D cone positions.

    Synchronizes YOLO 2D detections, a depth image, and camera info using
    ApproximateTimeSynchronizer. For each 2D detection, samples the median depth
    in a small ROI around the bounding box center and back-projects to 3D using
    the pinhole camera model.
    """

    def __init__(self) -> None:
        """@brief Initialize the cone depth localizer node.

        Declares parameters for input/output topics, depth ROI size, depth range
        limits, default 3D bounding box size, and time synchronization slop.
        """
        super().__init__("cone_depth_localizer")

        self.declare_parameter("detections_topic", "/perception/cones_2d")
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("output_topic", "/perception/cones_3d")
        self.declare_parameter("depth_roi_px", 5)
        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 40.0)
        self.declare_parameter("default_box_size_m", 0.25)
        self.declare_parameter("sync_slop", 2.0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.depth_roi_px = int(self.get_parameter("depth_roi_px").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.default_box_size_m = float(self.get_parameter("default_box_size_m").value)

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_ready = False

        self.publisher = self.create_publisher(Detection3DArray, self.output_topic, 10)

        self.det_sub = Subscriber(self, Detection2DArray, self.detections_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.info_sub = Subscriber(self, CameraInfo, self.camera_info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.det_sub, self.depth_sub, self.info_sub], queue_size=30, slop=float(self.get_parameter("sync_slop").value)
        )
        self.sync.registerCallback(self._on_synced)

    def _update_camera_model(self, info_msg: CameraInfo) -> None:
        """@brief Initialize the pinhole camera model from camera info (once).

        @param info_msg Camera intrinsics and distortion parameters.
        """
        if not self.camera_info_ready:
            self.camera_model.fromCameraInfo(info_msg)
            self.camera_info_ready = True

    def _depth_to_meters(self, depth_img: np.ndarray, encoding: str) -> np.ndarray:
        """@brief Convert a depth image to float32 meters.

        @param depth_img Raw depth image array.
        @param encoding ROS image encoding string (e.g. "16UC1", "32FC1").
        @return Depth image in meters as float32.
        """
        if encoding in ("16UC1", "mono16"):
            return depth_img.astype(np.float32) * 0.001
        return depth_img.astype(np.float32)

    def _median_depth(
        self, depth_m: np.ndarray, u: int, v: int, radius: int
    ) -> Optional[float]:
        """@brief Compute the median valid depth in a square ROI around a pixel.

        @param depth_m Depth image in meters.
        @param u Horizontal pixel coordinate (column).
        @param v Vertical pixel coordinate (row).
        @param radius Half-size of the square ROI in pixels.
        @return Median depth in meters, or None if no valid depths in the ROI.
        """
        h, w = depth_m.shape[:2]
        u0 = max(0, u - radius)
        u1 = min(w - 1, u + radius)
        v0 = max(0, v - radius)
        v1 = min(h - 1, v + radius)
        roi = depth_m[v0 : v1 + 1, u0 : u1 + 1].reshape(-1)
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > self.min_depth_m) & (valid < self.max_depth_m)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _project(self, u: float, v: float, depth_m: float) -> Tuple[float, float, float]:
        """@brief Back-project a pixel with known depth to a 3D point in camera frame.

        @param u Horizontal pixel coordinate.
        @param v Vertical pixel coordinate.
        @param depth_m Depth value in meters.
        @return 3D point (x, y, z) in the camera optical frame.
        """
        ray = self.camera_model.projectPixelTo3dRay((u, v))
        return (
            ray[0] * depth_m,
            ray[1] * depth_m,
            ray[2] * depth_m,
        )

    def _make_detection3d(
        self,
        header,
        class_id: str,
        score: float,
        xyz: Tuple[float, float, float],
    ) -> Detection3D:
        """@brief Construct a Detection3D message from a class label, score, and 3D position.

        @param header ROS message header.
        @param class_id Cone class name string.
        @param score Detection confidence score.
        @param xyz 3D position (x, y, z) in camera frame.
        @return Populated Detection3D message.
        """
        det3d = Detection3D()
        det3d.header = header

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = float(score)
        hypothesis.pose.pose.position.x = xyz[0]
        hypothesis.pose.pose.position.y = xyz[1]
        hypothesis.pose.pose.position.z = xyz[2]
        hypothesis.pose.pose.orientation.w = 1.0
        det3d.results.append(hypothesis)

        bbox = BoundingBox3D()
        bbox.center.position.x = xyz[0]
        bbox.center.position.y = xyz[1]
        bbox.center.position.z = xyz[2]
        bbox.center.orientation.w = 1.0
        bbox.size.x = self.default_box_size_m
        bbox.size.y = self.default_box_size_m
        bbox.size.z = self.default_box_size_m
        det3d.bbox = bbox

        return det3d

    def _on_synced(
        self, detections_msg: Detection2DArray, depth_msg: Image, info_msg: CameraInfo
    ) -> None:
        """@brief Callback triggered when 2D detections, depth image, and camera info are synchronized.

        For each 2D detection, samples depth at the bounding box center, back-projects
        to 3D, and publishes the resulting Detection3DArray.

        @param detections_msg Synchronized 2D detection array from YOLO.
        @param depth_msg Synchronized depth image.
        @param info_msg Synchronized camera intrinsics.
        """
        self._update_camera_model(info_msg)
        if not self.camera_info_ready:
            return

        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_m = self._depth_to_meters(depth_img, depth_msg.encoding)

        out = Detection3DArray()
        out.header = detections_msg.header
        if not out.header.frame_id:
            out.header.frame_id = info_msg.header.frame_id

        for det in detections_msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score

            u = int(round(det.bbox.center.position.x))
            v = int(round(det.bbox.center.position.y))
            if u < 0 or v < 0:
                continue
            if v >= depth_m.shape[0] or u >= depth_m.shape[1]:
                continue

            depth = self._median_depth(depth_m, u, v, self.depth_roi_px)
            if depth is None:
                continue

            xyz = self._project(u, v, depth)
            out.detections.append(self._make_detection3d(out.header, class_id, score, xyz))

        self.publisher.publish(out)


def main() -> None:
    """@brief Entry point for the cone depth localizer node."""
    rclpy.init()
    node = ConeDepthLocalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
