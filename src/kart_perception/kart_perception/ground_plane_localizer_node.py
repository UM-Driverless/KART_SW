#!/usr/bin/env python3
"""Republish ZED Object Detection cones with camera pitch/roll undone via the
fused IMU orientation.

Subscribes to:
  - /zed/zed_node/imu/data            (sensor_msgs/Imu, BEST_EFFORT)
  - /zed/zed_node/obj_det/objects     (zed_interfaces/ObjectsStamped)

Publishes:
  - /perception/cones_3d_ground         (vision_msgs/Detection3DArray)
  - /perception/cones_3d_ground_markers (visualization_msgs/MarkerArray)

The fused quaternion from the ZED's onboard sensor fusion is decomposed into
yaw-pitch-roll. Yaw is the kart's heading and is preserved. Pitch and roll are
the camera's tilt relative to gravity and are undone — every detected cone
position is rotated by the inverse of (pitch, roll). Result: cone positions in
a kart-level frame that doesn't lurch when the kart pitches under braking.

Known weakness: the fused orientation is biased during sustained linear
acceleration. Worst-case is hard braking, where the estimated "down" tilts
toward the deceleration vector. Ship v1 against the stock fusion and measure
the bias on real braking events before optimizing.
"""
from collections import deque
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from kart_perception.zed_od_utils import HAS_ZED_INTERFACES, zed_objects_to_det3d

if HAS_ZED_INTERFACES:
    from zed_interfaces.msg import ObjectsStamped


def _stamp_to_ns(stamp) -> int:
    """ROS time → integer nanoseconds, for closest-timestamp lookups."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _class_color(class_id: str) -> Tuple[float, float, float]:
    """RGB color per cone class (cyan-shifted from cone_marker_viz_3d so the
    ground-corrected markers are distinguishable in RViz when both topics are
    rendered side-by-side)."""
    if class_id == "blue_cone":
        return (0.4, 0.8, 1.0)
    if class_id == "yellow_cone":
        return (0.6, 1.0, 0.8)
    if class_id == "orange_cone":
        return (1.0, 0.7, 0.4)
    if class_id == "large_orange_cone":
        return (1.0, 0.5, 0.4)
    return (0.7, 0.9, 0.9)


class GroundPlaneLocalizerNode(Node):
    """Subscribe to ZED OD + ZED IMU, undo camera pitch/roll, republish cones."""

    def __init__(self) -> None:
        super().__init__("ground_plane_localizer")

        self.declare_parameter("imu_topic", "/zed/zed_node/imu/data")
        self.declare_parameter("objects_topic", "/zed/zed_node/obj_det/objects")
        self.declare_parameter("output_topic", "/perception/cones_3d_ground")
        self.declare_parameter("markers_topic", "/perception/cones_3d_ground_markers")
        self.declare_parameter("imu_match_window_ms", 50)
        self.declare_parameter("invert_correction", False)
        self.declare_parameter("gravity_axis", [0.0, 0.0, 1.0])  # world up; REP-103 default
        self.declare_parameter("cone_scale_m", 0.3)

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.objects_topic = str(self.get_parameter("objects_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.markers_topic = str(self.get_parameter("markers_topic").value)
        self.imu_match_window_ns = int(self.get_parameter("imu_match_window_ms").value) * 1_000_000
        self.invert_correction = bool(self.get_parameter("invert_correction").value)
        gravity_axis = np.array(list(self.get_parameter("gravity_axis").value), dtype=np.float64)
        n = np.linalg.norm(gravity_axis)
        self.gravity_axis = gravity_axis / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
        self.cone_scale_m = float(self.get_parameter("cone_scale_m").value)

        # IMU runs ~200 Hz; keep ~1 s of samples for closest-timestamp lookup.
        self._imu_buf: deque = deque(maxlen=400)
        self._dropped_no_imu = 0

        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.det_pub = self.create_publisher(Detection3DArray, self.output_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)

        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos_best_effort)

        if not HAS_ZED_INTERFACES:
            self.get_logger().error(
                "zed_interfaces not available — install zed-ros2-interfaces. "
                "ground_plane_localizer cannot run without it."
            )
            return
        self.create_subscription(ObjectsStamped, self.objects_topic, self._on_objects, 10)

        self.get_logger().info(
            f"ground_plane_localizer up: imu={self.imu_topic} → "
            f"objects={self.objects_topic} → {self.output_topic} "
            f"(invert_correction={self.invert_correction}, "
            f"gravity_axis={self.gravity_axis.tolist()})"
        )

    def _on_imu(self, msg: Imu) -> None:
        ts_ns = _stamp_to_ns(msg.header.stamp)
        q = (
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )
        self._imu_buf.append((ts_ns, q))

    def _nearest_imu_quat(self, target_ns: int) -> Optional[Tuple[float, float, float, float]]:
        """Closest-timestamp lookup. Returns None if no sample is within the
        match window."""
        if not self._imu_buf:
            return None
        best_dt = None
        best_q = None
        for ts_ns, q in self._imu_buf:
            dt = abs(ts_ns - target_ns)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_q = q
        if best_dt is None or best_dt > self.imu_match_window_ns:
            return None
        return best_q

    def _build_correction(self, q_xyzw: Tuple[float, float, float, float]) -> Rotation:
        """Strip the yaw (rotation about gravity) component of the IMU orientation
        and return what's left — the pitch+roll-only rotation. Applied to
        camera-frame cone positions, this rotates them into a kart-level frame
        with the kart's heading preserved.

        Math: q (camera attitude in world) = q_yaw * q_pitch_roll. We want
        p_kart = q_yaw.inv() @ q @ p_cam = q_pitch_roll @ p_cam. The "swing-twist"
        decomposition splits q into a "twist" (rotation about a chosen axis,
        gravity in our case) and a "swing" (the residual). q_yaw = twist,
        q_pitch_roll = swing. We return the swing.

        Assumption: the IMU's orientation field is expressed in a frame where
        gravity points along ``-gravity_axis`` (i.e., world +Z by REP-103
        default). If the workshop sanity check shows the correction is
        backwards, set ``invert_correction=true``.
        """
        full_rot = Rotation.from_quat(list(q_xyzw))
        # Twist about gravity_axis: project the imaginary part of the
        # quaternion onto gravity_axis, keep the real part, renormalize.
        qx, qy, qz, qw = q_xyzw
        v = np.array([qx, qy, qz], dtype=np.float64)
        v_proj = float(np.dot(v, self.gravity_axis)) * self.gravity_axis
        twist_unnorm = np.array([v_proj[0], v_proj[1], v_proj[2], qw])
        norm = float(np.linalg.norm(twist_unnorm))
        if norm < 1e-9:
            # Degenerate (q is a 180° rotation about an axis perpendicular to
            # gravity); fall back to no yaw.
            twist_quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            twist_quat = twist_unnorm / norm
        yaw_rot = Rotation.from_quat(twist_quat)
        swing_rot = yaw_rot.inv() * full_rot  # pitch+roll only
        if self.invert_correction:
            swing_rot = swing_rot.inv()
        return swing_rot

    def _on_objects(self, msg) -> None:
        ts_ns = _stamp_to_ns(msg.header.stamp)
        q = self._nearest_imu_quat(ts_ns)
        if q is None:
            self._dropped_no_imu += 1
            if self._dropped_no_imu % 30 == 1:
                self.get_logger().warn(
                    f"No IMU sample within {self.imu_match_window_ns // 1_000_000} ms "
                    f"of detection (dropped={self._dropped_no_imu}). "
                    "Check the ZED wrapper is publishing imu/data."
                )
            return

        correction = self._build_correction(q)

        # Reuse existing Detection3DArray builder, then rewrite positions.
        det_array = zed_objects_to_det3d(msg)
        for det in det_array.detections:
            if not det.results:
                continue
            p_cam = np.array(
                [
                    det.bbox.center.position.x,
                    det.bbox.center.position.y,
                    det.bbox.center.position.z,
                ],
                dtype=np.float64,
            )
            p_ground = correction.apply(p_cam)
            det.bbox.center.position.x = float(p_ground[0])
            det.bbox.center.position.y = float(p_ground[1])
            det.bbox.center.position.z = float(p_ground[2])
            det.results[0].pose.pose.position.x = float(p_ground[0])
            det.results[0].pose.pose.position.y = float(p_ground[1])
            det.results[0].pose.pose.position.z = float(p_ground[2])

        self.det_pub.publish(det_array)
        self.marker_pub.publish(self._make_markers(det_array))

    def _make_markers(self, msg: Detection3DArray) -> MarkerArray:
        markers = MarkerArray()
        marker_id = 0
        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            color = _class_color(class_id)
            p = det.bbox.center.position

            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = "cones_3d_ground"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = p.x
            sphere.pose.position.y = p.y
            sphere.pose.position.z = p.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = self.cone_scale_m
            sphere.color.r, sphere.color.g, sphere.color.b = color
            sphere.color.a = 0.9
            markers.markers.append(sphere)

            label = Marker()
            label.header = msg.header
            label.ns = "cones_3d_ground_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = p.x
            label.pose.position.y = p.y
            label.pose.position.z = p.z + self.cone_scale_m
            label.pose.orientation.w = 1.0
            label.scale.z = max(0.2, self.cone_scale_m * 0.6)
            label.color.r, label.color.g, label.color.b = color
            label.color.a = 1.0
            label.text = f"{class_id} {score:.2f}"
            markers.markers.append(label)

        return markers


def main() -> None:
    rclpy.init()
    node = GroundPlaneLocalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
