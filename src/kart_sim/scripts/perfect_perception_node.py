#!/usr/bin/env python3
"""Perfect perception node: reads cone positions from the SDF world
and publishes them as Detection3DArray relative to the kart's camera frame.

This bypasses YOLO entirely — useful for testing the control loop before
the vision pipeline works on Gazebo-rendered images.
"""
import math
import re
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


# Cone world positions extracted from the SDF (name -> (x, y, z, class_id))
# These are populated at startup by parsing the world file.
CONE_CLASSES = {
    "blue": "blue_cone",
    "yellow": "yellow_cone",
    "orange": "orange_cone",
}


def _class_from_name(name: str) -> str:
    """Determine cone class_id from model name prefix."""
    if name.startswith("blue"):
        return "blue_cone"
    elif name.startswith("yellow"):
        return "yellow_cone"
    elif name.startswith("orange"):
        return "orange_cone"
    return ""


def parse_cones_from_sdf(sdf_path: str) -> List[Dict]:
    """Parse cone model positions and colors from the world SDF file.

    Supports both inline <model> definitions and <include> tags.
    """
    cones = []
    with open(sdf_path, "r") as f:
        content = f.read()

    # Match inline cone models: <model name="blue_rs_0">...<pose>...</pose>
    inline_pat = r'<model name="((?:blue|yellow|orange)_\w+)".*?<pose>([\d\s.eE+\-]+)</pose>'
    for match in re.finditer(inline_pat, content, re.DOTALL):
        name = match.group(1)
        pose_str = match.group(2).strip().split()
        x, y, z = float(pose_str[0]), float(pose_str[1]), float(pose_str[2])
        class_id = _class_from_name(name)
        if class_id:
            cones.append({"name": name, "x": x, "y": y, "z": z, "class_id": class_id})

    # Match <include> tags: <include><uri>model://cone_*</uri><name>...</name><pose>...</pose></include>
    include_pat = (
        r'<include>\s*'
        r'<uri>model://cone_\w+</uri>\s*'
        r'<name>((?:blue|yellow|orange)_\w+)</name>\s*'
        r'<pose>([\d\s.eE+\-]+)</pose>\s*'
        r'</include>'
    )
    for match in re.finditer(include_pat, content, re.DOTALL):
        name = match.group(1)
        pose_str = match.group(2).strip().split()
        x, y, z = float(pose_str[0]), float(pose_str[1]), float(pose_str[2])
        class_id = _class_from_name(name)
        if class_id:
            cones.append({"name": name, "x": x, "y": y, "z": z, "class_id": class_id})

    return cones


class PerfectPerceptionNode(Node):
    def __init__(self):
        super().__init__("perfect_perception")

        self.declare_parameter("world_sdf", "")
        self.declare_parameter("output_topic", "/perception/cones_3d")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("fov_deg", 70.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("camera_frame", "camera_link")
        # Camera offset from base_link (matching the kart model)
        self.declare_parameter("camera_x_offset", 0.55)
        self.declare_parameter("camera_z_offset", 0.47)
        # Kart initial world pose (odom is relative to this)
        self.declare_parameter("kart_start_x", 20.0)
        self.declare_parameter("kart_start_y", 0.0)
        self.declare_parameter("kart_start_yaw", 1.5708)

        sdf_path = str(self.get_parameter("world_sdf").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.fov_rad = math.radians(float(self.get_parameter("fov_deg").value))
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.cam_x = float(self.get_parameter("camera_x_offset").value)
        self.cam_z = float(self.get_parameter("camera_z_offset").value)
        self.start_x = float(self.get_parameter("kart_start_x").value)
        self.start_y = float(self.get_parameter("kart_start_y").value)
        self.start_yaw = float(self.get_parameter("kart_start_yaw").value)

        # Parse cones from SDF
        if sdf_path:
            self.cones = parse_cones_from_sdf(sdf_path)
            self.get_logger().info(f"Loaded {len(self.cones)} cones from {sdf_path}")
        else:
            self.cones = []
            self.get_logger().warn("No world_sdf parameter set — no cones loaded")

        # Publisher
        self.pub = self.create_publisher(Detection3DArray, self.output_topic, 10)

        # Also publish a fake CameraInfo so the depth localizer is happy
        self.cam_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        # TF broadcaster for camera frame
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to ground-truth odometry (world frame, no drift)
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/model/kart/odom_gt", self._on_odom, odom_qos
        )

        # Current kart pose
        self.kart_x = 0.0
        self.kart_y = 0.0
        self.kart_yaw = 0.0
        self.got_odom = False

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)

    def _on_odom(self, msg: Odometry):
        # Ground-truth odom is already in world frame
        self.kart_x = msg.pose.pose.position.x
        self.kart_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.kart_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.got_odom = True

    def _publish(self):
        if not self.got_odom or not self.cones:
            self.get_logger().info(
                f"Skipping: got_odom={self.got_odom} cones={len(self.cones)}",
                throttle_duration_sec=2.0,
            )
            # Still publish empty array so controller knows we're alive
            out = Detection3DArray()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.camera_frame
            self.pub.publish(out)
            return

        now = self.get_clock().now().to_msg()

        # Camera position in world frame
        cos_yaw = math.cos(self.kart_yaw)
        sin_yaw = math.sin(self.kart_yaw)
        cam_world_x = self.kart_x + self.cam_x * cos_yaw
        cam_world_y = self.kart_y + self.cam_x * sin_yaw

        out = Detection3DArray()
        out.header.stamp = now
        out.header.frame_id = self.camera_frame

        half_fov = self.fov_rad / 2.0

        for cone in self.cones:
            # Vector from camera to cone in world frame
            dx = cone["x"] - cam_world_x
            dy = cone["y"] - cam_world_y

            # Transform to camera frame (camera looks along +X in kart frame)
            # Camera frame: X=forward, Y=left, Z=up
            cx = dx * cos_yaw + dy * sin_yaw
            cy = -dx * sin_yaw + dy * cos_yaw

            dist = math.sqrt(cx * cx + cy * cy)
            if dist > self.max_range or dist < 0.5:
                continue

            # Check FOV (angle from forward axis)
            angle = math.atan2(cy, cx)
            if abs(angle) > half_fov:
                continue

            # Publish in camera optical frame (Z=forward, X=right, Y=down)
            # to match what cone_depth_localizer outputs.
            # cone_follower expects this convention and converts internally.
            det = Detection3D()
            det.header = out.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cone["class_id"]
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = -cy   # optical X = right = -left
            hyp.pose.pose.position.y = -(cone["z"] - self.cam_z)  # optical Y = down
            hyp.pose.pose.position.z = cx     # optical Z = forward
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)

            bbox = BoundingBox3D()
            bbox.center.position.x = -cy
            bbox.center.position.y = -(cone["z"] - self.cam_z)
            bbox.center.position.z = cx
            bbox.center.orientation.w = 1.0
            bbox.size.x = 0.25
            bbox.size.y = 0.25
            bbox.size.z = 0.325
            det.bbox = bbox

            out.detections.append(det)

        self.pub.publish(out)
        self.get_logger().info(
            f"Published {len(out.detections)} cones  kart=({self.kart_x:.1f},{self.kart_y:.1f}) yaw={math.degrees(self.kart_yaw):.0f}°",
            throttle_duration_sec=2.0,
        )

        # Broadcast TF: odom -> base_link -> camera_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.kart_x
        t.transform.translation.y = self.kart_y
        t.transform.translation.z = 0.15
        # Quaternion from yaw
        t.transform.rotation.z = math.sin(self.kart_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.kart_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "base_link"
        t2.child_frame_id = self.camera_frame
        t2.transform.translation.x = self.cam_x
        t2.transform.translation.z = self.cam_z - 0.15
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)


def main():
    rclpy.init()
    node = PerfectPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
