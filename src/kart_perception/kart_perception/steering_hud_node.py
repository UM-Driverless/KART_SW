#!/usr/bin/env python3
"""Steering HUD overlay node.

Composites steering visualizations onto the YOLO-annotated image:
- Cone highlights with distance labels on nearest blue/yellow cones
- Midpoint crosshair between the target cones
- Steering arrow from bottom-center toward the midpoint
- Horizontal steering gauge at the bottom
- Text overlay with steering angle, speed, and FPS
- Status indicator (BLUE + YLW / BLUE ONLY / YLW ONLY / NO CONES)

Architecture: direct subscriber on annotated image (no sync). Cones, cmd_vel,
and camera_info are cached from standalone subscribers. This ensures the HUD
publishes at the same rate as YOLO with zero dropped frames.
"""
import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection3DArray

from kart_perception.zed_od_utils import HAS_ZED_INTERFACES, zed_objects_to_det3d
if HAS_ZED_INTERFACES:
    from zed_interfaces.msg import ObjectsStamped

# Colors (BGR)
BLUE_CONE_COLOR = (255, 150, 0)
YELLOW_CONE_COLOR = (0, 230, 255)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
RED = (0, 0, 255)
DARK_BG = (30, 30, 30)


class SteeringHudNode(Node):
    """@brief ROS2 node that composites a steering HUD overlay onto YOLO-annotated images.

    Draws nearest cone highlights, midpoint crosshair, steering arrow, steering gauge,
    text overlay (steering angle, speed, FPS), and cone detection status indicator.
    """

    def __init__(self):
        """@brief Initialize the steering HUD node.

        Declares parameters for input/output topics, half track width, lookahead
        distance, and cone staleness timeout. Subscribes to annotated images,
        3D cones, cmd_vel, and camera info.
        """
        super().__init__("steering_hud")

        self.declare_parameter("annotated_topic", "/perception/yolo/annotated")
        self.declare_parameter("cones_3d_topic", "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("output_topic", "/perception/hud")
        self.declare_parameter("half_track_width", 1.5)
        self.declare_parameter("lookahead_max", 15.0)
        self.declare_parameter("cone_staleness", 0.5)

        self.half_track_width = float(self.get_parameter("half_track_width").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.cone_staleness = float(self.get_parameter("cone_staleness").value)

        self.bridge = CvBridge()
        self.latest_cmd = Twist()
        self.latest_cones = None
        self.latest_cones_time = 0.0
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_ready = False
        self._fps_prev_time = time.monotonic()
        self._fps = 0.0
        self._yolo_fps = 0.0  # YOLO inference FPS from /perception/yolo/fps

        self.pub = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), 1
        )

        # Main driver: every annotated frame gets HUD overlays
        self.create_subscription(
            Image,
            str(self.get_parameter("annotated_topic").value),
            self._on_image,
            1,
        )

        # Cache latest cones, cmd_vel, camera_info
        self.create_subscription(
            Detection3DArray,
            str(self.get_parameter("cones_3d_topic").value),
            self._on_cones,
            1,
        )
        # Also subscribe to ZED SDK ObjectsStamped for built-in OD mode
        if HAS_ZED_INTERFACES:
            self.create_subscription(
                ObjectsStamped,
                "/zed/zed_node/obj_det/objects",
                lambda msg: self._on_cones(zed_objects_to_det3d(msg)),
                1,
            )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            1,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            1,
        )
        self.create_subscription(
            Float32, "/perception/yolo/fps", self._on_yolo_fps, 10
        )

        self.get_logger().info("SteeringHudNode ready")

    # ---- Callbacks ----

    def _on_yolo_fps(self, msg: Float32):
        """@brief Cache the latest YOLO inference FPS value.

        @param msg Float32 message with YOLO inference rate in Hz.
        """
        self._yolo_fps = msg.data

    def _on_cmd_vel(self, msg: Twist):
        """@brief Cache the latest velocity command for HUD display.

        @param msg Twist message with linear.x (speed) and angular.z (steering).
        """
        self.latest_cmd = msg

    def _on_cones(self, msg: Detection3DArray):
        """@brief Cache the latest 3D cone detections and record arrival time.

        @param msg Array of 3D cone detections.
        """
        self.latest_cones = msg
        self.latest_cones_time = time.monotonic()

    def _on_camera_info(self, msg: CameraInfo):
        """@brief Extract and cache camera intrinsics from camera info (once).

        @param msg Camera info message containing the intrinsic matrix K.
        """
        if not self.camera_info_ready:
            K = msg.k
            self.fx, self.fy = K[0], K[4]
            self.cx, self.cy = K[2], K[5]
            self.camera_info_ready = True
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f}"
            )

    def _on_image(self, img_msg: Image):
        """@brief Main callback triggered by each annotated YOLO image.

        Computes HUD overlays (cone highlights, midpoint, steering arrow, gauge,
        text, status) and publishes the composited image.

        @param img_msg Annotated image from the YOLO detector.
        """
        # FPS (EMA)
        now = time.monotonic()
        dt = now - self._fps_prev_time
        self._fps_prev_time = now
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 / dt

        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        h, w = img.shape[:2]

        # Use cached cones if fresh
        cones_fresh = (now - self.latest_cones_time) < self.cone_staleness
        nearest_blue = None
        nearest_yellow = None
        status = "NO CONES"

        if cones_fresh and self.latest_cones is not None:
            nearest_blue, nearest_yellow = self._find_nearest_cones(self.latest_cones)
            if nearest_blue and nearest_yellow:
                status = "BLUE + YLW"
            elif nearest_blue:
                status = "BLUE ONLY"
            elif nearest_yellow:
                status = "YLW ONLY"
        elif self.latest_cones is None:
            status = "NO 3D"
        else:
            status = "STALE"

        # Compute midpoint in optical frame
        mid_opt = None
        if nearest_blue and nearest_yellow:
            bx, by, bz, _ = nearest_blue
            yx, yy, yz, _ = nearest_yellow
            mid_opt = ((bx + yx) / 2.0, (by + yy) / 2.0, (bz + yz) / 2.0)
        elif nearest_blue:
            bx, by, bz, _ = nearest_blue
            mid_opt = (bx + self.half_track_width, by, bz)
        elif nearest_yellow:
            yx, yy, yz, _ = nearest_yellow
            mid_opt = (yx - self.half_track_width, yy, yz)

        if self.camera_info_ready:
            if nearest_blue:
                self._draw_cone(img, nearest_blue, BLUE_CONE_COLOR, "B")
            if nearest_yellow:
                self._draw_cone(img, nearest_yellow, YELLOW_CONE_COLOR, "Y")
            if mid_opt is not None and mid_opt[2] > 0:
                mu, mv = self._project(mid_opt[0], mid_opt[1], mid_opt[2])
                mu, mv = int(mu), int(mv)
                size = 15
                cv2.line(img, (mu - size, mv), (mu + size, mv), GREEN, 2)
                cv2.line(img, (mu, mv - size), (mu, mv + size), GREEN, 2)
                cv2.circle(img, (mu, mv), 4, GREEN, -1)
                cv2.arrowedLine(img, (w // 2, h - 30), (mu, mv), GREEN, 2, tipLength=0.04)

        self._draw_gauge(img, self.latest_cmd.angular.z)
        self._draw_text_overlay(img, self.latest_cmd)
        self._draw_status(img, status)

        out_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out_msg.header = img_msg.header
        self.pub.publish(out_msg)

    # ---- Logic ----

    def _find_nearest_cones(self, cones_msg):
        """@brief Find the nearest blue and yellow cones from 3D detections.

        Filters out cones behind the kart or beyond lookahead distance.

        @param cones_msg Detection3DArray with 3D cone positions.
        @return Tuple of (nearest_blue, nearest_yellow), each as (x, y, z, dist) or None.
        """
        nearest_blue = None
        nearest_yellow = None
        min_blue_dist = float("inf")
        min_yellow_dist = float("inf")

        for det in cones_msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos = det.results[0].pose.pose.position
            fwd = pos.z
            left = -pos.x
            if fwd < 0.5:
                continue
            dist = math.sqrt(fwd**2 + left**2)
            if dist > self.lookahead_max:
                continue
            if class_id == "blue_cone" and dist < min_blue_dist:
                min_blue_dist = dist
                nearest_blue = (pos.x, pos.y, pos.z, dist)
            elif class_id == "yellow_cone" and dist < min_yellow_dist:
                min_yellow_dist = dist
                nearest_yellow = (pos.x, pos.y, pos.z, dist)

        return nearest_blue, nearest_yellow

    # ---- Drawing helpers ----

    def _project(self, x, y, z):
        """@brief Project a 3D point in camera frame to 2D pixel coordinates.

        @param x X coordinate in camera optical frame.
        @param y Y coordinate in camera optical frame.
        @param z Z coordinate (depth) in camera optical frame.
        @return Tuple (u, v) pixel coordinates.
        """
        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return u, v

    def _draw_cone(self, img, cone_data, color, label):
        """@brief Draw a cone highlight circle and distance label on the image.

        @param img BGR image to draw on (modified in place).
        @param cone_data Tuple (x, y, z, dist) of the cone in camera frame.
        @param color BGR color tuple for the circle and text.
        @param label Short label prefix (e.g. "B" for blue, "Y" for yellow).
        """
        x, y, z, dist = cone_data
        if z <= 0 or not self.camera_info_ready:
            return
        u, v = self._project(x, y, z)
        u, v = int(u), int(v)
        h, w = img.shape[:2]
        if 0 <= u < w and 0 <= v < h:
            cv2.circle(img, (u, v), 20, color, 3)
            text = f"{label} {dist:.1f}m"
            cv2.putText(img, text, (u + 24, v + 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2, cv2.LINE_AA)

    def _draw_gauge(self, img, steer_rad):
        """@brief Draw a horizontal steering gauge bar at the bottom of the image.

        @param img BGR image to draw on (modified in place).
        @param steer_rad Steering angle in radians.
        """
        h, w = img.shape[:2]
        gauge_y = h - 12
        gauge_w = min(300, w - 40)
        gauge_x0 = (w - gauge_w) // 2
        cv2.rectangle(img, (gauge_x0, gauge_y - 6), (gauge_x0 + gauge_w, gauge_y + 6),
                      DARK_BG, -1)
        center_x = gauge_x0 + gauge_w // 2
        cv2.line(img, (center_x, gauge_y - 8), (center_x, gauge_y + 8), WHITE, 1)
        max_steer = 0.5
        frac = max(-1.0, min(1.0, -steer_rad / max_steer))
        ind_x = int(center_x + frac * (gauge_w // 2))
        cv2.circle(img, (ind_x, gauge_y), 7, RED, -1)
        cv2.circle(img, (ind_x, gauge_y), 7, WHITE, 1)

    def _draw_text_overlay(self, img, cmd: Twist):
        """@brief Draw steering angle, speed, and FPS text in the top-left corner.

        @param img BGR image to draw on (modified in place).
        @param cmd Current velocity command with steering and speed values.
        """
        steer_deg = math.degrees(cmd.angular.z)
        speed = cmd.linear.x
        lines = [
            f"Steer: {steer_deg:+.1f} deg",
            f"Speed: {speed:.1f} m/s",
            f"YOLO: {self._yolo_fps:.1f} Hz",
        ]
        y0 = 25
        for i, line in enumerate(lines):
            y = y0 + i * 22
            (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(img, (8, y - th - 4), (16 + tw, y + 4), DARK_BG, -1)
            cv2.putText(img, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, WHITE, 2, cv2.LINE_AA)

    def _draw_status(self, img, status):
        """@brief Draw the cone detection status indicator in the bottom-left corner.

        @param img BGR image to draw on (modified in place).
        @param status Status string (e.g. "BLUE + YLW", "NO CONES", "STALE").
        """
        h = img.shape[0]
        y = h - 20
        (tw, th), _ = cv2.getTextSize(status, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(img, (8, y - th - 4), (16 + tw, y + 4), DARK_BG, -1)
        color = GREEN if "+" in status else YELLOW_CONE_COLOR if "ONLY" in status else RED
        cv2.putText(img, status, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2, cv2.LINE_AA)


def main():
    """@brief Entry point for the steering HUD node."""
    rclpy.init()
    node = SteeringHudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
