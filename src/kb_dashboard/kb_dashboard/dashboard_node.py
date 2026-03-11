#!/usr/bin/env python3
"""
kb_dashboard — Phone dashboard for kart telemetry and mission control.

Runs a WebSocket server alongside a ROS2 node. Any phone/browser on the
same network can open http://<orin-ip>:8080 to see live sensor values
and send commands (mission select, start/stop, EBS).
"""

import asyncio
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from kb_interfaces.msg import Frame
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32, String

from geometry_msgs.msg import Twist

from kb_dashboard.protocol import (
    DashboardState,
    decode_steering,
    decode_steering_raw,
    decode_speed,
    decode_accel,
    decode_braking,
    decode_throttle,
    decode_health,
)
from kb_dashboard.server import run_websocket_server


class DashboardNode(Node):
    def __init__(self, state: DashboardState):
        super().__init__("kb_dashboard")
        self.state = state
        self.declare_parameter("port", 8080)
        self.port = self.get_parameter("port").value

        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ESP32 → Orin telemetry
        self.create_subscription(
            Frame, "/esp32/heartbeat", self._on_heartbeat, qos_reliable
        )
        self.create_subscription(
            Frame, "/esp32/steering", self._on_esp_steering, qos_reliable
        )
        self.create_subscription(
            Frame, "/esp32/speed", self._on_esp_speed, qos_reliable
        )
        self.create_subscription(
            Frame, "/esp32/acceleration", self._on_esp_accel, qos_reliable
        )
        self.create_subscription(
            Frame, "/esp32/throttle", self._on_esp_throttle, qos_reliable
        )
        self.create_subscription(
            Frame, "/esp32/braking", self._on_esp_braking, qos_reliable
        )
        self.create_subscription(
            Frame, "/esp32/health", self._on_esp_health, qos_reliable
        )

        # ZED2 IMU — uses BEST_EFFORT to match the ZED ROS2 wrapper's default QoS
        self.create_subscription(
            Imu, "/zed/zed_node/imu/data", self._on_zed_imu, qos_best_effort
        )

        # Orin → ESP32 commands (to show what we're sending)
        self.create_subscription(
            Frame, "/orin/throttle", self._on_orin_throttle, qos_reliable
        )
        self.create_subscription(
            Frame, "/orin/brake", self._on_orin_brake, qos_reliable
        )
        self.create_subscription(
            Frame, "/orin/steering", self._on_orin_steering, qos_reliable
        )

        # YOLO FPS
        self.create_subscription(
            Float32, "/perception/yolo/fps", self._on_yolo_fps, qos_reliable
        )

        # HUD image stream (JPEG bytes stored for WebSocket binary broadcast)
        self._bridge = CvBridge()
        self._hud_jpeg: bytes | None = None
        self.create_subscription(Image, "/perception/hud", self._on_hud_image, 1)

        # Publishers for mission commands
        self.mission_pub = self.create_publisher(String, "/dashboard/mission", 10)

        # Publisher for manual remote control (Twist for now)
        self.manual_cmd_pub = self.create_publisher(Twist, "/kart/cmd_vel_manual", 10)

        self.get_logger().info(f"Dashboard node started, web UI on port {self.port}")

    def _on_heartbeat(self, msg: Frame):
        self.state.heartbeat()

    def _on_esp_steering(self, msg: Frame):
        p = list(msg.payload)
        angle_rad, raw_encoder = decode_steering_raw(p)
        self.state.update("esp32_steering_rad", angle_rad)
        if raw_encoder:
            self.state.update("esp32_steering_raw", raw_encoder)
            now = self.get_clock().now().nanoseconds
            if (
                not hasattr(self, "_last_steer_log")
                or now - self._last_steer_log > 500_000_000
            ):
                self._last_steer_log = now
                self.get_logger().warn(
                    f"STEER deg={angle_rad * 180 / 3.14159:.1f}  raw={raw_encoder}"
                )

    def _on_esp_speed(self, msg: Frame):
        if msg.payload:
            self.state.update("esp32_speed", decode_speed(list(msg.payload)))

    def _on_esp_accel(self, msg: Frame):
        p = list(msg.payload)
        if p:
            lat, lon = decode_accel(p)
            self.state.update("esp32_accel_lat", lat)
            self.state.update("esp32_accel_lon", lon)

    def _on_esp_throttle(self, msg: Frame):
        self.state.update("esp32_throttle", decode_throttle(list(msg.payload)))

    def _on_esp_braking(self, msg: Frame):
        self.state.update("esp32_braking", decode_braking(list(msg.payload)))

    def _on_zed_imu(self, msg: Imu):
        # ZED2 ROS2 wrapper uses REP-103: x=forward, y=left, z=up
        self.state.update("esp32_accel_lon", msg.linear_acceleration.x)
        self.state.update(
            "esp32_accel_lat", -msg.linear_acceleration.y
        )  # flip: y=left → positive=right

    def _on_esp_health(self, msg: Frame):
        fields = decode_health(list(msg.payload))
        for k, v in fields.items():
            self.state.update(k, v)

    def _on_orin_throttle(self, msg: Frame):
        self.state.update("orin_cmd_throttle", decode_throttle(list(msg.payload)))

    def _on_orin_brake(self, msg: Frame):
        self.state.update("orin_cmd_brake", decode_throttle(list(msg.payload)))

    def _on_orin_steering(self, msg: Frame):
        self.state.update("orin_cmd_steering_rad", decode_steering(list(msg.payload)))

    def _on_yolo_fps(self, msg: Float32):
        self.state.update("yolo_fps", round(msg.data, 1))

    def _on_hud_image(self, msg: Image):
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        self._hud_jpeg = buf.tobytes()

    def get_hud_jpeg(self) -> bytes | None:
        return self._hud_jpeg

    def publish_mission(self, mission: str):
        msg = String()
        msg.data = mission
        self.mission_pub.publish(msg)
        self.get_logger().info(f"Mission set: {mission}")

    def publish_manual_control(
        self, steer: float, steer_type: str, throttle: float, brake: float
    ):
        """Publish remote control from the dashboard."""
        # Optional: Print/log if they chose PWM, since Twist doesn't natively support it.
        # But we can map "steer" straight to cmd.angular.z for now.
        cmd = Twist()

        # We assume gamepad provides:
        #   steer: -1.0 (left) to 1.0 (right)
        #   throttle: 0.0 to 1.0
        #   brake: 0.0 to 1.0

        # Combine throttle and brake into linear.x
        # For cmd_vel_bridge, positive is throttle, negative is brake.
        speed_cmd = throttle - brake

        # Assuming max speed is handled downstream by cmd_vel_bridge.
        # But we'll scale it to some nominal 'max speed' so it's not strictly 1.0 m/s
        # In cmd_vel_bridge_node: max_speed default is 5.0
        NOMINAL_MAX_SPEED = 5.0
        NOMINAL_MAX_STEER = 0.5  # radians

        cmd.linear.x = speed_cmd * NOMINAL_MAX_SPEED
        cmd.angular.z = (
            -steer * NOMINAL_MAX_STEER
        )  # Typically left stick left (-1) means positive angular.z

        self.manual_cmd_pub.publish(cmd)

        # Log periodically or only if state changes?
        now = self.get_clock().now().nanoseconds
        if (
            not hasattr(self, "_last_manual_log")
            or now - self._last_manual_log > 1_000_000_000
        ):
            self._last_manual_log = now
            self.get_logger().info(
                f"Manual Ctrl: steer={steer:.2f}({steer_type}), thr={throttle:.2f}, brk={brake:.2f}"
            )


# ── Entrypoint ─────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    state = DashboardState()
    node = DashboardNode(state)

    # Run ROS spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run the async web server in the main thread
    try:
        asyncio.run(run_websocket_server(state, node, node.port))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
