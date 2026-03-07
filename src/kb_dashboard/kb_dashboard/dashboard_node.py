#!/usr/bin/env python3
"""
kb_dashboard — Phone dashboard for kart telemetry and mission control.

Runs a WebSocket server alongside a ROS2 node. Any phone/browser on the
same network can open http://<orin-ip>:8080 to see live sensor values
and send commands (mission select, start/stop, EBS).
"""

import asyncio
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from kb_interfaces.msg import Frame
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from kb_dashboard.protocol import DashboardState, decode_steering, decode_u8, decode_health
from kb_dashboard.server import run_websocket_server


class DashboardNode(Node):
    def __init__(self, state: DashboardState):
        super().__init__("kb_dashboard")
        self.state = state
        self.declare_parameter("port", 8080)
        self.port = self.get_parameter("port").value

        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ESP32 → Orin telemetry
        self.create_subscription(Frame, "/esp32/heartbeat", self._on_heartbeat, qos_reliable)
        self.create_subscription(Frame, "/esp32/steering", self._on_esp_steering, qos_reliable)
        self.create_subscription(Frame, "/esp32/speed", self._on_esp_speed, qos_reliable)
        self.create_subscription(Frame, "/esp32/acceleration", self._on_esp_accel, qos_reliable)
        self.create_subscription(Frame, "/esp32/throttle", self._on_esp_throttle, qos_reliable)
        self.create_subscription(Frame, "/esp32/braking", self._on_esp_braking, qos_reliable)
        self.create_subscription(Frame, "/esp32/health", self._on_esp_health, qos_reliable)

        # ZED2 IMU — uses BEST_EFFORT to match the ZED ROS2 wrapper's default QoS
        self.create_subscription(Imu, "/zed/zed_node/imu/data", self._on_zed_imu, qos_best_effort)

        # Orin → ESP32 commands (to show what we're sending)
        self.create_subscription(Frame, "/orin/throttle", self._on_orin_throttle, qos_reliable)
        self.create_subscription(Frame, "/orin/brake", self._on_orin_brake, qos_reliable)
        self.create_subscription(Frame, "/orin/steering", self._on_orin_steering, qos_reliable)

        # Publishers for mission commands
        self.mission_pub = self.create_publisher(String, "/dashboard/mission", 10)

        self.get_logger().info(f"Dashboard node started, web UI on port {self.port}")

    def _on_heartbeat(self, msg: Frame):
        self.state.heartbeat()

    def _on_esp_steering(self, msg: Frame):
        p = list(msg.payload)
        rad = decode_steering(p)
        self.state.update("esp32_steering_rad", rad)
        if len(p) >= 4:
            raw = (p[2] << 8) | p[3]
            self.state.update("esp32_steering_raw", raw)
            print(f"STEER deg={rad*180/3.14159:.1f}  raw={raw}", flush=True)

    def _on_esp_speed(self, msg: Frame):
        if msg.payload:
            self.state.update("esp32_speed", decode_steering(list(msg.payload)))

    def _on_esp_accel(self, msg: Frame):
        # Acceleration frame: 4 bytes = lat(int16) + lon(int16), both rad*1000 encoding
        p = list(msg.payload)
        if len(p) >= 4:
            self.state.update("esp32_accel_lat", decode_steering(p[0:2]))
            self.state.update("esp32_accel_lon", decode_steering(p[2:4]))
        elif len(p) >= 2:
            self.state.update("esp32_accel_lon", decode_steering(p[0:2]))

    def _on_esp_throttle(self, msg: Frame):
        self.state.update("esp32_throttle", decode_u8(list(msg.payload)) / 255.0)

    def _on_esp_braking(self, msg: Frame):
        self.state.update("esp32_braking", decode_u8(list(msg.payload)) / 255.0)

    def _on_zed_imu(self, msg: Imu):
        # ZED2 ROS2 wrapper uses REP-103: x=forward, y=left, z=up
        self.state.update("esp32_accel_lon", msg.linear_acceleration.x)
        self.state.update("esp32_accel_lat", -msg.linear_acceleration.y)  # flip: y=left → positive=right

    def _on_esp_health(self, msg: Frame):
        fields = decode_health(list(msg.payload))
        for k, v in fields.items():
            self.state.update(k, v)

    def _on_orin_throttle(self, msg: Frame):
        self.state.update("orin_cmd_throttle", decode_u8(list(msg.payload)))

    def _on_orin_brake(self, msg: Frame):
        self.state.update("orin_cmd_brake", decode_u8(list(msg.payload)))

    def _on_orin_steering(self, msg: Frame):
        self.state.update("orin_cmd_steering_rad", decode_steering(list(msg.payload)))

    def publish_mission(self, mission: str):
        msg = String()
        msg.data = mission
        self.mission_pub.publish(msg)
        self.get_logger().info(f"Mission set: {mission}")


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
