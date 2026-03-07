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
from std_msgs.msg import String

from kb_dashboard.protocol import DashboardState, decode_steering, decode_u8
from kb_dashboard.server import run_websocket_server


class DashboardNode(Node):
    def __init__(self, state: DashboardState):
        super().__init__("kb_dashboard")
        self.state = state
        self.declare_parameter("port", 8080)
        self.port = self.get_parameter("port").value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ESP32 → Orin telemetry
        self.create_subscription(Frame, "/esp32/heartbeat", self._on_heartbeat, qos)
        self.create_subscription(Frame, "/esp32/steering", self._on_esp_steering, qos)
        self.create_subscription(Frame, "/esp32/speed", self._on_esp_speed, qos)
        self.create_subscription(Frame, "/esp32/acceleration", self._on_esp_accel, qos)
        self.create_subscription(Frame, "/esp32/braking", self._on_esp_braking, qos)

        # Orin → ESP32 commands (to show what we're sending)
        self.create_subscription(Frame, "/orin/throttle", self._on_orin_throttle, qos)
        self.create_subscription(Frame, "/orin/brake", self._on_orin_brake, qos)
        self.create_subscription(Frame, "/orin/steering", self._on_orin_steering, qos)

        # Publishers for mission commands
        self.mission_pub = self.create_publisher(String, "/dashboard/mission", 10)

        self.get_logger().info(f"Dashboard node started, web UI on port {self.port}")

    def _on_heartbeat(self, msg: Frame):
        self.state.heartbeat()

    def _on_esp_steering(self, msg: Frame):
        self.state.update("esp32_steering_rad", decode_steering(list(msg.payload)))

    def _on_esp_speed(self, msg: Frame):
        if msg.payload:
            self.state.update("esp32_speed", decode_steering(list(msg.payload)))

    def _on_esp_accel(self, msg: Frame):
        if msg.payload:
            self.state.update("esp32_acceleration", decode_steering(list(msg.payload)))

    def _on_esp_braking(self, msg: Frame):
        self.state.update("esp32_braking", decode_u8(list(msg.payload)))

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
