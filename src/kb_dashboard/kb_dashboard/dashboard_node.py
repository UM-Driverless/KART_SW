#!/usr/bin/env python3
"""
kb_dashboard — Phone dashboard for kart telemetry and mission control.

Runs a WebSocket server alongside a ROS2 node. Any phone/browser on the
same network can open http://<orin-ip>:9090 to see live sensor values
and send commands (mission select, start/stop, EBS).
"""

import asyncio
import threading
import time

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
    ORIN_STEER_MODE,
    decode_steering,
    decode_steering_raw,
    decode_speed,
    decode_accel,
    decode_braking,
    decode_throttle,
    decode_health,
    encode_steer_mode,
)
from kb_dashboard.server import run_websocket_server


class DashboardNode(Node):
    """@brief ROS2 node that bridges kart telemetry to a WebSocket-based dashboard.

    Subscribes to ESP32 telemetry, ZED IMU, YOLO FPS, HUD images, and state
    machine feedback, forwarding them to the dashboard web UI via DashboardState.
    Also publishes mission and manual control commands from the dashboard.
    """

    def __init__(self, state: DashboardState):
        """@brief Initialize the dashboard node with subscriptions and publishers.

        @param state Shared DashboardState instance for thread-safe telemetry exchange.
        """
        super().__init__("kb_dashboard")
        self.state = state
        self.declare_parameter("port", 9090)
        self.declare_parameter("password", "0")
        self.port = self.get_parameter("port").value
        self.password = self.get_parameter("password").value

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

        # ZED VIO speed (from cone_follower odom extraction)
        self.create_subscription(
            Float32, "/kart/speed", self._on_kart_speed, qos_reliable
        )

        # YOLO FPS
        self.create_subscription(
            Float32, "/perception/yolo/fps", self._on_yolo_fps, qos_reliable
        )

        # HUD image stream (JPEG bytes stored for WebSocket binary broadcast)
        self._bridge = CvBridge()
        self._hud_jpeg: bytes | None = None
        self.create_subscription(Image, "/perception/hud", self._on_hud_image, 1)

        # State machine feedback
        self.create_subscription(
            String, "/kart/state", self._on_kart_state, qos_reliable
        )

        # Publishers for mission commands
        self.mission_pub = self.create_publisher(String, "/dashboard/mission", 10)
        self.state_cmd_pub = self.create_publisher(String, "/dashboard/state_cmd", 10)

        # Publisher for manual remote control (Twist for now)
        self.manual_cmd_pub = self.create_publisher(Twist, "/kart/cmd_vel_manual", 10)
        # Steering mode publisher (Frame to ESP32 via kb_coms_micro)
        self.steer_mode_pub = self.create_publisher(Frame, "/orin/steer_mode", 10)
        self._steer_mode = 0  # 0=PID, 1=direct PWM
        self.declare_parameter("pwm_limit", 0.40)
        self._pwm_limit = float(self.get_parameter("pwm_limit").value)
        # Pending commands set from asyncio thread, published by ROS timer
        self._pending_manual_cmd = None
        self._manual_cmd_time = 0.0  # monotonic timestamp of last WS manual_control
        self._pending_mission = None
        self._pending_mission_count = 0
        self._pending_state_cmd = None
        self._pending_state_cmd_count = 0
        self._pending_steer_mode = None
        self._pending_steer_mode_count = 0
        # Controller type publisher (String to cone_follower)
        self.controller_type_pub = self.create_publisher(String, "/dashboard/controller_type", 10)
        self._pending_controller_type = None
        self._pending_controller_type_count = 0
        self.create_timer(0.01, self._publish_pending)  # 100 Hz

        # One-shot self-test after 2 seconds
        self._selftest_timer = self.create_timer(2.0, self._selftest)

        self.get_logger().info(f"Dashboard node started, web UI on port {self.port}")

    def _selftest(self):
        """@brief One-shot self-test: log subscriber counts for all publishers."""
        self._selftest_timer.cancel()
        pubs = {
            "/dashboard/mission": self.mission_pub,
            "/dashboard/state_cmd": self.state_cmd_pub,
            "/kart/cmd_vel_manual": self.manual_cmd_pub,
        }
        for topic, pub in pubs.items():
            subs = pub.get_subscription_count()
            if subs == 0:
                self.get_logger().warn(f"Self-test: {topic} has 0 subscribers")
            else:
                self.get_logger().info(f"Self-test: {topic} OK ({subs} subs)")

    def _on_heartbeat(self, msg: Frame):
        """@brief Callback for ESP32 heartbeat frames. Updates heartbeat timestamp.
        Also publishes any pending commands — this callback runs on the ROS thread
        so publish() is guaranteed to work (unlike timers which may not fire).
        """
        self.state.heartbeat()
        self._flush_pending()

    def _on_esp_steering(self, msg: Frame):
        """@brief Callback for ESP32 steering frames."""
        self._flush_pending()
        p = list(msg.payload)
        angle_rad, raw_encoder, pid_pwm = decode_steering_raw(p)
        self.state.update("esp32_steering_rad", angle_rad)
        if raw_encoder:
            self.state.update("esp32_steering_raw", raw_encoder)
        self.state.update("esp32_steering_pwm", pid_pwm)

    def _on_esp_speed(self, msg: Frame):
        """@brief Callback for ESP32 speed frames. Decodes speed in m/s."""
        if msg.payload:
            self.state.update("esp32_speed", decode_speed(list(msg.payload)))

    def _on_kart_speed(self, msg):
        """@brief Callback for ZED VIO speed (Float32, m/s). Updates same state key as ESP32 speed."""
        self.state.update("esp32_speed", round(msg.data, 2))

    def _on_esp_accel(self, msg: Frame):
        """@brief Callback for ESP32 acceleration frames. Decodes lateral and longitudinal acceleration."""
        p = list(msg.payload)
        if p:
            lat, lon = decode_accel(p)
            self.state.update("esp32_accel_lat", lat)
            self.state.update("esp32_accel_lon", lon)

    def _on_esp_throttle(self, msg: Frame):
        """@brief Callback for ESP32 throttle frames. Decodes throttle effort 0.0-1.0."""
        self.state.update("esp32_throttle", decode_throttle(list(msg.payload)))

    def _on_esp_braking(self, msg: Frame):
        """@brief Callback for ESP32 braking frames. Decodes braking effort 0.0-1.0."""
        self.state.update("esp32_braking", decode_braking(list(msg.payload)))

    def _on_zed_imu(self, msg: Imu):
        """@brief Callback for ZED2 IMU data. Extracts linear acceleration (REP-103 convention)."""
        # ZED2 ROS2 wrapper uses REP-103: x=forward, y=left, z=up
        self.state.update("esp32_accel_lon", msg.linear_acceleration.x)
        self.state.update(
            "esp32_accel_lat", -msg.linear_acceleration.y
        )  # flip: y=left → positive=right

    def _on_esp_health(self, msg: Frame):
        """@brief Callback for ESP32 health status frames. Updates magnet, I2C, heap fields."""
        fields = decode_health(list(msg.payload))
        for k, v in fields.items():
            self.state.update(k, v)

    def _on_orin_throttle(self, msg: Frame):
        """@brief Callback for Orin-to-ESP32 throttle command echo."""
        self.state.update("orin_cmd_throttle", decode_throttle(list(msg.payload)))

    def _on_orin_brake(self, msg: Frame):
        """@brief Callback for Orin-to-ESP32 brake command echo."""
        self.state.update("orin_cmd_brake", decode_throttle(list(msg.payload)))

    def _on_orin_steering(self, msg: Frame):
        """@brief Callback for Orin-to-ESP32 steering command echo."""
        self.state.update("orin_cmd_steering_rad", decode_steering(list(msg.payload)))

    def _on_kart_state(self, msg: String):
        """@brief Callback for AS state machine updates. Maps AS states to dashboard state names."""
        # Map AS state names to dashboard state names for the UI
        as_to_dash = {
            "AS_OFF": "idle",
            "AS_READY": "ready",
            "AS_DRIVING": "running",
            "AS_FINISHED": "finished",
            "AS_EMERGENCY": "ebs",
        }
        self.state.update("state", as_to_dash.get(msg.data, "idle"))
        self.state.update("as_state", msg.data)

    def _on_yolo_fps(self, msg: Float32):
        """@brief Callback for YOLO inference FPS updates."""
        self.state.update("yolo_fps", round(msg.data, 1))

    def _on_hud_image(self, msg: Image):
        """@brief Callback for HUD image. Converts to JPEG for WebSocket binary broadcast."""
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        self._hud_jpeg = buf.tobytes()

    def get_hud_jpeg(self) -> bytes | None:
        """@brief Get the latest HUD JPEG frame for WebSocket broadcast.

        @return JPEG bytes or None if no HUD image has been received yet.
        """
        return self._hud_jpeg

    def publish_mission(self, mission: str):
        """@brief Queue a mission selection for the ROS timer to publish."""
        msg = String()
        msg.data = mission
        self._pending_mission = msg
        self._pending_mission_count = 100  # publish for 1 second (100 Hz timer)
        self.get_logger().info(f"Mission set: {mission}")

    def publish_state_cmd(self, cmd: str):
        """@brief Queue a state command for the ROS timer to publish."""
        msg = String()
        msg.data = cmd
        self._pending_state_cmd = msg
        self._pending_state_cmd_count = 100
        self.get_logger().info(f"State cmd: {cmd}")

    def _publish_pending(self):
        """@brief Timer callback (100 Hz): publish pending commands from ROS thread."""
        self._flush_pending()

    def _flush_pending(self):
        """@brief Publish any pending commands. Safe to call from any ROS callback."""
        cmd = self._pending_manual_cmd
        if cmd is not None:
            # If no WS input for 500ms, publish zeros (safe stop)
            if time.monotonic() - self._manual_cmd_time > 0.5:
                self._pending_manual_cmd = Twist()  # zero, not None — keep publishing zeros
            self.manual_cmd_pub.publish(self._pending_manual_cmd)
        if self._pending_mission is not None and self._pending_mission_count > 0:
            self.mission_pub.publish(self._pending_mission)
            self._pending_mission_count -= 1
        if self._pending_state_cmd is not None and self._pending_state_cmd_count > 0:
            self.state_cmd_pub.publish(self._pending_state_cmd)
            self._pending_state_cmd_count -= 1
        if self._pending_steer_mode is not None and self._pending_steer_mode_count > 0:
            self.steer_mode_pub.publish(self._pending_steer_mode)
            self._pending_steer_mode_count -= 1
        if self._pending_controller_type is not None and self._pending_controller_type_count > 0:
            self.controller_type_pub.publish(self._pending_controller_type)
            self._pending_controller_type_count -= 1

    def publish_steer_mode(self, mode: int):
        """@brief Publish steering mode change to ESP32.

        @param mode 0=PID, 1=direct PWM.
        """
        self._steer_mode = mode
        self.state.update("steer_mode", "pwm" if mode else "pid")
        frame = Frame()
        frame.type = ORIN_STEER_MODE
        frame.payload = encode_steer_mode(mode)
        self._pending_steer_mode = frame
        self._pending_steer_mode_count = 100  # publish for 1s to ensure delivery
        self.get_logger().info(f"Steer mode: {'PWM' if mode else 'PID'}")

    def publish_controller_type(self, ctrl_type: str):
        """@brief Publish controller type change to cone_follower.

        @param ctrl_type One of: geometric, pure_pursuit, neural_v2.
        """
        msg = String()
        msg.data = ctrl_type
        self._pending_controller_type = msg
        self._pending_controller_type_count = 100
        self.get_logger().info(f"Controller type: {ctrl_type}")

    def publish_manual_control(
        self, steer: float, steer_type: str, throttle: float, brake: float
    ):
        """@brief Store remote control command for publishing by the ROS timer.

        Called from the asyncio thread. The actual publish happens in _publish_pending_manual
        on the ROS spin thread to avoid thread-safety issues.

        @param steer Steering input, -1.0 (left) to 1.0 (right).
        @param steer_type Steering mode string (e.g. "angle", "pwm").
        @param throttle Throttle input, 0.0 to 1.0.
        @param brake Brake input, 0.0 to 1.0.
        """
        NOMINAL_MAX_SPEED = 5.0
        NOMINAL_MAX_STEER = 0.785  # radians (~45 deg)

        cmd = Twist()
        cmd.linear.x = (throttle - brake) * NOMINAL_MAX_SPEED
        if self._steer_mode == 1:
            # Direct PWM mode: scale joystick [-1, 1] to [-pwm_limit, pwm_limit]
            cmd.angular.z = steer * self._pwm_limit
        else:
            # PID mode: scale to radians
            cmd.angular.z = steer * NOMINAL_MAX_STEER
        self._pending_manual_cmd = cmd
        self._manual_cmd_time = time.monotonic()


# ── Entrypoint ─────────────────────────────────────────────────────────


def main(args=None):
    """@brief Entrypoint: spins ROS in a background thread and runs the async web server."""
    rclpy.init(args=args)
    state = DashboardState()
    node = DashboardNode(state)

    # Run ROS spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run the async web server in the main thread
    import signal

    def _shutdown_on_signal(*_):
        """Stop the asyncio event loop so the process exits on SIGTERM."""
        try:
            loop = asyncio.get_running_loop()
            loop.call_soon_threadsafe(loop.stop)
        except RuntimeError:
            pass

    signal.signal(signal.SIGTERM, _shutdown_on_signal)
    try:
        asyncio.run(run_websocket_server(state, node, node.port, password=node.password))
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
