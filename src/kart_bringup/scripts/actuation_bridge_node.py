#!/usr/bin/env python3
"""Bridge between ROS2 actuation commands and ESP32 serial frames.

Subscribes to /actuation_cmd (AckermannDriveStamped) and publishes
kb_interfaces/Frame messages on /orin/throttle, /orin/brake,
/orin/steering — the topics that kb_coms_micro subscribes to and
relays over UART to the ESP32.

Scaling:
  - steering_angle (radians) → int16 big-endian, radians × 1000
  - acceleration > 0 → THROTTLE u8 [0, 255]
  - acceleration < 0 → BRAKE u8 [0, 255]
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from kb_interfaces.msg import Frame


class ActuationBridgeNode(Node):
    def __init__(self):
        super().__init__("actuation_bridge")

        self.declare_parameter("input_topic", "/actuation_cmd")
        self.declare_parameter("rate_hz", 100.0)

        in_topic = str(self.get_parameter("input_topic").value)
        rate = float(self.get_parameter("rate_hz").value)

        # Publish to the /orin/* topics that kb_coms_micro subscribes to
        self.throttle_pub = self.create_publisher(Frame, "/orin/throttle", 10)
        self.brake_pub = self.create_publisher(Frame, "/orin/brake", 10)
        self.steering_pub = self.create_publisher(Frame, "/orin/steering", 10)

        self.sub = self.create_subscription(
            AckermannDriveStamped, in_topic, self._on_cmd, 10
        )

        self._last_steer = 0
        self._last_throttle = 0
        self._last_brake = 0

        # Periodic send so ESP32 doesn't hit its watchdog timeout
        self.timer = self.create_timer(1.0 / rate, self._send_frames)

    def _on_cmd(self, msg: AckermannDriveStamped):
        steer = msg.drive.steering_angle  # radians
        accel = msg.drive.acceleration     # [-1, 1]

        # Steering: signed int16, radians × 1000
        steer_i16 = int(steer * 1000.0)
        self._last_steer = max(-32768, min(32767, steer_i16))

        # Throttle / brake split
        if accel >= 0:
            self._last_throttle = int(min(1.0, accel) * 255.0)
            self._last_brake = 0
        else:
            self._last_throttle = 0
            self._last_brake = int(min(1.0, -accel) * 255.0)

    def _send_frames(self):
        # ORIN_TARG_STEERING (0x22): int16 big-endian, radians × 1000
        steer_frame = Frame()
        steer_frame.type = Frame.ORIN_TARG_STEERING
        steer_frame.payload = [
            (self._last_steer >> 8) & 0xFF,
            self._last_steer & 0xFF,
        ]

        # ORIN_TARG_THROTTLE (0x20): payload = u8
        throttle_frame = Frame()
        throttle_frame.type = Frame.ORIN_TARG_THROTTLE
        throttle_frame.payload = [self._last_throttle]

        # ORIN_TARG_BRAKING (0x21): payload = u8
        brake_frame = Frame()
        brake_frame.type = Frame.ORIN_TARG_BRAKING
        brake_frame.payload = [self._last_brake]

        self.steering_pub.publish(steer_frame)
        self.throttle_pub.publish(throttle_frame)
        self.brake_pub.publish(brake_frame)


def main():
    rclpy.init()
    node = ActuationBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
