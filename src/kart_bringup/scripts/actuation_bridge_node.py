#!/usr/bin/env python3
"""Bridge between ROS2 actuation commands and ESP32 serial frames.

Subscribes to /actuation_cmd (AckermannDriveStamped) and publishes
kb_interfaces/Frame messages on /esp32/tx for kb_coms_micro to send
over serial.

Scaling (matches ACTUATION_PROTOCOL.md V1):
  - steering_angle [-1, 1] → STEER s8 [-127, 127] (positive = left)
  - acceleration > 0 → THROTTLE u8 [0, 255]
  - acceleration < 0 → BRAKE u8 [0, 255]
"""
import struct

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from kb_interfaces.msg import Frame


class ActuationBridgeNode(Node):
    def __init__(self):
        super().__init__("actuation_bridge")

        self.declare_parameter("input_topic", "/actuation_cmd")
        self.declare_parameter("output_topic", "/esp32/tx")
        self.declare_parameter("rate_hz", 20.0)

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        rate = float(self.get_parameter("rate_hz").value)

        self.frame_pub = self.create_publisher(Frame, out_topic, 10)
        self.sub = self.create_subscription(
            AckermannDriveStamped, in_topic, self._on_cmd, 10
        )

        self._last_steer = 0
        self._last_throttle = 0
        self._last_brake = 0

        # Periodic send so ESP32 doesn't hit its watchdog timeout
        self.timer = self.create_timer(1.0 / rate, self._send_frames)

    def _on_cmd(self, msg: AckermannDriveStamped):
        steer = msg.drive.steering_angle  # [-1, 1]
        accel = msg.drive.acceleration     # [-1, 1]

        # Steering: clamp and scale to s8
        steer_clamped = max(-1.0, min(1.0, steer))
        self._last_steer = int(steer_clamped * 127.0)

        # Throttle / brake split
        if accel >= 0:
            self._last_throttle = int(min(1.0, accel) * 255.0)
            self._last_brake = 0
        else:
            self._last_throttle = 0
            self._last_brake = int(min(1.0, -accel) * 255.0)

    def _send_frames(self):
        # ORIN_TARG_STEERING (0x22): payload = [direction, magnitude]
        steer_frame = Frame()
        steer_frame.type = Frame.ORIN_TARG_STEERING
        direction = 0 if self._last_steer >= 0 else 1
        magnitude = min(255, abs(self._last_steer) * 2)  # scale s8 [-127,127] → u8
        steer_frame.payload = [direction, magnitude]

        # ORIN_TARG_THROTTLE (0x20): payload = u8
        throttle_frame = Frame()
        throttle_frame.type = Frame.ORIN_TARG_THROTTLE
        throttle_frame.payload = [self._last_throttle]

        # ORIN_TARG_BRAKING (0x21): payload = u8
        brake_frame = Frame()
        brake_frame.type = Frame.ORIN_TARG_BRAKING
        brake_frame.payload = [self._last_brake]

        self.frame_pub.publish(steer_frame)
        self.frame_pub.publish(throttle_frame)
        self.frame_pub.publish(brake_frame)


def main():
    rclpy.init()
    node = ActuationBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
