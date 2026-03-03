#!/usr/bin/env python3
"""Bridge from Twist (cone_follower output) to ESP32 serial frames.

Subscribes to /kart/cmd_vel (Twist) and publishes kb_interfaces/Frame
messages on /orin/throttle, /orin/brake, /orin/steering — the topics
that kb_coms_micro subscribes to and relays over UART to the ESP32.

Payload encoding (matches ESP32 km_coms.c KM_COMS_ProccessPayload):
  - ORIN_TARG_THROTTLE (0x20): payload[0] = u8 [0, 255]
  - ORIN_TARG_BRAKING  (0x21): payload[0] = u8 [0, 255]
  - ORIN_TARG_STEERING (0x22): payload[0] = direction (0=positive, 1=negative)
                                payload[1] = magnitude u8 [0, 255]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kb_interfaces.msg import Frame


class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        self.declare_parameter("input_topic", "/kart/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("max_speed", 5.0)
        self.declare_parameter("max_steer", 0.5)

        in_topic = str(self.get_parameter("input_topic").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_steer = float(self.get_parameter("max_steer").value)

        # Publish to the /orin/* topics that kb_coms_micro subscribes to
        self.throttle_pub = self.create_publisher(Frame, "/orin/throttle", 10)
        self.brake_pub = self.create_publisher(Frame, "/orin/brake", 10)
        self.steering_pub = self.create_publisher(Frame, "/orin/steering", 10)

        self.sub = self.create_subscription(Twist, in_topic, self._on_cmd, 10)

        self._throttle = 0
        self._brake = 0
        self._steer_dir = 0
        self._steer_mag = 0

        self.timer = self.create_timer(1.0 / rate, self._send_frames)
        self.get_logger().info(f"CmdVelBridge: {in_topic} @ {rate} Hz")

    def _on_cmd(self, msg: Twist):
        speed = msg.linear.x
        steer = msg.angular.z

        # Throttle / brake from speed
        if speed >= 0:
            self._throttle = int(min(1.0, speed / self.max_speed) * 255.0)
            self._brake = 0
        else:
            self._throttle = 0
            self._brake = int(min(1.0, -speed / self.max_speed) * 255.0)

        # Steering: direction + magnitude
        steer_clamped = max(-self.max_steer, min(self.max_steer, steer))
        if steer_clamped >= 0:
            self._steer_dir = 0
        else:
            self._steer_dir = 1
        self._steer_mag = int(abs(steer_clamped) / self.max_steer * 255.0)

    def _send_frames(self):
        throttle_frame = Frame()
        throttle_frame.type = Frame.ORIN_TARG_THROTTLE
        throttle_frame.payload = [self._throttle]

        brake_frame = Frame()
        brake_frame.type = Frame.ORIN_TARG_BRAKING
        brake_frame.payload = [self._brake]

        steer_frame = Frame()
        steer_frame.type = Frame.ORIN_TARG_STEERING
        steer_frame.payload = [self._steer_dir, self._steer_mag]

        self.throttle_pub.publish(throttle_frame)
        self.brake_pub.publish(brake_frame)
        self.steering_pub.publish(steer_frame)


def main():
    rclpy.init()
    node = CmdVelBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
