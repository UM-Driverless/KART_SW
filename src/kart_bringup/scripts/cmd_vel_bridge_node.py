#!/usr/bin/env python3
"""Bridge from Twist (cone_follower output) to ESP32 serial frames.

Subscribes to /kart/cmd_vel (Twist) and publishes kb_interfaces/Frame
messages on /orin/throttle, /orin/brake, /orin/steering — the topics
that kb_coms_micro subscribes to and relays over UART to the ESP32.

Payload encoding: protobuf (TargThrottle, TargBraking, TargSteering).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kb_interfaces.msg import Frame
from kb_dashboard.protocol import encode_steering, encode_throttle, encode_braking


class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        self.declare_parameter("input_topic", "/kart/cmd_vel")
        self.declare_parameter("rate_hz", 100.0)
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

        self._throttle_effort = 0.0
        self._brake_effort = 0.0
        self._steer_rad = 0.0

        self.timer = self.create_timer(1.0 / rate, self._send_frames)
        self.get_logger().info(f"CmdVelBridge: {in_topic} @ {rate} Hz")

    def _on_cmd(self, msg: Twist):
        speed = msg.linear.x
        steer = msg.angular.z

        # Throttle / brake from speed
        if speed >= 0:
            self._throttle_effort = min(1.0, speed / self.max_speed)
            self._brake_effort = 0.0
        else:
            self._throttle_effort = 0.0
            self._brake_effort = min(1.0, -speed / self.max_speed)

        # Steering: clamp to max_steer
        self._steer_rad = max(-self.max_steer, min(self.max_steer, steer))

    def _send_frames(self):
        throttle_frame = Frame()
        throttle_frame.type = Frame.ORIN_TARG_THROTTLE
        throttle_frame.payload = encode_throttle(self._throttle_effort)

        brake_frame = Frame()
        brake_frame.type = Frame.ORIN_TARG_BRAKING
        brake_frame.payload = encode_braking(self._brake_effort)

        steer_frame = Frame()
        steer_frame.type = Frame.ORIN_TARG_STEERING
        steer_frame.payload = encode_steering(self._steer_rad)

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
