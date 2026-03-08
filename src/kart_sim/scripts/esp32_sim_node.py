#!/usr/bin/env python3
"""Simulate ESP32 telemetry from Gazebo odometry.

Replaces kb_coms_micro in simulation: reads Gazebo odom and publishes
the same /esp32/* Frame topics that the real ESP32 would produce.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from kb_interfaces.msg import Frame
from kb_dashboard.protocol import (
    encode_act_steering,
    encode_act_speed,
    encode_act_accel,
    encode_act_braking,
    encode_act_throttle,
    encode_heartbeat,
    encode_health,
)


class Esp32SimNode(Node):
    def __init__(self):
        super().__init__("esp32_sim")

        # Publishers (same as kb_coms_micro)
        self.pub_heartbeat = self.create_publisher(Frame, "/esp32/heartbeat", 10)
        self.pub_steering = self.create_publisher(Frame, "/esp32/steering", 10)
        self.pub_speed = self.create_publisher(Frame, "/esp32/speed", 10)
        self.pub_accel = self.create_publisher(Frame, "/esp32/acceleration", 10)
        self.pub_throttle = self.create_publisher(Frame, "/esp32/throttle", 10)
        self.pub_braking = self.create_publisher(Frame, "/esp32/braking", 10)
        self.pub_health = self.create_publisher(Frame, "/esp32/health", 10)

        # Subscribers
        self.create_subscription(Odometry, "/model/kart/odometry", self._on_odom, 10)
        self.create_subscription(Twist, "/kart/cmd_vel", self._on_cmd_vel, 10)

        # State
        self._prev_speed = 0.0
        self._prev_time = self.get_clock().now()
        self._steer_angle = 0.0
        self._throttle = 0.0
        self._brake = 0.0

        # Timers
        self.create_timer(1.0, self._publish_heartbeat)
        self.create_timer(0.05, self._publish_telemetry)  # 20 Hz
        self.create_timer(2.0, self._publish_health)

        self.get_logger().info("ESP32 sim node started (fake telemetry from Gazebo)")

    def _on_odom(self, msg: Odometry):
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds / 1e9
        speed = msg.twist.twist.linear.x

        if dt > 0.001:
            self._accel_lon = (speed - self._prev_speed) / dt
        else:
            self._accel_lon = 0.0

        self._speed = speed
        self._accel_lat = speed * msg.twist.twist.angular.z  # v * yaw_rate
        self._prev_speed = speed
        self._prev_time = now

    def _on_cmd_vel(self, msg: Twist):
        self._steer_angle = msg.angular.z  # steer angle in radians
        speed = msg.linear.x
        if speed > 0.1:
            self._throttle = min(1.0, speed / 10.0)
            self._brake = 0.0
        elif speed < -0.1:
            self._throttle = 0.0
            self._brake = min(1.0, abs(speed) / 10.0)
        else:
            self._throttle = 0.0
            self._brake = 0.0

    def _publish_heartbeat(self):
        msg = Frame()
        msg.type = Frame.ESP_HEARTBEAT
        msg.payload = encode_heartbeat()
        self.pub_heartbeat.publish(msg)

    def _publish_telemetry(self):
        speed = getattr(self, "_speed", 0.0)
        accel_lat = getattr(self, "_accel_lat", 0.0)
        accel_lon = getattr(self, "_accel_lon", 0.0)

        # Steering: protobuf ActSteering with angle + fake raw encoder
        steer_msg = Frame()
        steer_msg.type = Frame.ESP_ACT_STEERING
        raw_encoder = 2048 + int(self._steer_angle * 650)  # fake AS5600-like value
        steer_msg.payload = encode_act_steering(self._steer_angle, raw_encoder)
        self.pub_steering.publish(steer_msg)

        # Speed: protobuf ActSpeed
        speed_msg = Frame()
        speed_msg.type = Frame.ESP_ACT_SPEED
        speed_msg.payload = encode_act_speed(speed)
        self.pub_speed.publish(speed_msg)

        # Acceleration: protobuf ActAcceleration
        accel_msg = Frame()
        accel_msg.type = Frame.ESP_ACT_ACCELERATION
        accel_msg.payload = encode_act_accel(accel_lat, accel_lon)
        self.pub_accel.publish(accel_msg)

        # Throttle: protobuf TargThrottle (reused for sim feedback)
        throttle_msg = Frame()
        throttle_msg.type = 0x00  # sim-only: dashboard routes by topic
        throttle_msg.payload = encode_act_throttle(self._throttle)
        self.pub_throttle.publish(throttle_msg)

        # Braking: protobuf ActBraking
        brake_msg = Frame()
        brake_msg.type = Frame.ESP_ACT_BRAKING
        brake_msg.payload = encode_act_braking(self._brake)
        self.pub_braking.publish(brake_msg)

    def _publish_health(self):
        msg = Frame()
        msg.type = 0x0B  # ESP_HEALTH_STATUS
        msg.payload = encode_health(
            magnet_ok=True, i2c_ok=True, heap_ok=True,
            agc=50, heap_kb=200, i2c_errors=0,
        )
        self.pub_health.publish(msg)


def main():
    rclpy.init()
    node = Esp32SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
