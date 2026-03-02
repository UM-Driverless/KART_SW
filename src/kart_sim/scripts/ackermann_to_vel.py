#!/usr/bin/env python3
"""Convert Ackermann cmd_vel (steer angle) to velocity cmd (yaw rate).

Subscribes to /kart/cmd_vel   (linear.x=speed, angular.z=steer_angle)
Publishes  to /kart/vel_cmd   (linear.x=speed, angular.z=yaw_rate)

Applies acceleration limits to match real-kart dynamics, since
VelocityControl has no inertia.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

WHEELBASE = 1.05  # metres (from model.sdf)
MAX_LINEAR_ACCEL = 2.0  # m/s²
MAX_LINEAR_DECEL = 3.0  # m/s²
MAX_ANGULAR_ACCEL = 2.0  # rad/s²


class AckermannToVel(Node):
    def __init__(self):
        super().__init__("ackermann_to_vel")
        self.sub = self.create_subscription(
            Twist, "/kart/cmd_vel", self._on_cmd, 10
        )
        self.pub = self.create_publisher(Twist, "/kart/vel_cmd", 10)

        self._cur_linear = 0.0
        self._cur_angular = 0.0
        self._last_time = time.monotonic()

        self.get_logger().info("AckermannToVel: steer→yaw + accel limits active")

    def _on_cmd(self, msg: Twist):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        dt = max(0.001, min(dt, 0.5))

        # Target velocities
        target_linear = msg.linear.x
        if abs(msg.linear.x) > 0.01:
            target_angular = msg.linear.x * math.tan(msg.angular.z) / WHEELBASE
        else:
            target_angular = 0.0

        # Apply acceleration limits
        lin_diff = target_linear - self._cur_linear
        max_lin_change = (MAX_LINEAR_ACCEL if lin_diff > 0 else MAX_LINEAR_DECEL) * dt
        if abs(lin_diff) > max_lin_change:
            self._cur_linear += math.copysign(max_lin_change, lin_diff)
        else:
            self._cur_linear = target_linear

        # No angular rate limit — the steering servo is fast and Gazebo
        # physics already handles the kart's rotational inertia.
        self._cur_angular = target_angular

        out = Twist()
        out.linear.x = self._cur_linear
        out.angular.z = self._cur_angular
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AckermannToVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
