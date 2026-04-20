#!/usr/bin/env python3
"""Stanley Controller Node for the kart.

Implements the Stanley control algorithm for autonomous racing.
Receives Detection3DArray in the camera optical frame (Z=forward, X=right, Y=down)
and Odometry for current speed, and publishes Twist on /kart/cmd_vel.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray


def _build_midpoint_path(cones, half_track_width=1.5):
    """Pair blue/yellow cones and return a sorted list of (fwd, left) midpoints."""
    blues = [
        (fwd, left, math.hypot(fwd, left))
        for cls, fwd, left in cones
        if cls == "blue_cone" and fwd > 0.3
    ]
    yellows = [
        (fwd, left, math.hypot(fwd, left))
        for cls, fwd, left in cones
        if cls == "yellow_cone" and fwd > 0.3
    ]

    blues.sort(key=lambda c: c[2])
    yellows.sort(key=lambda c: c[2])

    midpoints = []
    if blues and yellows:
        used_y = set()
        for bx, by, bd in blues:
            best_j, best_dd = -1, float("inf")
            for j, (yx, yy, yd) in enumerate(yellows):
                if j in used_y:
                    continue
                dd = abs(bd - yd)
                if dd < best_dd:
                    best_dd = dd
                    best_j = j
            if best_j >= 0 and best_dd < 8.0:
                yx, yy, _ = yellows[best_j]
                used_y.add(best_j)
                midpoints.append(((bx + yx) / 2.0, (by + yy) / 2.0))
            else:
                midpoints.append((bx, by - half_track_width))
        for j, (yx, yy, _) in enumerate(yellows):
            if j not in used_y:
                midpoints.append((yx, yy + half_track_width))
    elif blues:
        for bx, by, _ in blues:
            midpoints.append((bx, by - half_track_width))
    elif yellows:
        for yx, yy, _ in yellows:
            midpoints.append((yx, yy + half_track_width))

    midpoints.sort(key=lambda p: p[0])
    return midpoints


def _path_heading(midpoints, idx):
    """Estimate path heading at *idx* using finite differences."""
    n = len(midpoints)
    if n < 2:
        return 0.0
    i0 = max(0, idx - 1)
    i1 = min(n - 1, idx + 1)
    if i0 == i1:
        return 0.0
    dx = midpoints[i1][0] - midpoints[i0][0]
    dy = midpoints[i1][1] - midpoints[i0][1]
    return math.atan2(dy, dx)


class StanleyControllerNode(Node):
    """Stanley controller node for vehicle steering."""

    def __init__(self):
        super().__init__("stanley_controller")

        # Parameters
        self.declare_parameter("detections_topic", "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
        self.declare_parameter("odom_topic", "/zed/zed_node/odom")

        self.declare_parameter("stanley_k", 1.5)  # Control gain
        self.declare_parameter(
            "stanley_ks", 0.5
        )  # Softening gain (to avoid singularity at zero speed)
        self.declare_parameter("base_speed", 2.0)  # Base speed to command
        self.declare_parameter("max_steer_angle", 0.4)  # ~23 degrees

        det_topic = self.get_parameter("detections_topic").value
        cmd_topic = self.get_parameter("cmd_vel_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.k = self.get_parameter("stanley_k").value
        self.ks = self.get_parameter("stanley_ks").value
        self.base_speed = self.get_parameter("base_speed").value
        self.max_steer = self.get_parameter("max_steer_angle").value

        # State
        self.current_speed = 0.0

        # Publishers / Subscribers
        qos_det = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_odom = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.sub_det = self.create_subscription(
            Detection3DArray, det_topic, self.det_callback, qos_det
        )
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_odom
        )

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        self.get_logger().info("StanleyControllerNode initialized.")

    def odom_callback(self, msg: Odometry):
        # Update current velocity (forward speed)
        self.current_speed = msg.twist.twist.linear.x

    def det_callback(self, msg: Detection3DArray):
        cones = []
        for det in msg.detections:
            cls = det.results[0].hypothesis.class_id
            fwd = det.bbox.center.position.z
            left = -det.bbox.center.position.x
            cones.append((cls, fwd, left))

        if not cones:
            self._stop()
            return

        midpoints = _build_midpoint_path(cones, half_track_width=1.5)
        if not midpoints:
            self._stop()
            return

        # Stanley Control Logic
        # The front axle is our reference point for cross-track error.
        # Since our camera is near the front (or we assume base_link is front axle for simplicity here),
        # the origin (0,0) is our reference.

        # Find the closest point on the path
        min_dist = float("inf")
        min_idx = 0
        for i, (px, py) in enumerate(midpoints):
            dist = math.hypot(px, py)  # distance from origin to (px, py)
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        cx, cy = midpoints[min_idx]

        # Calculate heading error (theta_e)
        # Path heading relative to kart
        path_heading = _path_heading(midpoints, min_idx)
        # Kart heading is 0 in its own frame
        theta_e = path_heading - 0.0

        # Normalize theta_e to [-pi, pi]
        theta_e = (theta_e + math.pi) % (2 * math.pi) - math.pi

        # Calculate cross-track error (e_fa)
        # We need the lateral distance from the vehicle to the path.
        # cy is the lateral position of the closest path point.
        # But we need the signed distance. If path is to the left (cy > 0), e_fa is positive.
        # Actually, let's project the path point vector onto the path normal to get accurate cross-track error.

        if min_idx < len(midpoints) - 1:
            dx = midpoints[min_idx + 1][0] - midpoints[min_idx][0]
            dy = midpoints[min_idx + 1][1] - midpoints[min_idx][1]
        else:
            if len(midpoints) > 1:
                dx = midpoints[min_idx][0] - midpoints[min_idx - 1][0]
                dy = midpoints[min_idx][1] - midpoints[min_idx - 1][1]
            else:
                dx, dy = 1.0, 0.0  # fallback straight

        # Normal vector to the path (pointing left)
        length = math.hypot(dx, dy)
        if length > 0.001:
            nx = -dy / length
            ny = dx / length
        else:
            nx, ny = 0.0, 1.0

        # Vector from kart (0,0) to closest point (cx, cy)
        # e_fa = dot product of (cx, cy) and normal (nx, ny)
        # Wait, if normal points left and path is on left, this gives a positive value
        e_fa = cx * nx + cy * ny

        # Stanley steering law:
        # steer = theta_e + atan2(k * e_fa, v + ks)
        v = max(self.current_speed, 0.0)  # avoid negative velocity issues
        cross_track_steer = math.atan2(self.k * e_fa, v + self.ks)

        steer = theta_e + cross_track_steer

        # Clamp steering
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # Command speed (simple constant for now)
        speed = self.base_speed

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steer)
        self.pub_cmd.publish(cmd)

    def _stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
