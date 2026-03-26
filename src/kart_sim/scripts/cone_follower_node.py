#!/usr/bin/env python3
"""Cone-following controller for the kart.

Supports three controller types (selected via the ``controller_type`` param):

**geometric**
    Nearest blue/yellow midpoint → atan2 → steer.  Six tunable params.

**neural**
    Small feed-forward net (8→8→2), 90 weights.

**neural_v2**
    Larger net (17→16→2) with 4 cones per side + speed feedback, 322 weights.
    Trained with lap-time fitness for faster driving.

All controllers receive Detection3DArray in the camera *optical* frame
(Z=forward, X=right, Y=down) and publish Twist on ``/kart/cmd_vel``.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

try:
    from kart_perception.zed_od_utils import HAS_ZED_INTERFACES, zed_objects_to_det3d
    if HAS_ZED_INTERFACES:
        from zed_interfaces.msg import ObjectsStamped
except ImportError:
    HAS_ZED_INTERFACES = False


class ConeFollowerNode(Node):
    """@brief Cone-following controller node.

    Supports geometric, neural (v1), and neural_v2 controller types. Receives
    Detection3DArray in camera optical frame and publishes Twist on /kart/cmd_vel.
    """

    def __init__(self):
        """@brief Initialize the controller with parameters, neural weights (if applicable), and ROS plumbing."""
        super().__init__("cone_follower")

        # --- common params ---
        self.declare_parameter("detections_topic", "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
        self.declare_parameter("no_cone_timeout", 1.0)
        self.declare_parameter("controller_type", "geometric")  # geometric|pure_pursuit|neural|neural_v2
        self.declare_parameter("weights_json", "")               # path for neural

        # --- geometric params ---
        self.declare_parameter("steering_gain", 3.0)
        self.declare_parameter("max_steer", 1.047)
        self.declare_parameter("max_speed", 2.625)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("lookahead_max", 15.0)
        self.declare_parameter("half_track_width", 1.5)
        self.declare_parameter("speed_curve_factor", 0.0)

        det_topic = str(self.get_parameter("detections_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.no_cone_timeout = float(self.get_parameter("no_cone_timeout").value)
        self.controller_type = str(self.get_parameter("controller_type").value)

        # geometric fields
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.half_track_width = float(self.get_parameter("half_track_width").value)
        self.speed_curve_factor = float(self.get_parameter("speed_curve_factor").value)

        # neural net weights (loaded for neural or neural_v2)
        self._nn_W1 = self._nn_b1 = self._nn_W2 = self._nn_b2 = None
        self._nn_max_steer = 0.5
        self._nn_max_speed = 10.0
        self._nn_input_size = 8    # v1 default
        self._nn_n_blue = 2        # cones per side for v1
        self._nn_n_yellow = 2
        self._nn_uses_speed = False
        self._current_speed = 0.0

        if self.controller_type in ("neural", "neural_v2"):
            self._load_neural_weights()

        # ROS plumbing
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )
        # Also subscribe to ZED SDK ObjectsStamped for built-in OD mode
        if HAS_ZED_INTERFACES:
            self.declare_parameter("zed_objects_topic", "/zed/zed_node/obj_det/objects")
            self.create_subscription(
                ObjectsStamped,
                str(self.get_parameter("zed_objects_topic").value),
                lambda msg: self._on_detections(zed_objects_to_det3d(msg)),
                10,
            )
        # Subscribe to odometry for actual speed feedback
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/model/kart/odom_gt", self._on_odom, odom_qos
        )
        self._actual_speed = 0.0
        self._last_steer = 0.0
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._safety_check)
        self.create_subscription(String, "/dashboard/controller_type", self._on_controller_type, 10)

        self.get_logger().info(f"Controller type: {self.controller_type}")

    def _on_controller_type(self, msg: String):
        """@brief Callback for runtime controller type changes from the dashboard."""
        new_type = msg.data
        if new_type in ("geometric", "pure_pursuit", "neural_v2") and new_type != self.controller_type:
            self.get_logger().info(f"Controller type: {self.controller_type} → {new_type}")
            self.controller_type = new_type
            if new_type in ("neural", "neural_v2") and self._nn_W1 is None:
                self._load_neural_weights()

    def _on_odom(self, msg: Odometry):
        """@brief Callback for odometry. Extracts current speed for neural_v2 speed feedback."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._actual_speed = math.sqrt(vx * vx + vy * vy)

    # ── neural net loading ────────────────────────────────────────────

    def _load_neural_weights(self):
        """@brief Load neural network weights from a JSON file specified by the weights_json parameter."""
        path = str(self.get_parameter("weights_json").value)
        if not path:
            self.get_logger().error(
                f"controller_type={self.controller_type} but weights_json not set")
            raise SystemExit(1)

        with open(path) as f:
            data = json.load(f)

        genes = np.array(data["genes"], dtype=np.float64)
        self.get_logger().info(
            f"Loaded {self.controller_type} weights from {path} "
            f"(fitness={data.get('fitness', '?')})"
        )

        if self.controller_type == "neural_v2":
            # 17→16→2: W1 (17×16), b1 (16), W2 (16×2), b2 (2) — 322 genes
            self._nn_input_size = 17
            self._nn_n_blue = 4
            self._nn_n_yellow = 4
            self._nn_uses_speed = True
            hs = 16
            i = 0
            self._nn_W1 = genes[i:i + 17 * hs].reshape(17, hs);  i += 17 * hs
            self._nn_b1 = genes[i:i + hs];                        i += hs
            self._nn_W2 = genes[i:i + hs * 2].reshape(hs, 2);    i += hs * 2
            self._nn_b2 = genes[i:i + 2]
        else:
            # 8→8→2: W1 (8×8), b1 (8), W2 (8×2), b2 (2) — 90 genes
            hs = 8
            i = 0
            self._nn_W1 = genes[i:i + 8 * hs].reshape(8, hs);  i += 8 * hs
            self._nn_b1 = genes[i:i + hs];                      i += hs
            self._nn_W2 = genes[i:i + hs * 2].reshape(hs, 2);   i += hs * 2
            self._nn_b2 = genes[i:i + 2]

    # ── detection callback ────────────────────────────────────────────

    def _on_detections(self, msg: Detection3DArray):
        """@brief Callback for 3D cone detections. Filters cones by FOV/range and runs the active controller.

        @param msg Detection3DArray in camera optical frame (Z=forward, X=right, Y=down).
        """
        self.last_detection_time = self.get_clock().now()

        # Parse detections into (class_id, fwd, left) in camera_link frame.
        # Filter to match 2D-sim training perception (FOV ±40°, range 15 m)
        # so the neural nets receive in-distribution inputs.
        cones = []
        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos = det.results[0].pose.pose.position
            fwd = pos.z
            left = -pos.x
            if fwd < 0.5:
                continue
            dist = math.hypot(fwd, left)
            if dist > 15.0:
                continue
            angle = abs(math.atan2(left, fwd))
            if angle > 0.6109:  # ±35° in radians (ZED 2i @ VGA = 70° total)
                continue
            cones.append((class_id, fwd, left))

        if self.controller_type in ("neural", "neural_v2"):
            steer, speed = self._control_neural(cones)
        elif self.controller_type == "pure_pursuit":
            steer, speed = self._control_pure_pursuit(cones)
        else:
            steer, speed = self._control_geometric(cones)

        # Safety: if no cones visible, slow down and keep last steer
        # (don't hard-stop — cones may reappear after a curve transition)
        if not cones:
            speed = 0.0
            steer = 0.0

        cmd = Twist()
        cmd.angular.z = steer
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)

    # ── geometric controller ──────────────────────────────────────────

    def _control_geometric(self, cones):
        """@brief Geometric controller: steer toward nearest blue/yellow midpoint.

        @param cones List of (class_id, fwd, left) tuples in camera_link frame.
        @return Tuple of (steer_rad, speed_mps).
        """
        nearest_blue = None
        nearest_yellow = None
        min_bd = float("inf")
        min_yd = float("inf")

        for cls, fwd, left in cones:
            dist = math.hypot(fwd, left)
            if dist > self.lookahead_max:
                continue
            if cls == "blue_cone" and dist < min_bd:
                min_bd = dist
                nearest_blue = (fwd, left)
            elif cls == "yellow_cone" and dist < min_yd:
                min_yd = dist
                nearest_yellow = (fwd, left)

        if nearest_blue and nearest_yellow:
            mid_f = (nearest_blue[0] + nearest_yellow[0]) / 2.0
            mid_l = (nearest_blue[1] + nearest_yellow[1]) / 2.0
        elif nearest_blue:
            mid_f = nearest_blue[0]
            mid_l = nearest_blue[1] - self.half_track_width
        elif nearest_yellow:
            mid_f = nearest_yellow[0]
            mid_l = nearest_yellow[1] + self.half_track_width
        else:
            return self._last_steer, self.min_speed

        angle = math.atan2(mid_l, mid_f)
        steer = max(-self.max_steer,
                     min(self.max_steer, self.steering_gain * angle))
        self._last_steer = steer

        speed = self.max_speed * (1.0 - self.speed_curve_factor * abs(steer))
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.get_logger().info(
            f"[geo] angle={math.degrees(angle):.1f}° steer={steer:.3f} "
            f"speed={speed:.1f} blue={nearest_blue} yellow={nearest_yellow}"
        )
        return steer, speed

    # ── pure pursuit controller ─────────────────────────────────────────

    def _control_pure_pursuit(self, cones):
        """@brief Pure pursuit: build centreline from cone pairs, follow with adaptive lookahead.

        Uses nearest-neighbor pairing in 2D (not just distance), sorts midpoints
        along the path by forward distance, interpolates a target at a speed-adaptive
        lookahead, and applies the pure pursuit steering law.

        @param cones List of (class_id, fwd, left) tuples in camera_link frame.
        @return Tuple of (steer_rad, speed_mps).
        """
        WHEELBASE = 1.05
        LOOKAHEAD_MIN = 3.0
        LOOKAHEAD_MAX = 5.0
        CONE_RANGE = 8.0  # only consider cones within this range

        blues = []
        yellows = []
        for cls, fwd, left in cones:
            if fwd < 0.3:
                continue
            if math.hypot(fwd, left) > CONE_RANGE:
                continue
            if cls == "blue_cone":
                blues.append((fwd, left))
            elif cls == "yellow_cone":
                yellows.append((fwd, left))

        blues.sort(key=lambda c: c[0])    # sort by forward distance
        yellows.sort(key=lambda c: c[0])

        # Pair blue/yellow cones by nearest 2D distance
        midpoints = []
        if blues and yellows:
            used_y = set()
            for bx, by in blues:
                best_j, best_dd = -1, float('inf')
                for j, (yx, yy) in enumerate(yellows):
                    if j in used_y:
                        continue
                    dd = math.hypot(bx - yx, by - yy)
                    if dd < best_dd:
                        best_dd = dd
                        best_j = j
                if best_j >= 0 and best_dd < 6.0:
                    yx, yy = yellows[best_j]
                    used_y.add(best_j)
                    midpoints.append(((bx + yx) / 2.0, (by + yy) / 2.0))
                else:
                    midpoints.append((bx, by - self.half_track_width))
            for j, (yx, yy) in enumerate(yellows):
                if j not in used_y:
                    midpoints.append((yx, yy + self.half_track_width))
        elif blues:
            for bx, by in blues:
                midpoints.append((bx, by - self.half_track_width))
        elif yellows:
            for yx, yy in yellows:
                midpoints.append((yx, yy + self.half_track_width))

        if not midpoints:
            return self._last_steer, self.min_speed

        # Sort by distance from kart (origin)
        midpoints.sort(key=lambda p: math.hypot(p[0], p[1]))

        # Adaptive lookahead from kart: shorter when turning, longer when straight
        abs_last_steer = abs(self._last_steer)
        steer_ratio = abs_last_steer / self.max_steer  # 0 = straight, 1 = max turn
        lookahead = LOOKAHEAD_MAX - (LOOKAHEAD_MAX - LOOKAHEAD_MIN) * steer_ratio

        # Pick the midpoint closest to the lookahead distance from the kart
        target_x, target_y = midpoints[0]
        best_diff = float('inf')
        for mx, my in midpoints:
            d = math.hypot(mx, my)
            diff = abs(d - lookahead)
            if diff < best_diff:
                best_diff = diff
                target_x, target_y = mx, my

        # Pure pursuit steering law
        alpha = math.atan2(target_y, target_x)
        ld = math.hypot(target_x, target_y)
        if ld < 0.5:
            ld = 0.5
        steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)
        steer = self.steering_gain * steer
        steer = max(-self.max_steer, min(self.max_steer, steer))
        self._last_steer = steer

        # Speed: same profile as geometric
        speed = self.max_speed * (1.0 - self.speed_curve_factor * abs(steer))
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.get_logger().info(
            f"[pp] target=({target_x:.1f},{target_y:.1f}) ld={ld:.1f} "
            f"steer={steer:.3f} speed={speed:.1f} midpts={len(midpoints)}"
        )
        return steer, speed

    # ── neural net controller ─────────────────────────────────────────

    def _control_neural(self, cones):
        """@brief Neural net controller: feed-forward network produces steer and speed.

        @param cones List of (class_id, fwd, left) tuples in camera_link frame.
        @return Tuple of (steer_rad, speed_mps).
        """
        blues = []
        yellows = []
        for cls, fwd, left in cones:
            dist = math.hypot(fwd, left)
            angle = math.atan2(left, fwd)
            if cls == "blue_cone":
                blues.append((dist, angle))
            elif cls == "yellow_cone":
                yellows.append((dist, angle))
        blues.sort()
        yellows.sort()

        nb = self._nn_n_blue
        ny = self._nn_n_yellow
        inp = np.zeros(self._nn_input_size)
        for j, (d, a) in enumerate(blues[:nb]):
            inp[j * 2] = d / 15.0
            inp[j * 2 + 1] = a / np.pi
        for j, (d, a) in enumerate(yellows[:ny]):
            inp[nb * 2 + j * 2] = d / 15.0
            inp[nb * 2 + j * 2 + 1] = a / np.pi
        if self._nn_uses_speed:
            inp[-1] = self._actual_speed / self._nn_max_speed

        hidden = np.tanh(inp @ self._nn_W1 + self._nn_b1)
        out = hidden @ self._nn_W2 + self._nn_b2

        steer = float(np.tanh(out[0])) * self._nn_max_steer
        speed = float(1.0 / (1.0 + np.exp(-out[1]))) * self._nn_max_speed
        speed = max(self.min_speed, min(self.max_speed, speed))

        self._last_steer = steer
        self.get_logger().info(
            f"[{self.controller_type}] steer={steer:.3f} cmd_spd={speed:.1f} "
            f"act_spd={self._actual_speed:.1f} "
            f"blues={len(blues)} yellows={len(yellows)}"
        )
        return steer, speed

    # ── safety timeout ────────────────────────────────────────────────

    def _safety_check(self):
        """@brief Timer callback: publish zero-velocity if no detections received within timeout."""
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > self.no_cone_timeout:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


def main():
    """@brief Entrypoint for the cone follower node."""
    rclpy.init()
    node = ConeFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
