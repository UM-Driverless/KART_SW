#!/usr/bin/env python3
"""Cone-following controller for the kart.

Supports four controller types (selected via the ``controller_type`` param):

**geometric**
    Nearest blue/yellow midpoint → atan2 → steer.  Six tunable params.

**neural**
    Small feed-forward net (8→8→2), 90 weights.

**neural_v2**
    Larger net (17→16→2) with 4 cones per side + speed feedback, 322 weights.
    Trained with lap-time fitness for faster driving.

**mpc**
    Kinematic bicycle model MPC. Builds a midpoint reference path from cone
    pairs, then minimises lateral cross-track error + steering-rate over a
    receding horizon using scipy SLSQP.  Path points are re-expressed in the
    bicycle model's X-forward frame (camera Z→model X, camera -X→model Y) so
    the vehicle dynamics and the reference are in a consistent coordinate
    system.  Key tunable params:

    * ``mpc_horizon``       – prediction steps  (default 8)
    * ``mpc_dt``            – step duration [s] (default 0.10, matches 10 Hz)
    * ``mpc_w_cte``         – cross-track-error weight (default 3.0)
    * ``mpc_w_dsteer``      – steering-rate weight     (default 40.0)
    * ``mpc_w_heading``     – heading-error weight     (default 2.0)
    * ``mpc_lookahead``     – max path distance used   (default 8.0 m)

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
    from scipy.optimize import minimize as scipy_minimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    from kart_perception.zed_od_utils import HAS_ZED_INTERFACES, zed_objects_to_det3d
    if HAS_ZED_INTERFACES:
        from zed_interfaces.msg import ObjectsStamped
except ImportError:
    HAS_ZED_INTERFACES = False


# ---------------------------------------------------------------------------
# Kinematic bicycle model helpers (used by MPC)
# ---------------------------------------------------------------------------

def _bicycle_step(x, y, psi, v, delta, wheelbase, dt):
    """Propagate kinematic bicycle model one step forward.

    @param x        Current x position [m] in the controller's local frame.
    @param y        Current y position [m].
    @param psi      Current heading [rad].
    @param v        Speed [m/s] (held constant over the horizon).
    @param delta    Steering angle [rad].
    @param wheelbase Kart wheelbase [m].
    @param dt       Time step [s].
    @return Tuple (x_next, y_next, psi_next).
    """
    x_next   = x   + v * math.cos(psi) * dt
    y_next   = y   + v * math.sin(psi) * dt
    psi_next = psi + (v / wheelbase) * math.tan(delta) * dt
    return x_next, y_next, psi_next


def _build_midpoint_path(cones, half_track_width):
    """Pair blue/yellow cones and return a sorted list of midpoints.

    Mirrors the pairing logic used in the pure-pursuit controller so all
    controllers share the same path representation.

    @param cones            List of (class_id, fwd, left) in camera_link frame.
    @param half_track_width Fallback offset when only one colour is visible [m].
    @return List of (fwd, left) midpoints sorted by increasing forward distance.
    """
    blues   = [(fwd, left, math.hypot(fwd, left))
               for cls, fwd, left in cones if cls == "blue_cone"   and fwd > 0.3]
    yellows = [(fwd, left, math.hypot(fwd, left))
               for cls, fwd, left in cones if cls == "yellow_cone" and fwd > 0.3]

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
                    best_j  = j
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
    """Estimate path heading at *idx* using finite differences.

    @param midpoints Sorted list of (fwd, left) path points.
    @param idx       Index of the point whose heading is needed.
    @return Heading angle [rad] relative to the forward axis.
    """
    n = len(midpoints)
    if n < 2:
        return 0.0
    i0 = max(0, idx - 1)
    i1 = min(n - 1, idx + 1)
    dx = midpoints[i1][0] - midpoints[i0][0]
    dy = midpoints[i1][1] - midpoints[i0][1]
    return math.atan2(dy, dx)


def _nearest_path_point(midpoints, x, y):
    """Return the index of the closest midpoint to (x, y).

    @param midpoints Sorted list of (fwd, left) path points.
    @param x         Query x coordinate [m].
    @param y         Query y coordinate [m].
    @return Index into midpoints.
    """
    best_i, best_d = 0, float("inf")
    for i, (px, py) in enumerate(midpoints):
        d = math.hypot(px - x, py - y)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


# ---------------------------------------------------------------------------
# MPC cost function
# ---------------------------------------------------------------------------

def _mpc_cost(u_flat, x0, y0, psi0, v, midpoints,
               N, dt, wheelbase,
               w_cte, w_dsteer, w_heading,
               prev_steer, max_steer):
    """Compute the MPC objective for a candidate control sequence.

    @param u_flat    Flat array of N steering angles [rad].
    @param x0        Initial x [m].
    @param y0        Initial y [m].
    @param psi0      Initial heading [rad].
    @param v         Constant speed over horizon [m/s].
    @param midpoints Reference path points (fwd, left).
    @param N         Horizon length.
    @param dt        Time step [s].
    @param wheelbase Kart wheelbase [m].
    @param w_cte     Cross-track error weight.
    @param w_dsteer  Steering-rate weight.
    @param w_heading Heading error weight.
    @param prev_steer Previous steering command [rad] (for rate penalty).
    @param max_steer  Steering limit [rad] (soft barrier, not enforced here).
    @return Scalar cost.
    """
    x, y, psi = x0, y0, psi0
    cost = 0.0
    prev_u = prev_steer

    for k in range(N):
        delta = float(u_flat[k])

        # Cross-track error: signed distance to nearest path segment
        idx = _nearest_path_point(midpoints, x, y)
        px, py = midpoints[idx]
        cte = math.hypot(x - px, y - py)

        # Heading error relative to local path direction
        path_psi = _path_heading(midpoints, idx)
        heading_err = psi - path_psi
        # Wrap to [-pi, pi]
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        # Steering rate penalty
        d_steer = delta - prev_u

        cost += w_cte    * cte          ** 2
        cost += w_heading * heading_err ** 2
        cost += w_dsteer  * d_steer     ** 2

        x, y, psi = _bicycle_step(x, y, psi, v, delta, wheelbase, dt)
        prev_u = delta

    return cost


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class ConeFollowerNode(Node):
    """@brief Cone-following controller node.

    Supports geometric, neural (v1), neural_v2, and mpc controller types.
    Receives Detection3DArray in camera optical frame and publishes Twist on
    /kart/cmd_vel.
    """

    WHEELBASE = 1.05  # [m]

    def __init__(self):
        """@brief Initialize the controller with parameters, neural weights (if applicable), and ROS plumbing."""
        super().__init__("cone_follower")

        # --- common params ---
        self.declare_parameter("detections_topic", "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
        self.declare_parameter("no_cone_timeout", 1.0)
        self.declare_parameter("controller_type", "geometric")  # geometric|pure_pursuit|neural|neural_v2|mpc
        self.declare_parameter("weights_json", "")               # path for neural

        # --- geometric params ---
        self.declare_parameter("steering_gain", 3.0)
        self.declare_parameter("max_steer", 1.047)
        self.declare_parameter("max_speed", 2.625)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("lookahead_max", 15.0)
        self.declare_parameter("half_track_width", 1.5)
        self.declare_parameter("speed_curve_factor", 0.0)

        # --- MPC params ---
        # mpc_dt: set to 1.0–1.5× your actual detection callback period.
        #         Default assumes ~10 Hz detections.  Tune first if oscillating.
        self.declare_parameter("mpc_horizon",    8)      # shorter = faster solve, less stale
        self.declare_parameter("mpc_dt",          0.10)  # matches confirmed 10 Hz detection rate
        self.declare_parameter("mpc_w_cte",       3.0)   # reduced: don't chase path too hard
        self.declare_parameter("mpc_w_dsteer",   40.0)   # high: primary anti-oscillation knob
        self.declare_parameter("mpc_w_heading",   2.0)
        self.declare_parameter("mpc_lookahead",   8.0)   # trim path to this distance [m]

        det_topic = str(self.get_parameter("detections_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.no_cone_timeout   = float(self.get_parameter("no_cone_timeout").value)
        self.controller_type   = str(self.get_parameter("controller_type").value)

        # geometric fields
        self.steering_gain     = float(self.get_parameter("steering_gain").value)
        self.max_steer         = float(self.get_parameter("max_steer").value)
        self.max_speed         = float(self.get_parameter("max_speed").value)
        self.min_speed         = float(self.get_parameter("min_speed").value)
        self.lookahead_max     = float(self.get_parameter("lookahead_max").value)
        self.half_track_width  = float(self.get_parameter("half_track_width").value)
        self.speed_curve_factor = float(self.get_parameter("speed_curve_factor").value)

        # MPC fields
        self.mpc_N          = int(self.get_parameter("mpc_horizon").value)
        self.mpc_dt         = float(self.get_parameter("mpc_dt").value)
        self.mpc_w_cte      = float(self.get_parameter("mpc_w_cte").value)
        self.mpc_w_dsteer   = float(self.get_parameter("mpc_w_dsteer").value)
        self.mpc_w_heading  = float(self.get_parameter("mpc_w_heading").value)
        self.mpc_lookahead  = float(self.get_parameter("mpc_lookahead").value)
        # Shifted warm-start: reuse previous solution to avoid cold re-solve oscillation
        self._mpc_prev_solution: np.ndarray | None = None

        if self.controller_type == "mpc" and not HAS_SCIPY:
            self.get_logger().error("controller_type=mpc but scipy is not installed. "
                                    "Run: pip install scipy")
            raise SystemExit(1)

        # neural net weights (loaded for neural or neural_v2)
        self._nn_W1 = self._nn_b1 = self._nn_W2 = self._nn_b2 = None
        self._nn_max_steer   = 0.5
        self._nn_max_speed   = 10.0
        self._nn_input_size  = 8
        self._nn_n_blue      = 2
        self._nn_n_yellow    = 2
        self._nn_uses_speed  = False
        self._current_speed  = 0.0

        if self.controller_type in ("neural", "neural_v2"):
            self._load_neural_weights()

        # ROS plumbing
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )
        if HAS_ZED_INTERFACES:
            self.declare_parameter("zed_objects_topic", "/zed/zed_node/obj_det/objects")
            self.create_subscription(
                ObjectsStamped,
                str(self.get_parameter("zed_objects_topic").value),
                lambda msg: self._on_detections(zed_objects_to_det3d(msg)),
                10,
            )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/model/kart/odom_gt", self._on_odom, odom_qos
        )
        self._actual_speed = 0.0
        self._last_steer   = 0.0
        import time as _time
        self._last_det_wall      = _time.monotonic()
        self._last_rate_log_wall = _time.monotonic()
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._safety_check)
        self.create_subscription(String, "/dashboard/controller_type",
                                 self._on_controller_type, 10)

        self.get_logger().info(f"Controller type: {self.controller_type}")

    # ── runtime controller switching ──────────────────────────────────

    def _on_controller_type(self, msg: String):
        """@brief Callback for runtime controller type changes from the dashboard."""
        new_type = msg.data
        valid = ("geometric", "pure_pursuit", "neural", "neural_v2", "mpc")
        if new_type in valid and new_type != self.controller_type:
            self.get_logger().info(
                f"Controller type: {self.controller_type} → {new_type}")
            self.controller_type = new_type
            if new_type == "mpc" and not HAS_SCIPY:
                self.get_logger().error("scipy not installed – cannot switch to mpc")
                self.controller_type = "geometric"
                return
            if new_type in ("neural", "neural_v2") and self._nn_W1 is None:
                self._load_neural_weights()

    def _on_odom(self, msg: Odometry):
        """@brief Callback for odometry. Extracts current speed for neural_v2 and MPC."""
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
            self._nn_input_size = 17
            self._nn_n_blue     = 4
            self._nn_n_yellow   = 4
            self._nn_uses_speed = True
            hs = 16
            i = 0
            self._nn_W1 = genes[i:i + 17 * hs].reshape(17, hs);  i += 17 * hs
            self._nn_b1 = genes[i:i + hs];                        i += hs
            self._nn_W2 = genes[i:i + hs * 2].reshape(hs, 2);    i += hs * 2
            self._nn_b2 = genes[i:i + 2]
        else:
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
        import time as _time
        _now_wall = _time.monotonic()
        _dt_wall_ms = (_now_wall - self._last_det_wall) * 1000.0
        self._last_det_wall = _now_wall
        self.last_detection_time = self.get_clock().now()
        # Log actual callback rate once per second (wall clock, not sim time).
        # Set mpc_dt to the reported recommended value if oscillation persists.
        if self.controller_type == "mpc" and _dt_wall_ms > 1.0:
            _now_log = _time.monotonic()
            if _now_log - self._last_rate_log_wall > 1.0:
                self._last_rate_log_wall = _now_log
                self.get_logger().info(
                    f"[mpc] callback_dt={_dt_wall_ms:.1f}ms "
                    f"({1000.0/_dt_wall_ms:.1f} Hz)  "
                    f"-> recommended mpc_dt={_dt_wall_ms * 1.25 / 1000:.3f}s"
                )

        cones = []
        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos      = det.results[0].pose.pose.position
            fwd      = pos.z
            left     = -pos.x
            if fwd < 0.5:
                continue
            dist  = math.hypot(fwd, left)
            if dist > 15.0:
                continue
            angle = abs(math.atan2(left, fwd))
            if angle > 0.6109:
                continue
            cones.append((class_id, fwd, left))

        if self.controller_type in ("neural", "neural_v2"):
            steer, speed = self._control_neural(cones)
        elif self.controller_type == "pure_pursuit":
            steer, speed = self._control_pure_pursuit(cones)
        elif self.controller_type == "mpc":
            steer, speed = self._control_mpc(cones)
        else:
            steer, speed = self._control_geometric(cones)

        if not cones:
            speed = 0.0
            steer = 0.0
            if self.controller_type == "mpc":
                self.get_logger().warn(
                    "[mpc] no cones visible — holding zero cmd. "
                    "Check perception node output."
                )

        cmd = Twist()
        cmd.angular.z = steer
        cmd.linear.x  = speed
        self.cmd_pub.publish(cmd)

    # ── geometric controller ──────────────────────────────────────────

    def _control_geometric(self, cones):
        """@brief Geometric controller: steer toward nearest blue/yellow midpoint.

        @param cones List of (class_id, fwd, left) tuples in camera_link frame.
        @return Tuple of (steer_rad, speed_mps).
        """
        nearest_blue   = None
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

    # ── pure pursuit controller ───────────────────────────────────────

    def _control_pure_pursuit(self, cones):
        """@brief Pure pursuit: build path from all cone pairs, follow with lookahead.

        @param cones List of (class_id, fwd, left) tuples in camera_link frame.
        @return Tuple of (steer_rad, speed_mps).
        """
        blues   = []
        yellows = []
        for cls, fwd, left in cones:
            if fwd < 0.3:
                continue
            dist = math.hypot(fwd, left)
            if cls == "blue_cone":
                blues.append((fwd, left, dist))
            elif cls == "yellow_cone":
                yellows.append((fwd, left, dist))

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
                        best_j  = j
                if best_j >= 0 and best_dd < 8.0:
                    yx, yy, _ = yellows[best_j]
                    used_y.add(best_j)
                    midpoints.append(((bx + yx) / 2.0, (by + yy) / 2.0))
                else:
                    midpoints.append((bx, by - self.half_track_width))
            for j, (yx, yy, _) in enumerate(yellows):
                if j not in used_y:
                    midpoints.append((yx, yy + self.half_track_width))
        elif blues:
            for bx, by, _ in blues:
                midpoints.append((bx, by - self.half_track_width))
        elif yellows:
            for yx, yy, _ in yellows:
                midpoints.append((yx, yy + self.half_track_width))

        if not midpoints:
            return self._last_steer, self.min_speed

        midpoints.sort(key=lambda p: p[0])

        target_x, target_y = midpoints[0]
        cum_dist = 0.0
        for i in range(len(midpoints)):
            if i > 0:
                dx = midpoints[i][0] - midpoints[i - 1][0]
                dy = midpoints[i][1] - midpoints[i - 1][1]
                cum_dist += math.hypot(dx, dy)
            if cum_dist >= self.lookahead_max:
                target_x, target_y = midpoints[i]
                break
            target_x, target_y = midpoints[i]

        alpha  = math.atan2(target_y, target_x)
        ld     = math.hypot(target_x, target_y)
        ld     = max(0.5, ld)
        steer  = math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), ld)
        steer  = max(-self.max_steer, min(self.max_steer, steer))
        self._last_steer = steer

        speed = self.max_speed * (1.0 - self.speed_curve_factor * abs(steer))
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.get_logger().info(
            f"[pp] target=({target_x:.1f},{target_y:.1f}) steer={steer:.3f} "
            f"speed={speed:.1f} midpoints={len(midpoints)}"
        )
        return steer, speed

    # ── MPC controller ────────────────────────────────────────────────

    def _control_mpc(self, cones):
        """@brief Model Predictive Control: kinematic bicycle model over a receding horizon.

        Builds a midpoint reference path from paired blue/yellow cones, converts
        it into the bicycle model's X-forward/Y-left frame (camera fwd→X,
        camera left→Y), trims it to mpc_lookahead metres, then minimises a cost
        that penalises cross-track error, heading error, and steering rate.

        Frame mapping (camera optical → bicycle model):
            camera  Z (forward) → model  X
            camera -X (left)    → model  Y

        Because the kart is at the frame origin with ψ=0 pointing along +X,
        the untransformed midpoints (fwd, left) already map directly to (X, Y)
        in the bicycle model's local frame — this is the frame that was missing
        before and causing the oscillation.

        @param cones List of (class_id, fwd, left) tuples in camera_link frame.
        @return Tuple of (steer_rad, speed_mps).
        """
        # Build raw midpoints in camera frame: (fwd=Z, left=-X)
        raw_midpoints = _build_midpoint_path(cones, self.half_track_width)

        if len(raw_midpoints) < 2:
            return self._last_steer, self.min_speed

        # --- Frame conversion + lookahead trim ---
        # Camera (fwd, left) maps directly to bicycle model (x, y):
        #   model_x = fwd   (forward is +X in bicycle frame)
        #   model_y = left  (left is +Y in bicycle frame)
        # Trim to mpc_lookahead to avoid distant, noisy points pulling the cost.
        path: list[tuple[float, float]] = []
        cum = 0.0
        prev = (0.0, 0.0)
        for fwd, left in raw_midpoints:
            mx, my = fwd, left          # direct mapping — fwd→X, left→Y
            cum += math.hypot(mx - prev[0], my - prev[1])
            if cum > self.mpc_lookahead:
                break
            path.append((mx, my))
            prev = (mx, my)

        if len(path) < 2:
            # Only one point visible within lookahead — fall back gracefully
            return self._last_steer, self.min_speed

        # Use odometry speed; clamp to operational range
        v = max(self.min_speed, min(self.max_speed, self._actual_speed))

        N      = self.mpc_N
        dt     = self.mpc_dt
        bounds = [(-self.max_steer, self.max_steer)] * N

        # --- Shifted warm-start ---
        # Reuse the tail of the previous solution and append the last value,
        # rather than repeating _last_steer N times.  This gives the optimizer
        # a much better starting point and dramatically reduces oscillation.
        if self._mpc_prev_solution is not None and len(self._mpc_prev_solution) == N:
            u0 = np.empty(N)
            u0[:-1] = self._mpc_prev_solution[1:]
            u0[-1]  = self._mpc_prev_solution[-1]
        else:
            u0 = np.full(N, self._last_steer)

        import time as _time
        _t0 = _time.monotonic()
        result = scipy_minimize(
            _mpc_cost,
            u0,
            args=(
                0.0, 0.0, 0.0,    # x0, y0, psi0 — always at local origin
                v,
                path,
                N, dt, self.WHEELBASE,
                self.mpc_w_cte,
                self.mpc_w_dsteer,
                self.mpc_w_heading,
                self._last_steer,
                self.max_steer,
            ),
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": 60, "ftol": 1e-4},
        )
        _solve_ms = (_time.monotonic() - _t0) * 1000

        # Store solution for next warm-start
        self._mpc_prev_solution = result.x.copy()

        # Apply only the first control action (receding horizon)
        steer = float(np.clip(result.x[0], -self.max_steer, self.max_steer))
        self._last_steer = steer

        speed = self.max_speed * (1.0 - self.speed_curve_factor * abs(steer))
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.get_logger().info(
            f"[mpc] steer={steer:.3f} speed={speed:.1f} "
            f"pts={len(path)} ok={result.success} itr={result.nit} solve={_solve_ms:.1f}ms"
        )
        return steer, speed

    # ── neural net controller ─────────────────────────────────────────

    def _control_neural(self, cones):
        """@brief Neural net controller: feed-forward network produces steer and speed.

        @param cones List of (class_id, fwd, left) tuples in camera_link frame.
        @return Tuple of (steer_rad, speed_mps).
        """
        blues   = []
        yellows = []
        for cls, fwd, left in cones:
            dist  = math.hypot(fwd, left)
            angle = math.atan2(left, fwd)
            if cls == "blue_cone":
                blues.append((dist, angle))
            elif cls == "yellow_cone":
                yellows.append((dist, angle))
        blues.sort()
        yellows.sort()

        nb  = self._nn_n_blue
        ny  = self._nn_n_yellow
        inp = np.zeros(self._nn_input_size)
        for j, (d, a) in enumerate(blues[:nb]):
            inp[j * 2]     = d / 15.0
            inp[j * 2 + 1] = a / np.pi
        for j, (d, a) in enumerate(yellows[:ny]):
            inp[nb * 2 + j * 2]     = d / 15.0
            inp[nb * 2 + j * 2 + 1] = a / np.pi
        if self._nn_uses_speed:
            inp[-1] = self._actual_speed / self._nn_max_speed

        hidden = np.tanh(inp @ self._nn_W1 + self._nn_b1)
        out    = hidden @ self._nn_W2 + self._nn_b2

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
            cmd.linear.x  = 0.0
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