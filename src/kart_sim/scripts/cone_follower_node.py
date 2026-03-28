#!/usr/bin/env python3
"""MPC controller para kart — versión anti-oscilación en curvas encadenadas.

Fixes respecto a la versión anterior:

  FIX-1: Horizonte extendido + dt más largo
    N=15, dt=0.15s → 2.25s de anticipación (~4.5m a 2m/s).
    Permite ver curvas encadenadas dentro del mismo horizonte.

  FIX-2: Warm-start robusto con fallback parcial
    Si el optimizador falla (success=False), en lugar de tirar la solución
    entera se acepta igualmente y se guarda para el siguiente warm-start.
    Así nunca se cae en el ciclo vicioso fallo → cold-start → fallo.

  FIX-3: Velocidad constante en el modelo de predicción
    En lugar del filtro de primer orden que hacía que el modelo predijera
    aceleración, se usa la velocidad real medida como constante durante
    todo el horizonte. Más conservador y exacto para dt cortos.

  FIX-4: Peso w_dsteer adaptativo según curvatura del path
    En recta se penaliza más el steering rate (oscilaciones finas).
    En curva se relaja para que el optimizador pueda girar libremente.
    Factor: w_dsteer = w_dsteer_base / (1 + 8*kappa_max)

  FIX-5: Filtro IIR sobre el steer publicado
    steer_pub = alpha * steer_prev + (1-alpha) * steer_optimo
    alpha=0.55 amortigua oscilaciones rápidas sin introducir retardo excesivo.

Parámetros recomendados (valores por defecto):
    mpc_horizon      = 15
    mpc_dt           = 0.12
    mpc_w_cte        = 4.0
    mpc_w_heading    = 2.5
    mpc_w_dsteer     = 25.0   (base; se adapta en curva)
    mpc_lookahead    = 10.0
    mpc_target_speed = 2.0
    mpc_steer_alpha  = 0.55
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray

try:
    from scipy.optimize import minimize as scipy_minimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

WHEELBASE        = 1.25
MAX_STEER        = 1.047
MAX_SPEED        = 2.625
MIN_SPEED        = 0.5
HALF_TRACK_WIDTH = 1.5


# ---------------------------------------------------------------------------
# Modelo cinemático de bicicleta
# ---------------------------------------------------------------------------

def bicycle_step(
    x: float, y: float, psi: float,
    v: float, delta: float,
    wheelbase: float, dt: float
) -> tuple[float, float, float]:
    """Propaga el modelo cinemático de bicicleta un paso con velocidad constante."""
    x_next   = x   + v * math.cos(psi) * dt
    y_next   = y   + v * math.sin(psi) * dt
    psi_next = psi + (v / wheelbase) * math.tan(delta) * dt
    return x_next, y_next, psi_next


# ---------------------------------------------------------------------------
# Path de referencia
# ---------------------------------------------------------------------------

def build_midpoint_path(
    cones: list[tuple[str, float, float]],
    half_track_width: float
) -> list[tuple[float, float]]:
    """Empareja conos azul/amarillo y devuelve midpoints ordenados por distancia."""
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

    midpoints: list[tuple[float, float]] = []
    used_y: set[int] = set()

    if blues and yellows:
        for bx, by, bd in blues:
            best_j, best_dd = -1, float("inf")
            for j, (yx, yy, yd) in enumerate(yellows):
                if j in used_y:
                    continue
                dd = abs(bd - yd)
                if dd < best_dd:
                    best_dd, best_j = dd, j
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
        midpoints = [(bx, by - half_track_width) for bx, by, _ in blues]
    elif yellows:
        midpoints = [(yx, yy + half_track_width) for yx, yy, _ in yellows]

    midpoints.sort(key=lambda p: p[0])
    return midpoints


def build_arc_parameterization(
    path: list[tuple[float, float]]
) -> tuple[np.ndarray, np.ndarray]:
    """Parametriza el path por longitud de arco acumulada."""
    pts = np.array(path, dtype=np.float64)
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
    s = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    return s, pts


def interpolate_path(
    s_ref: np.ndarray,
    pts_np: np.ndarray,
    s_query: float
) -> tuple[float, float, float]:
    """Interpola posición y tangente en el arco s_query."""
    s_query = float(np.clip(s_query, s_ref[0], s_ref[-1]))
    idx = int(np.clip(
        np.searchsorted(s_ref, s_query, side='right') - 1,
        0, len(s_ref) - 2
    ))
    ds  = s_ref[idx + 1] - s_ref[idx]
    t   = float(np.clip((s_query - s_ref[idx]) / ds if ds > 1e-9 else 0.0, 0.0, 1.0))
    p0, p1 = pts_np[idx], pts_np[idx + 1]
    px  = p0[0] + t * (p1[0] - p0[0])
    py  = p0[1] + t * (p1[1] - p0[1])
    psi = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return px, py, psi


# ---------------------------------------------------------------------------
# FIX-4: Estimador de curvatura
# ---------------------------------------------------------------------------

def estimate_path_curvature(s_ref: np.ndarray, pts_np: np.ndarray) -> float:
    """Curvatura máxima del path visible (kappa = dpsi/ds).

    Retorna 0 en recta, ~0.5 en curva cerrada típica de kart.
    Usado para adaptar w_dsteer: más curva → menos penalización de steering rate.
    """
    if len(pts_np) < 3:
        return 0.0
    kappas = []
    for i in range(1, len(pts_np) - 1):
        p0, p1, p2 = pts_np[i - 1], pts_np[i], pts_np[i + 1]
        psi0 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        psi1 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        dpsi = math.atan2(math.sin(psi1 - psi0), math.cos(psi1 - psi0))
        ds   = s_ref[i + 1] - s_ref[i - 1]
        if ds > 1e-6:
            kappas.append(abs(dpsi / ds))
    return max(kappas) if kappas else 0.0


# ---------------------------------------------------------------------------
# Función de coste MPC
# ---------------------------------------------------------------------------

def mpc_cost(
    u_flat: np.ndarray,
    x0: float, y0: float, psi0: float, v: float,
    s_ref: np.ndarray, pts_np: np.ndarray,
    N: int, dt: float, wheelbase: float,
    w_cte: float, w_heading: float, w_dsteer: float,
    prev_steer: float,
) -> float:
    """Coste MPC con velocidad constante en el horizonte (FIX-3)."""
    x, y, psi = x0, y0, psi0
    cost      = 0.0
    prev_u    = prev_steer
    s_vehicle = 0.0

    for k in range(N):
        delta = float(u_flat[k])

        ref_x, ref_y, ref_psi = interpolate_path(s_ref, pts_np, s_vehicle)

        dx  = x - ref_x
        dy  = y - ref_y
        cte = -dx * math.sin(ref_psi) + dy * math.cos(ref_psi)

        heading_err = math.atan2(
            math.sin(psi - ref_psi),
            math.cos(psi - ref_psi)
        )

        d_steer = delta - prev_u

        cost += w_cte     * cte         ** 2
        cost += w_heading * heading_err ** 2
        cost += w_dsteer  * d_steer     ** 2

        x, y, psi = bicycle_step(x, y, psi, v, delta, wheelbase, dt)
        s_vehicle += v * dt
        prev_u = delta

    return cost


# ---------------------------------------------------------------------------
# Nodo ROS2
# ---------------------------------------------------------------------------

class KartMPCNode(Node):
    """Controlador MPC anti-oscilación para kart autónomo."""

    def __init__(self):
        super().__init__("kart_mpc")

        self.declare_parameter("detections_topic",   "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic",      "/kart/cmd_vel")
        self.declare_parameter("no_cone_timeout",    1.0)

        self.declare_parameter("mpc_horizon",        15)     # FIX-1
        self.declare_parameter("mpc_dt",             0.12)   # FIX-1
        self.declare_parameter("mpc_w_cte",          4.0)
        self.declare_parameter("mpc_w_heading",      2.5)
        self.declare_parameter("mpc_w_dsteer",       25.0)   # FIX-4: valor base
        self.declare_parameter("mpc_lookahead",      10.0)   # FIX-1
        self.declare_parameter("mpc_target_speed",   2.0)
        self.declare_parameter("mpc_max_iter_cold",  300)
        self.declare_parameter("mpc_max_iter_warm",  80)
        self.declare_parameter("mpc_ftol",           1e-4)
        self.declare_parameter("mpc_steer_alpha",    0.55)   # FIX-5

        self.declare_parameter("wheelbase",          WHEELBASE)
        self.declare_parameter("max_steer",          MAX_STEER)
        self.declare_parameter("max_speed",          MAX_SPEED)
        self.declare_parameter("min_speed",          MIN_SPEED)
        self.declare_parameter("half_track_width",   HALF_TRACK_WIDTH)

        g = self.get_parameter
        self.N             = int(g("mpc_horizon").value)
        self.dt            = float(g("mpc_dt").value)
        self.w_cte         = float(g("mpc_w_cte").value)
        self.w_heading     = float(g("mpc_w_heading").value)
        self.w_dsteer_base = float(g("mpc_w_dsteer").value)
        self.lookahead     = float(g("mpc_lookahead").value)
        self.v_target      = float(g("mpc_target_speed").value)
        self.maxiter_cold  = int(g("mpc_max_iter_cold").value)
        self.maxiter_warm  = int(g("mpc_max_iter_warm").value)
        self.ftol          = float(g("mpc_ftol").value)
        self.steer_alpha   = float(g("mpc_steer_alpha").value)
        self.wheelbase     = float(g("wheelbase").value)
        self.max_steer     = float(g("max_steer").value)
        self.max_speed     = float(g("max_speed").value)
        self.min_speed     = float(g("min_speed").value)
        self.htw           = float(g("half_track_width").value)
        det_topic          = str(g("detections_topic").value)
        cmd_topic          = str(g("cmd_vel_topic").value)
        self.no_cone_timeout = float(g("no_cone_timeout").value)

        if not HAS_SCIPY:
            self.get_logger().error("scipy no instalado: pip install scipy")
            raise SystemExit(1)

        self._actual_speed:  float             = 0.0
        self._last_steer:    float             = 0.0   # steer publicado (post-IIR)
        self._raw_steer:     float             = 0.0   # steer del optimizador
        self._prev_solution: np.ndarray | None = None
        self._last_det_wall: float             = time.monotonic()
        self._last_log_wall: float             = time.monotonic()
        self.last_det_time                     = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry, "/model/kart/odom_gt", self._on_odom, odom_qos
        )
        self.create_timer(0.1, self._safety_check)
        self.get_logger().info(
            f"KartMPCNode listo — N={self.N} dt={self.dt}s "
            f"horizonte={self.N * self.dt:.2f}s "
            f"v_target={self.v_target}m/s alpha={self.steer_alpha}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._actual_speed = math.sqrt(vx * vx + vy * vy)

    def _on_detections(self, msg: Detection3DArray) -> None:
        now_wall   = time.monotonic()
        dt_wall_ms = (now_wall - self._last_det_wall) * 1000.0
        self._last_det_wall = now_wall
        self.last_det_time  = self.get_clock().now()

        if now_wall - self._last_log_wall > 1.0:
            self._last_log_wall = now_wall
            self.get_logger().info(
                f"[mpc] cb={dt_wall_ms:.1f}ms "
                f"mpc_dt recomendado={dt_wall_ms * 1.2 / 1000:.3f}s"
            )

        cones: list[tuple[str, float, float]] = []
        for det in msg.detections:
            if not det.results:
                continue
            cls  = det.results[0].hypothesis.class_id
            pos  = det.results[0].pose.pose.position
            fwd  = pos.z
            left = -pos.x
            if fwd < 0.5:
                continue
            dist  = math.hypot(fwd, left)
            angle = abs(math.atan2(left, fwd))
            if dist > 15.0 or angle > 0.6109:
                continue
            cones.append((cls, fwd, left))

        steer, speed = self._control_mpc(cones)

        if not cones:
            steer, speed = 0.0, 0.0
            self._prev_solution = None
            self._raw_steer     = 0.0
            self._last_steer    = 0.0
            self.get_logger().warn("[mpc] sin conos — stop.")

        cmd = Twist()
        cmd.angular.z = steer
        cmd.linear.x  = speed
        self.cmd_pub.publish(cmd)

    def _control_mpc(
        self, cones: list[tuple[str, float, float]]
    ) -> tuple[float, float]:

        raw = build_midpoint_path(cones, self.htw)
        if len(raw) < 2:
            return self._last_steer, self.min_speed

        # Recortar a lookahead
        path: list[tuple[float, float]] = []
        cum, prev = 0.0, (0.0, 0.0)
        for fwd, left in raw:
            seg = math.hypot(fwd - prev[0], left - prev[1])
            cum += seg
            path.append((fwd, left))
            prev = (fwd, left)
            if cum > self.lookahead:
                break

        if len(path) < 2:
            return self._last_steer, self.min_speed

        s_ref, pts_np = build_arc_parameterization(path)

        # FIX-4: w_dsteer adaptativo
        max_kappa = estimate_path_curvature(s_ref, pts_np)
        w_dsteer  = self.w_dsteer_base / (1.0 + 8.0 * max_kappa)

        # FIX-3: velocidad constante = real medida
        v = max(self._actual_speed, self.min_speed)

        # Warm-start (FIX-2: siempre conservar solución anterior)
        if self._prev_solution is not None and len(self._prev_solution) == self.N:
            u0       = np.empty(self.N)
            u0[:-1]  = self._prev_solution[1:]
            u0[-1]   = self._prev_solution[-1]
            max_iter = self.maxiter_warm
        else:
            u0       = np.full(self.N, self._raw_steer)
            max_iter = self.maxiter_cold

        bounds = [(-self.max_steer, self.max_steer)] * self.N

        t0 = time.monotonic()
        result = scipy_minimize(
            mpc_cost,
            u0,
            args=(
                0.0, 0.0, 0.0, v,
                s_ref, pts_np,
                self.N, self.dt, self.wheelbase,
                self.w_cte, self.w_heading, w_dsteer,
                self._raw_steer,
            ),
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": max_iter, "ftol": self.ftol},
        )
        solve_ms = (time.monotonic() - t0) * 1000.0

        # FIX-2: guardar siempre, converja o no
        self._prev_solution = result.x.copy()
        self._raw_steer = float(np.clip(result.x[0], -self.max_steer, self.max_steer))

        # FIX-5: filtro IIR
        steer = float(np.clip(
            self.steer_alpha * self._last_steer + (1.0 - self.steer_alpha) * self._raw_steer,
            -self.max_steer, self.max_steer
        ))
        self._last_steer = steer

        # Velocidad coherente con el modelo
        curve_ratio = abs(steer) / self.max_steer
        speed = float(np.clip(
            self.v_target * (1.0 - 0.35 * curve_ratio ** 2),
            self.min_speed, self.max_speed
        ))

        if not result.success:
            self.get_logger().warn(
                f"[mpc] no convergió — itr={result.nit} J={result.fun:.3f} "
                f"usando solución parcial δ={math.degrees(self._raw_steer):.1f}°"
            )

        self.get_logger().info(
            f"[mpc] δ_raw={math.degrees(self._raw_steer):.1f}° "
            f"δ_pub={math.degrees(steer):.1f}° "
            f"v={speed:.2f} κ={max_kappa:.3f} w_ds={w_dsteer:.1f} "
            f"ok={result.success} itr={result.nit} J={result.fun:.4f} "
            f"t={solve_ms:.1f}ms"
        )
        return steer, speed

    def _safety_check(self) -> None:
        elapsed = (self.get_clock().now() - self.last_det_time).nanoseconds / 1e9
        if elapsed > self.no_cone_timeout:
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = KartMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
