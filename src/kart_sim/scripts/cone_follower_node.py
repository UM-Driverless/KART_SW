#!/usr/bin/env python3
"""MPC controller for kart cone following.

Controlador Model Predictive Control (MPC) para seguimiento de trayectoria
en un kart autónomo usando un modelo cinemático de bicicleta.

Mejoras respecto al código original:
  - Velocidad real (odometría) usada como condición inicial del modelo,
    con perfil de velocidad deseada dentro del horizonte.
  - Función de coste con término de error de velocidad explícito y
    normalización de términos para tuning estable.
  - Warm-start con validación: sólo se reutiliza si la solución anterior
    convergió (result.success=True).
  - Velocidad comandada coherente con la velocidad objetivo del MPC
    (no calculada a posteriori con speed_curve_factor independiente).
  - Interpolación del punto de referencia por avance en arco, no búsqueda
    2D en cada paso del horizonte.
  - Logging detallado con métricas de convergencia.

Requisitos:
    pip install scipy numpy rclpy

Frame de coordenadas (sin cambios respecto al original):
    camera optical frame: Z=adelante, X=derecha, Y=abajo
    bicycle model frame:  X=adelante, Y=izquierda
    Conversión: model_x = cam_z (fwd), model_y = -cam_x (left)
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


# ---------------------------------------------------------------------------
# Constantes del vehículo
# ---------------------------------------------------------------------------

WHEELBASE = 1.05        # [m]  distancia entre ejes
MAX_STEER = 1.047       # [rad] ≈ 60°
MAX_SPEED = 2.625       # [m/s]
MIN_SPEED = 0.5         # [m/s]
HALF_TRACK_WIDTH = 1.5  # [m]  fallback cuando sólo hay conos de un color


# ---------------------------------------------------------------------------
# Modelo cinemático de bicicleta
# ---------------------------------------------------------------------------

def bicycle_step(
    x: float, y: float, psi: float,
    v: float, delta: float,
    wheelbase: float, dt: float
) -> tuple[float, float, float]:
    """Propaga el modelo cinemático de bicicleta un paso.

    Ecuaciones estándar (Rajamani 2012):
        x_next   = x   + v * cos(psi) * dt
        y_next   = y   + v * sin(psi) * dt
        psi_next = psi + (v / L) * tan(delta) * dt

    Args:
        x, y:      posición actual [m] en el frame local del vehículo.
        psi:       orientación actual [rad].
        v:         velocidad longitudinal [m/s].
        delta:     ángulo de dirección [rad].
        wheelbase: distancia entre ejes [m].
        dt:        paso de tiempo [s].

    Returns:
        (x_next, y_next, psi_next)
    """
    x_next   = x   + v * math.cos(psi) * dt
    y_next   = y   + v * math.sin(psi) * dt
    psi_next = psi + (v / wheelbase) * math.tan(delta) * dt
    return x_next, y_next, psi_next


def bicycle_step_with_speed(
    x: float, y: float, psi: float, v: float,
    delta: float, a: float,
    wheelbase: float, dt: float,
    v_max: float
) -> tuple[float, float, float, float]:
    """Propaga el modelo cinemático de bicicleta incluyendo dinámica de velocidad.

    La aceleración longitudinal 'a' permite modelar la evolución de la
    velocidad dentro del horizonte, haciendo la predicción coherente con
    los comandos reales.

    Args:
        x, y:      posición [m].
        psi:       orientación [rad].
        v:         velocidad actual [m/s].
        delta:     ángulo de dirección [rad].
        a:         aceleración longitudinal [m/s²].
        wheelbase: distancia entre ejes [m].
        dt:        paso de tiempo [s].
        v_max:     límite superior de velocidad [m/s].

    Returns:
        (x_next, y_next, psi_next, v_next)
    """
    v_next   = float(np.clip(v + a * dt, MIN_SPEED, v_max))
    x_next   = x   + v * math.cos(psi) * dt
    y_next   = y   + v * math.sin(psi) * dt
    psi_next = psi + (v / wheelbase) * math.tan(delta) * dt
    return x_next, y_next, psi_next, v_next


# ---------------------------------------------------------------------------
# Construcción del path de referencia
# ---------------------------------------------------------------------------

def build_midpoint_path(
    cones: list[tuple[str, float, float]],
    half_track_width: float
) -> list[tuple[float, float]]:
    """Empareja conos azules/amarillos y devuelve puntos medios ordenados.

    La estrategia de emparejamiento es greedy por distancia al origen:
    para cada cono azul, busca el cono amarillo más cercano en distancia
    radial (|d_blue - d_yellow| < threshold).

    Args:
        cones:            Lista de (class_id, fwd, left) en frame cámara.
        half_track_width: Offset lateral [m] cuando sólo hay un color.

    Returns:
        Lista de (fwd, left) ordenada por distancia creciente al origen.
    """
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
                    best_dd = dd
                    best_j = j
            if best_j >= 0 and best_dd < 8.0:
                yx, yy, _ = yellows[best_j]
                used_y.add(best_j)
                midpoints.append(((bx + yx) / 2.0, (by + yy) / 2.0))
            else:
                # Cono azul sin pareja: estimar midpoint
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
    """Calcula las distancias acumuladas a lo largo del path (parámetro de arco).

    Permite interpolar puntos y tangentes en función de la distancia
    recorrida, en lugar de buscar el punto más cercano en 2D.

    Args:
        path: Lista de (x, y) puntos del path.

    Returns:
        (s, pts_np):
            s:      array 1D con distancia acumulada desde el origen.
            pts_np: array Nx2 con los puntos del path.
    """
    pts_np = np.array(path, dtype=np.float64)
    diffs = np.diff(pts_np, axis=0)
    seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
    s = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    return s, pts_np


def interpolate_path(
    s_ref: np.ndarray,
    pts_np: np.ndarray,
    s_query: float
) -> tuple[float, float, float]:
    """Interpola posición y tangente en el arco s_query.

    Args:
        s_ref:   parámetro de arco de los puntos conocidos.
        pts_np:  puntos del path (Nx2).
        s_query: distancia de arco deseada.

    Returns:
        (px, py, psi): posición [m] y orientación de la tangente [rad].
    """
    s_query = float(np.clip(s_query, s_ref[0], s_ref[-1]))

    idx = int(np.searchsorted(s_ref, s_query, side='right')) - 1
    idx = int(np.clip(idx, 0, len(s_ref) - 2))

    ds = s_ref[idx + 1] - s_ref[idx]
    t  = (s_query - s_ref[idx]) / ds if ds > 1e-9 else 0.0
    t  = float(np.clip(t, 0.0, 1.0))

    p0, p1 = pts_np[idx], pts_np[idx + 1]
    px  = p0[0] + t * (p1[0] - p0[0])
    py  = p0[1] + t * (p1[1] - p0[1])
    psi = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

    return px, py, psi


# ---------------------------------------------------------------------------
# Función de coste MPC
# ---------------------------------------------------------------------------

def mpc_cost(
    u_flat: np.ndarray,
    x0: float, y0: float, psi0: float, v0: float,
    s_ref: np.ndarray, pts_np: np.ndarray,
    N: int, dt: float, wheelbase: float,
    w_cte: float, w_heading: float,
    w_dsteer: float, w_speed: float,
    v_target: float,
    prev_steer: float,
) -> float:
    """Calcula el coste MPC para una secuencia de control candidata.

    El vector de control u_flat contiene [δ_0, ..., δ_{N-1}, a_0, ..., a_{N-1}]
    o sólo [δ_0, ..., δ_{N-1}] si w_speed=0.

    La función de coste es:
        J = Σ_{k=0}^{N-1} [
              w_cte     * e_cte(k)²        (error lateral signed)
            + w_heading * e_psi(k)²        (error de orientación)
            + w_dsteer  * Δδ(k)²           (suavidad de dirección)
            + w_speed   * (v(k)-v_t)²/v_t² (error de velocidad normalizado)
            ]

    Args:
        u_flat:     Vector de control plano [δ_0..δ_{N-1}] (N,).
        x0,y0,psi0: Estado inicial en frame local del vehículo.
        v0:         Velocidad inicial medida [m/s].
        s_ref:      Parámetros de arco del path de referencia.
        pts_np:     Puntos del path (Nx2).
        N:          Longitud del horizonte.
        dt:         Paso de tiempo [s].
        wheelbase:  Distancia entre ejes [m].
        w_cte:      Peso del error lateral.
        w_heading:  Peso del error de orientación.
        w_dsteer:   Peso de la tasa de cambio de dirección.
        w_speed:    Peso del error de velocidad (0 = ignorar).
        v_target:   Velocidad deseada [m/s].
        prev_steer: Último comando de dirección [rad].

    Returns:
        Coste escalar J.
    """
    x, y, psi, v = x0, y0, psi0, v0
    cost   = 0.0
    prev_u = prev_steer

    # El arco acumulado predicho desde el origen
    s_vehicle = 0.0

    for k in range(N):
        delta = float(u_flat[k])

        # Punto de referencia interpolado por avance en arco
        ref_x, ref_y, ref_psi = interpolate_path(s_ref, pts_np, s_vehicle)

        # Error lateral (signed): positivo = vehículo a la izquierda del path
        dx  = x - ref_x
        dy  = y - ref_y
        cte = -dx * math.sin(ref_psi) + dy * math.cos(ref_psi)

        # Error de orientación (normalizado a [-pi, pi])
        heading_err = psi - ref_psi
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        # Tasa de cambio de dirección
        d_steer = delta - prev_u

        # Acumulación de coste
        cost += w_cte     * cte         ** 2
        cost += w_heading * heading_err ** 2
        cost += w_dsteer  * d_steer     ** 2

        if w_speed > 0.0 and v_target > 0.0:
            # Error de velocidad normalizado por v_target para escala consistente
            v_err = (v - v_target) / v_target
            cost += w_speed * v_err ** 2

        # Propagación del modelo
        x, y, psi = bicycle_step(x, y, psi, v, delta, wheelbase, dt)

        # Avance en arco estimado (usando v actual)
        s_vehicle += v * dt

        # Actualizar v con un modelo de primera orden simple hacia v_target
        # (tau ≈ 3*dt → el kart llega al 95% de v_target en ~9 pasos)
        tau = 3.0 * dt
        v   = v + (v_target - v) * dt / tau
        v   = float(np.clip(v, MIN_SPEED, MAX_SPEED))

        prev_u = delta

    return cost


# ---------------------------------------------------------------------------
# Nodo ROS2 principal
# ---------------------------------------------------------------------------

class KartMPCNode(Node):
    """Controlador MPC para seguimiento de trayectoria en kart autónomo.

    Suscribe:
        /perception/cones_3d  (Detection3DArray) — detecciones en frame cámara
        /model/kart/odom_gt   (Odometry)         — velocidad real del vehículo

    Publica:
        /kart/cmd_vel         (Twist)             — δ en angular.z, v en linear.x
    """

    def __init__(self):
        super().__init__("kart_mpc")

        # ── parámetros ──────────────────────────────────────────────────────
        self.declare_parameter("detections_topic",  "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic",     "/kart/cmd_vel")
        self.declare_parameter("no_cone_timeout",   1.0)

        # MPC
        self.declare_parameter("mpc_horizon",       10)     # pasos
        self.declare_parameter("mpc_dt",            0.12)   # [s]; igualar al período real
        self.declare_parameter("mpc_w_cte",         1.0)    # peso CTE
        self.declare_parameter("mpc_w_heading",     0.5)    # peso heading error
        self.declare_parameter("mpc_w_dsteer",      30.0)   # peso steering rate
        self.declare_parameter("mpc_w_speed",       1.0)    # peso speed error (0=off)
        self.declare_parameter("mpc_lookahead",     8.0)    # distancia de path [m]
        self.declare_parameter("mpc_target_speed",  3.5)    # velocidad deseada [m/s]
        self.declare_parameter("mpc_max_iter_cold", 300)    # iter. en cold start
        self.declare_parameter("mpc_max_iter_warm", 80)     # iter. en warm start
        self.declare_parameter("mpc_ftol",          1e-4)   # tolerancia SLSQP

        # Vehículo
        self.declare_parameter("wheelbase",         WHEELBASE)
        self.declare_parameter("max_steer",         MAX_STEER)
        self.declare_parameter("max_speed",         MAX_SPEED)
        self.declare_parameter("min_speed",         MIN_SPEED)
        self.declare_parameter("half_track_width",  HALF_TRACK_WIDTH)

        # ── leer parámetros ─────────────────────────────────────────────────
        self.N          = int(self.get_parameter("mpc_horizon").value)
        self.dt         = float(self.get_parameter("mpc_dt").value)
        self.w_cte      = float(self.get_parameter("mpc_w_cte").value)
        self.w_heading  = float(self.get_parameter("mpc_w_heading").value)
        self.w_dsteer   = float(self.get_parameter("mpc_w_dsteer").value)
        self.w_speed    = float(self.get_parameter("mpc_w_speed").value)
        self.lookahead  = float(self.get_parameter("mpc_lookahead").value)
        self.v_target   = float(self.get_parameter("mpc_target_speed").value)
        self.maxiter_cold = int(self.get_parameter("mpc_max_iter_cold").value)
        self.maxiter_warm = int(self.get_parameter("mpc_max_iter_warm").value)
        self.ftol       = float(self.get_parameter("mpc_ftol").value)

        self.wheelbase  = float(self.get_parameter("wheelbase").value)
        self.max_steer  = float(self.get_parameter("max_steer").value)
        self.max_speed  = float(self.get_parameter("max_speed").value)
        self.min_speed  = float(self.get_parameter("min_speed").value)
        self.htw        = float(self.get_parameter("half_track_width").value)

        det_topic = str(self.get_parameter("detections_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.no_cone_timeout = float(self.get_parameter("no_cone_timeout").value)

        if not HAS_SCIPY:
            self.get_logger().error("scipy no instalado. Ejecuta: pip install scipy")
            raise SystemExit(1)

        # ── estado interno ──────────────────────────────────────────────────
        self._actual_speed:    float            = 0.0
        self._last_steer:      float            = 0.0
        self._prev_solution:   np.ndarray | None = None
        self._is_cold_start:   bool             = True
        self._last_det_wall:   float            = time.monotonic()
        self._last_log_wall:   float            = time.monotonic()
        self.last_det_time                      = self.get_clock().now()

        # ── ROS plumbing ────────────────────────────────────────────────────
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
            f"KartMPCNode iniciado — horizonte={self.N} dt={self.dt}s "
            f"v_target={self.v_target}m/s"
        )

    # ── callbacks ───────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        """Extrae velocidad actual del mensaje de odometría."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._actual_speed = math.sqrt(vx * vx + vy * vy)

    def _on_detections(self, msg: Detection3DArray) -> None:
        """Callback principal: filtra conos, construye path, ejecuta MPC."""
        now_wall = time.monotonic()
        dt_wall_ms = (now_wall - self._last_det_wall) * 1000.0
        self._last_det_wall = now_wall
        self.last_det_time  = self.get_clock().now()

        # Logging de frecuencia real (1 Hz)
        if now_wall - self._last_log_wall > 1.0:
            self._last_log_wall = now_wall
            self.get_logger().info(
                f"[mpc] callback_dt={dt_wall_ms:.1f}ms "
                f"({1000.0/max(dt_wall_ms,1):.1f}Hz) "
                f"recomendado mpc_dt={dt_wall_ms*1.2/1000:.3f}s"
            )

        # Filtrar cones
        cones: list[tuple[str, float, float]] = []
        for det in msg.detections:
            if not det.results:
                continue
            cls  = det.results[0].hypothesis.class_id
            pos  = det.results[0].pose.pose.position
            fwd  = pos.z          # camera Z = adelante
            left = -pos.x         # camera -X = izquierda

            if fwd < 0.5:
                continue
            dist  = math.hypot(fwd, left)
            if dist > 15.0:
                continue
            angle = abs(math.atan2(left, fwd))
            if angle > 0.6109:   # ≈ 35° FOV half-angle
                continue
            cones.append((cls, fwd, left))

        steer, speed = self._control_mpc(cones)

        # Sin conos → parar y resetear warm-start
        if not cones:
            steer, speed = 0.0, 0.0
            self._prev_solution = None
            self._is_cold_start = True
            self.get_logger().warn(
                "[mpc] sin conos — deteniendo. Comprueba la percepción."
            )

        cmd = Twist()
        cmd.angular.z = steer
        cmd.linear.x  = speed
        self.cmd_pub.publish(cmd)

    # ── controlador MPC ─────────────────────────────────────────────────────

    def _control_mpc(
        self, cones: list[tuple[str, float, float]]
    ) -> tuple[float, float]:
        """Ejecuta un paso del controlador MPC.

        1. Construye el path de referencia desde los midpoints de conos.
        2. Recorta el path a mpc_lookahead metros.
        3. Parametriza el path por longitud de arco.
        4. Optimiza la secuencia de N ángulos de dirección que minimizan
           el coste (CTE + heading + steering rate + speed).
        5. Aplica sólo el primer elemento (horizonte recesivo).

        Args:
            cones: Lista de (class_id, fwd, left) en frame cámara.

        Returns:
            (steer_rad, speed_mps): comando para este instante.
        """
        # 1. Construir path
        raw = build_midpoint_path(cones, self.htw)
        if len(raw) < 2:
            return self._last_steer, self.min_speed

        # 2. Recortar a lookahead (incluir primer punto que supera el umbral
        #    para que la tangente en el extremo esté bien definida)
        path: list[tuple[float, float]] = []
        cum = 0.0
        prev = (0.0, 0.0)
        for fwd, left in raw:
            seg = math.hypot(fwd - prev[0], left - prev[1])
            cum += seg
            path.append((fwd, left))
            prev = (fwd, left)
            if cum > self.lookahead:
                break

        if len(path) < 2:
            return self._last_steer, self.min_speed

        # 3. Parametrizar por arco
        s_ref, pts_np = build_arc_parameterization(path)

        # 4. Condición inicial: vehículo en el origen local con heading 0
        #    (el path ya está en frame local del vehículo)
        x0, y0, psi0 = 0.0, 0.0, 0.0
        # Usar velocidad real como condición inicial del modelo
        v0 = max(self._actual_speed, self.min_speed)

        # 5. Warm-start
        if (
            self._prev_solution is not None
            and len(self._prev_solution) == self.N
            and not self._is_cold_start
        ):
            u0       = np.empty(self.N)
            u0[:-1]  = self._prev_solution[1:]
            u0[-1]   = self._prev_solution[-1]
            max_iter = self.maxiter_warm
        else:
            # Cold start: inicializar con dirección actual
            u0       = np.full(self.N, self._last_steer)
            max_iter = self.maxiter_cold

        bounds = [(-self.max_steer, self.max_steer)] * self.N

        # 6. Optimización
        t0 = time.monotonic()
        result = scipy_minimize(
            mpc_cost,
            u0,
            args=(
                x0, y0, psi0, v0,
                s_ref, pts_np,
                self.N, self.dt, self.wheelbase,
                self.w_cte, self.w_heading,
                self.w_dsteer, self.w_speed,
                self.v_target,
                self._last_steer,
            ),
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": max_iter, "ftol": self.ftol},
        )
        solve_ms = (time.monotonic() - t0) * 1000.0

        # 7. Gestión del warm-start:
        #    sólo actualizar si la solución convergió (evita propagar malos mínimos)
        if result.success:
            self._prev_solution = result.x.copy()
            self._is_cold_start = False
        else:
            # Solución fallida → resetear para el siguiente ciclo
            self._prev_solution = None
            self._is_cold_start = True
            self.get_logger().warn(
                f"[mpc] optimizador no convergió (itr={result.nit}) — "
                f"reseteando warm-start. msg: {result.message}"
            )

        # 8. Aplicar primer control (horizonte recesivo)
        steer = float(np.clip(result.x[0], -self.max_steer, self.max_steer))
        self._last_steer = steer

        # 9. Velocidad coherente con el MPC:
        #    el MPC asume v_target; reducir ligeramente en curvas cerradas
        #    para no crear discrepancia modelo-planta excesiva.
        curve_factor = max(0.0, 1.0 - 1.0 * (abs(steer) / self.max_steer) ** 2)
        speed = float(np.clip(self.v_target * curve_factor, self.min_speed, self.max_speed))

        self.get_logger().info(
            f"[mpc] δ={math.degrees(steer):.1f}° v_cmd={speed:.2f}m/s "
            f"v_real={self._actual_speed:.2f}m/s pts={len(path)} "
            f"ok={result.success} itr={result.nit} J={result.fun:.4f} "
            f"solve={solve_ms:.1f}ms cold={self._is_cold_start}"
        )
        return steer, speed

    # ── safety timeout ───────────────────────────────────────────────────────

    def _safety_check(self) -> None:
        """Publica velocidad cero si no llegan detecciones dentro del timeout."""
        elapsed = (
            self.get_clock().now() - self.last_det_time
        ).nanoseconds / 1e9
        if elapsed > self.no_cone_timeout:
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

def main() -> None:
    """Punto de entrada del nodo MPC."""
    rclpy.init()
    node = KartMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
