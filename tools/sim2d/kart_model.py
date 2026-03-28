"""Bicycle-model (Ackermann) kinematics for the kart."""

import numpy as np

WHEELBASE = 1.05   # m
MAX_STEER = 1.047  # rad (~60°, matches real kart)
MAX_SPEED = 10.0   # m/s (reasonable cap for real kart)
MAX_ACCEL = 2.0    # m/s²
MAX_DECEL = 3.0    # m/s²
TIRE_MU = 1.2      # tire friction coefficient
GRAVITY = 9.81     # m/s²
MAX_LAT_ACCEL = TIRE_MU * GRAVITY  # ~11.8 m/s² — cornering grip limit
DT = 0.05          # s  (20 Hz, matching real controller rate)


class KartState:
    """Minimal mutable state vector [x, y, yaw, speed]."""
    __slots__ = ("x", "y", "yaw", "speed")

    def __init__(self, x: float, y: float, yaw: float, speed: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed


def step(state: KartState, steer_cmd: float, speed_cmd: float,
         dt: float = DT) -> KartState:
    """Advance one timestep and return a *new* KartState."""
    steer = float(np.clip(steer_cmd, -MAX_STEER, MAX_STEER))

    # Cornering speed limit: v_max = sqrt(mu * g * R), R = wheelbase / tan(steer)
    target = float(np.clip(speed_cmd, 0.0, MAX_SPEED))
    if abs(steer) > 0.01:
        turn_radius = abs(WHEELBASE / np.tan(steer))
        max_cornering_speed = float(np.sqrt(MAX_LAT_ACCEL * turn_radius))
        target = min(target, max_cornering_speed)

    # Asymmetric acceleration limits (accel < decel, like real kart)
    dv_target = target - state.speed
    if dv_target > 0:
        max_dv = MAX_ACCEL * dt
    else:
        max_dv = MAX_DECEL * dt
    dv = float(np.clip(dv_target, -max_dv, max_dv))
    new_speed = float(np.clip(state.speed + dv, 0.0, MAX_SPEED))

    # Bicycle kinematics (use current speed for position update)
    dx = state.speed * np.cos(state.yaw) * dt
    dy = state.speed * np.sin(state.yaw) * dt
    dyaw = (state.speed * np.tan(steer) / WHEELBASE) * dt

    return KartState(
        x=state.x + dx,
        y=state.y + dy,
        yaw=state.yaw + dyaw,
        speed=new_speed,
    )
