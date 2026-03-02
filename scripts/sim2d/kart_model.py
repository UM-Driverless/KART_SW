"""Bicycle-model (Ackermann) kinematics for the kart."""

import numpy as np

WHEELBASE = 1.05   # m
MAX_STEER = 0.5    # rad
MAX_SPEED = 10.0   # m/s
MAX_ACCEL = 2.0    # m/s²
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

    # Acceleration-limited speed update
    target = float(np.clip(speed_cmd, 0.0, MAX_SPEED))
    max_dv = MAX_ACCEL * dt
    dv = float(np.clip(target - state.speed, -max_dv, max_dv))
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
