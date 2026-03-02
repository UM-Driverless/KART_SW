"""Simulated perception: FOV + range filter, output in kart frame."""

import numpy as np
from kart_model import KartState

CAMERA_OFFSET = 0.55          # m forward of chassis center
FOV_HALF = np.radians(35)     # ±35° = 70° total (ZED 2i @ VGA)
RANGE_MIN = 0.5               # m
RANGE_MAX = 15.0              # m


def perceive(state: KartState, blue_cones, yellow_cones, orange_cones,
             noise_std=0.0, dropout=0.0):
    """Return cones visible from the kart's camera.

    Each entry is ``(class_id, x, y, z)`` in the kart frame
    (X = forward, Y = left, Z = up).

    Parameters
    ----------
    noise_std : float
        Gaussian noise added to cone positions (metres).  0 = perfect.
    dropout : float
        Probability of dropping each visible cone (0–1).  0 = no drops.
    """
    cos_yaw = np.cos(state.yaw)
    sin_yaw = np.sin(state.yaw)

    cam_x = state.x + CAMERA_OFFSET * cos_yaw
    cam_y = state.y + CAMERA_OFFSET * sin_yaw

    visible = []

    for class_id, cones in (("blue_cone", blue_cones),
                             ("yellow_cone", yellow_cones),
                             ("orange_cone", orange_cones)):
        if len(cones) == 0:
            continue
        # Vectorised relative position
        dx = cones[:, 0] - cam_x
        dy = cones[:, 1] - cam_y

        # Kart frame: fwd / left
        fwd = dx * cos_yaw + dy * sin_yaw
        left = -dx * sin_yaw + dy * cos_yaw

        dist = np.sqrt(fwd * fwd + left * left)
        angle = np.arctan2(left, fwd)

        mask = (dist >= RANGE_MIN) & (dist <= RANGE_MAX) & (np.abs(angle) <= FOV_HALF)
        for i in np.where(mask)[0]:
            if dropout > 0 and np.random.random() < dropout:
                continue
            fx = float(fwd[i])
            fy = float(left[i])
            if noise_std > 0:
                fx += np.random.randn() * noise_std
                fy += np.random.randn() * noise_std
            visible.append((class_id, fx, fy, 0.0))

    return visible
