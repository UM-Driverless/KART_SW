"""Simulation loop: perceive → control → step physics → check termination."""

import numpy as np

from track import OVAL_TRACK, get_track
from kart_model import KartState, step as kart_step, DT, MAX_SPEED
from perception import perceive

DEFAULT_MAX_STEPS = 2000
BOUNDARY_DANGER = 0.5  # m — penalize when kart is within this distance of boundary

# Active track — set via set_track() before training, defaults to oval
_active_track = OVAL_TRACK
_active_tracks = None  # list of tracks for multi-track evaluation
_max_steps = DEFAULT_MAX_STEPS
_noise_std = 0.0       # perception noise (metres)
_dropout = 0.0         # cone dropout probability


def set_track(name_or_track, max_steps=None, noise_std=0.0, dropout=0.0):
    """Set the active track for all subsequent run_episode() calls.

    Must be called before forking worker processes.
    """
    global _active_track, _active_tracks, _max_steps, _noise_std, _dropout
    if isinstance(name_or_track, str):
        if "," in name_or_track:
            names = [n.strip() for n in name_or_track.split(",")]
            _active_tracks = [get_track(n) for n in names]
            _active_track = _active_tracks[0]
        else:
            _active_track = get_track(name_or_track)
            _active_tracks = None
    else:
        _active_track = name_or_track
        _active_tracks = None
    if max_steps is not None:
        _max_steps = max_steps
    _noise_std = noise_std
    _dropout = dropout


def run_episode(controller, max_steps=None, fitness_mode="v1"):
    """Run one full episode.

    Parameters
    ----------
    fitness_mode : "v1" | "v2" | "v3" | "v4" | "v5" | "v6"
        v1: distance + lap bonus - CTE + speed (original)
        v2: lap-time based — completing laps fast is king
        v3: track-keeping first — nonlinear CTE⁴ penalty, reward progress,
            speed only matters through completing laps
        v4: boundary-aware — terminate if outside track, penalize near boundary
        v5: centered progress — each meter of progress scaled by centering
        v6: centered fast progress — v5 × (1 + speed/max_speed)

    Returns
    -------
    dict with keys: fitness, distance, laps, avg_cte, avg_speed, steps, time,
                    max_cte, cte_penalty, min_boundary_dist, boundary_penalty,
                    weighted_progress
    """
    if max_steps is None:
        max_steps = _max_steps
    track = _active_track
    state = KartState(track.spawn_x, track.spawn_y, track.spawn_yaw, speed=0.0)
    controller.reset()

    # Check if controller accepts current_speed (v2)
    wants_speed = hasattr(controller, '_last_speed')

    s_prev, _ = track.project_to_centerline(state.x, state.y)
    total_distance = 0.0
    total_cte = 0.0
    total_cte4 = 0.0   # sum of (cte / half_track)^4 per step
    max_cte = 0.0
    total_speed = 0.0
    boundary_penalty = 0.0
    min_boundary_dist = float('inf')
    weighted_progress = 0.0
    speed_weighted_progress = 0.0
    steps = 0

    for _ in range(max_steps):
        visible = perceive(state, track.blue_cones, track.yellow_cones,
                           track.orange_cones, noise_std=_noise_std,
                           dropout=_dropout)

        if wants_speed:
            steer, speed_cmd = controller.control(visible,
                                                  current_speed=state.speed)
        else:
            steer, speed_cmd = controller.control(visible)

        state = kart_step(state, steer, speed_cmd)
        steps += 1

        s_cur, cte = track.project_to_centerline(state.x, state.y)

        # Incremental distance (handles wrap-around)
        ds = s_cur - s_prev
        if ds > track.track_length / 2:
            ds -= track.track_length
        elif ds < -track.track_length / 2:
            ds += track.track_length
        total_distance += ds
        s_prev = s_cur

        total_cte += cte
        total_speed += state.speed
        if cte > max_cte:
            max_cte = cte

        # Nonlinear: (cte / half_track)^4 — explodes when approaching edges
        cte_norm = cte / track.half_track_width
        total_cte4 += cte_norm ** 4

        # Weighted progress: scale each step by centering quality
        centering = max(0.0, 1.0 - cte_norm ** 2)
        weighted_progress += ds * centering
        # Speed-weighted progress: reward going faster while centered
        speed_weighted_progress += ds * centering * (1.0 + state.speed / MAX_SPEED)

        # Boundary check — distance to nearest boundary line segment
        bd = track.dist_to_boundary(state.x, state.y)
        if bd < min_boundary_dist:
            min_boundary_dist = bd

        # Outside track → terminate
        if bd < 0:
            break

        # Close to boundary → accumulate penalty
        if bd < BOUNDARY_DANGER:
            boundary_penalty += (1.0 - bd / BOUNDARY_DANGER) ** 2

    avg_cte = total_cte / steps if steps else 0.0
    avg_speed = total_speed / steps if steps else 0.0
    avg_cte4 = total_cte4 / steps if steps else 0.0
    laps = int(total_distance / track.track_length) if total_distance > 0 else 0
    time = steps * DT

    if fitness_mode == "v6":
        # Centered fast progress: centering × speed, no separate lap bonus
        fitness = speed_weighted_progress
    elif fitness_mode == "v5":
        # Centered progress: each meter scaled by how centered the kart is
        fitness = weighted_progress + 200.0 * laps
    elif fitness_mode == "v4":
        # Boundary-aware fitness:
        #   CTE^4 keeps kart centered, boundary_penalty keeps it off the edges
        cte_penalty = 1.5 * total_cte4
        fitness = total_distance + 200.0 * laps - cte_penalty - 50.0 * boundary_penalty
    elif fitness_mode == "v3":
        cte_penalty = 0.5 * total_cte4
        if max_cte > track.half_track_width:
            cte_penalty += 200.0 * ((max_cte / track.half_track_width) ** 2)
        fitness = total_distance + 200.0 * laps - cte_penalty
    elif fitness_mode == "v2":
        if laps >= 1:
            fitness = 500.0 * laps - time - 5.0 * avg_cte
        else:
            fitness = total_distance - 2.0 * avg_cte
    else:
        fitness = (total_distance
                   + 100.0 * laps
                   - 2.0 * avg_cte
                   + 0.5 * avg_speed)

    return {
        "fitness": fitness,
        "distance": total_distance,
        "laps": laps,
        "avg_cte": avg_cte,
        "avg_speed": avg_speed,
        "steps": steps,
        "time": time,
        "max_cte": max_cte,
        "cte_penalty": cte_penalty if fitness_mode in ("v3", "v4") else 0.0,
        "weighted_progress": weighted_progress,
        "speed_weighted_progress": speed_weighted_progress,
        "min_boundary_dist": min_boundary_dist,
        "boundary_penalty": boundary_penalty,
    }


def run_episode_multitrack(controller, fitness_mode="v1"):
    """Evaluate on all active tracks; return min fitness (worst track).

    Falls back to single-track if _active_tracks is not set.
    """
    global _active_track
    tracks = _active_tracks or [_active_track]
    results = []
    saved = _active_track
    for track in tracks:
        _active_track = track
        controller.reset()
        r = run_episode(controller, fitness_mode=fitness_mode)
        results.append(r)
    _active_track = saved

    # Average fitness across tracks — balanced pressure on all tracks
    avg_fitness = sum(r["fitness"] for r in results) / len(results)
    # Use the result from the worst track for other metrics
    worst_idx = min(range(len(results)), key=lambda i: results[i]["fitness"])
    result = dict(results[worst_idx])
    result["fitness"] = avg_fitness
    result["per_track"] = [{"track_length": t.track_length,
                            "fitness": r["fitness"],
                            "laps": r["laps"],
                            "avg_cte": r["avg_cte"]}
                           for t, r in zip(tracks, results)]
    return result
