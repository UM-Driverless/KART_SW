#!/usr/bin/env python3
"""Replay the best run with a matplotlib plot (or animation)."""

import argparse
import json
import sys

import numpy as np
import matplotlib.pyplot as plt

from track import get_track
from kart_model import KartState, step as kart_step
from perception import perceive
from controllers import GeometricController, NeuralNetController, NeuralNetV2Controller

CONTROLLER_CLASSES = {
    "geometric": GeometricController,
    "neural": NeuralNetController,
    "neural_v2": NeuralNetV2Controller,
}


def run_and_record(controller, track, max_steps=5000):
    """Run one episode, returning an Nx5 array (x, y, yaw, speed, steer)."""
    state = KartState(track.spawn_x, track.spawn_y, track.spawn_yaw, speed=0.0)
    controller.reset()
    wants_speed = hasattr(controller, '_last_speed')
    rows = [(state.x, state.y, state.yaw, state.speed, 0.0)]

    for _ in range(max_steps):
        visible = perceive(state, track.blue_cones, track.yellow_cones,
                           track.orange_cones)
        if wants_speed:
            steer, speed_cmd = controller.control(visible,
                                                  current_speed=state.speed)
        else:
            steer, speed_cmd = controller.control(visible)
        state = kart_step(state, steer, speed_cmd)
        rows.append((state.x, state.y, state.yaw, state.speed, steer))

        _, cte = track.project_to_centerline(state.x, state.y)
        if cte > 5.0:
            break

    return np.array(rows)


def plot_trajectory(traj, track, title="2D Sim — Kart Trajectory",
                    save_path=None):
    fig, ax = plt.subplots(figsize=(12, 10))

    # Centerline
    ax.plot(track.centerline_xy[:, 0], track.centerline_xy[:, 1],
            "k--", alpha=0.3, lw=1, label="Centerline")

    # Cones
    ax.scatter(*track.blue_cones.T, c="blue", s=30, marker="^", label="Blue")
    ax.scatter(*track.yellow_cones.T, c="gold", s=30, marker="^",
               label="Yellow")
    ax.scatter(*track.orange_cones.T, c="orange", s=60, marker="^",
               label="Orange")

    # Trajectory coloured by speed
    sc = ax.scatter(traj[:, 0], traj[:, 1], c=traj[:, 3],
                    cmap="RdYlGn", s=2, vmin=0, vmax=5)
    plt.colorbar(sc, ax=ax, label="Speed (m/s)")

    ax.plot(traj[0, 0], traj[0, 1], "go", ms=10, label="Start")
    ax.plot(traj[-1, 0], traj[-1, 1], "ro", ms=10, label="End")

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved → {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualise best controller")
    parser.add_argument("json_file", nargs="?", default=None,
                        help="Path to best_*.json")
    parser.add_argument("--default", action="store_true",
                        help="Run default geometric parameters")
    parser.add_argument("--track", type=str, default=None,
                        help="Track: built-in name (oval, hairpin, autocross), "
                             "JSON path, or random spec (random:seed=42). "
                             "Default: from JSON metadata or oval.")
    parser.add_argument("--save", type=str, default=None,
                        help="Save plot to file instead of showing")
    args = parser.parse_args()

    if args.default:
        ctrl = GeometricController(GeometricController.DEFAULTS)
        label = "Default geometric"
        track_name = args.track or "oval"
    elif args.json_file:
        with open(args.json_file) as f:
            data = json.load(f)
        cls = CONTROLLER_CLASSES[data["controller_type"]]
        ctrl = cls(np.array(data["genes"]))
        label = f"Best {data['controller_type']} (fitness {data['fitness']:.1f})"
        track_name = args.track or data.get("track", "oval")
    else:
        print("Usage: visualize.py <best.json>  or  visualize.py --default")
        sys.exit(1)

    track = get_track(track_name)
    print(f"Running episode: {label} on '{track_name}' ...")
    traj = run_and_record(ctrl, track)
    print(f"  {len(traj)} steps recorded")
    plot_trajectory(traj, track, title=f"{label} — {track_name}",
                    save_path=args.save)


if __name__ == "__main__":
    main()
