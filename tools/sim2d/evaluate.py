"""Fixed evaluation script for autoresearch. DO NOT MODIFY.

Trains a controller using the strategy defined in strategy.py,
then evaluates the best result on the autocross track.
Outputs a single metric: lap_time (lower is better).
"""

import json
import sys
import time
import multiprocessing

from sim import run_episode, set_track
from track import get_track

# Fixed evaluation parameters
TRACK = "autocross"
EVAL_MAX_STEPS = 5000  # enough for ~2 laps at moderate speed
TRAIN_TIME_BUDGET = 60  # seconds — fixed training budget


def evaluate():
    set_track(TRACK, max_steps=EVAL_MAX_STEPS)

    # Import the strategy (the file the agent edits)
    from strategy import train_controller

    t0 = time.time()
    controller, train_info = train_controller(time_budget=TRAIN_TIME_BUDGET)
    train_time = time.time() - t0

    # Run 3 evaluation episodes and take the best
    best_result = None
    for _ in range(3):
        result = run_episode(controller, max_steps=EVAL_MAX_STEPS, fitness_mode="v2")
        if best_result is None or result["fitness"] > best_result["fitness"]:
            best_result = result

    laps = best_result["laps"]
    ep_time = best_result["time"]
    distance = best_result["distance"]
    avg_cte = best_result["avg_cte"]
    max_cte = best_result["max_cte"]
    avg_speed = best_result["avg_speed"]
    min_bd = best_result["min_boundary_dist"]
    steps = best_result["steps"]

    # Lap time: only meaningful if at least 1 lap completed
    if laps >= 1:
        lap_time = ep_time / laps
    else:
        lap_time = 999.0

    # Did it stay on track the whole time?
    on_track = min_bd >= 0

    # Combined loss (lower is better):
    # Primary: lap time (only if completing laps)
    # Penalties for not completing, going off track, poor centering
    if laps == 0:
        # Didn't complete a lap — use distance as proxy (higher distance = lower loss)
        track = get_track(TRACK)
        progress_frac = distance / track.track_length
        loss = 100.0 - 50.0 * progress_frac  # 100 if no progress, 50 if almost 1 lap
    else:
        loss = lap_time  # just the lap time

    if not on_track:
        loss += 50.0  # harsh penalty for going off track

    loss += 0.5 * avg_cte  # small centering bonus

    print("---")
    print(f"laps:             {laps}")
    print(f"lap_time:         {lap_time:.6f}")
    print(f"distance:         {distance:.2f}")
    print(f"avg_speed:        {avg_speed:.3f}")
    print(f"avg_cte:          {avg_cte:.4f}")
    print(f"max_cte:          {max_cte:.4f}")
    print(f"min_boundary:     {min_bd:.4f}")
    print(f"on_track:         {on_track}")
    print(f"train_time:       {train_time:.1f}")
    print(f"loss:             {loss:.6f}")


if __name__ == "__main__":
    multiprocessing.set_start_method("fork")
    evaluate()
