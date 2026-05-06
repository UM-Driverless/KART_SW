#!/usr/bin/env python3
"""Monitor Gazebo odom and report lap times + speed."""
import math
import time
import subprocess
import sys

# Autocross start/finish line: spawn at (30, -7.5), heading +Y
# Detect lap crossing when kart passes through x≈30, y≈-7.5 heading north
START_X, START_Y = 30.0, -7.5
CROSSING_RADIUS = 3.0  # how close to start line to count

def main():
    proc = subprocess.Popen(
        ["ssh", "utm", "bash -ic 'source ~/kart-brain/install/setup.bash && ros2 topic echo /model/kart/odom_gt'"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
    )

    lap = 0
    lap_start = None
    last_near = False
    speeds = []

    x = y = vx = vy = 0.0
    parsing = {}

    for line in proc.stdout:
        line = line.strip()
        # Parse position
        if "x:" in line and "position" not in line and "orientation" not in line:
            parts = line.split(":")
            key = parts[0].strip()
            val = parts[1].strip()
            try:
                val = float(val)
            except ValueError:
                continue
            if key == "x":
                parsing["x"] = val
            elif key == "y":
                parsing["y"] = val
            elif key == "z" and "x" in parsing and "y" in parsing:
                x, y = parsing["x"], parsing["y"]
                parsing = {}

                # Speed from position changes (rough)
                dist = math.hypot(x - START_X, y - START_Y)
                near = dist < CROSSING_RADIUS

                if near and not last_near:
                    now = time.time()
                    if lap_start is not None:
                        lap_time = now - lap_start
                        avg_spd = sum(speeds) / len(speeds) if speeds else 0
                        lap += 1
                        print(f"Lap {lap}: {lap_time:.2f}s  (avg odom speed would need vel parsing)")
                        sys.stdout.flush()
                    lap_start = now
                    speeds = []

                last_near = near

if __name__ == "__main__":
    main()
