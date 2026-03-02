# Scratchpad — Simulation & Track Work (2026-03-01)

## Context: Running Claude Code on the UTM VM

### The problem
Claude Code runs on the Mac. The Gazebo simulator runs on the UTM VM (Ubuntu 22.04, GNOME on Wayland). Taking screenshots and launching Gazebo over SSH is painful:
- GNOME runs **Wayland**, so X11 tools (`scrot`, `xdotool`) can't see windows
- `gnome-screenshot` works but requires `DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus`
- If GNOME Activities overlay gets stuck, there's no reliable way to dismiss it over SSH
- Launching Gazebo via `nohup ... &` over SSH causes Ignition Transport disconnection (bridge sees `blues=0 yellows=0`). Must use `gnome-terminal --` to launch within the GNOME session.
- Camera sensor images don't publish without a display (headless EGL fails on llvmpipe)

### The plan
**Run Claude Code directly inside the UTM VM.** This eliminates all SSH/Wayland/DBUS issues:
- Same GNOME session → `gnome-screenshot` works natively
- Same terminal → Gazebo launches properly with full Ignition Transport connectivity
- Direct filesystem access → no SCP needed
- Can commit and push from the VM (git is already set up there)

### VM specs (should be enough)
- 8 GB RAM, 4 CPU cores
- ~17 GB free disk
- Node.js needed for Claude Code (install via `nvm` or `apt`)
- The workspace `~/kart_brain` is already there and built

### What to try
1. Install Claude Code on the VM
2. Open a terminal in UTM, run `claude`
3. Test: launch Gazebo, take screenshot, validate rendering
4. Edit code, commit, push — all from the VM

## Current state: Autocross track

### What's done
- **New autocross track created** — ~80x60m field, mix of left/right turns, S-chicane, long straights
- `scripts/sim2d/track.py` — `AUTOCROSS_TRACK` with 70 blue + 70 yellow + 4 orange cones
- `scripts/sim2d/generate_sdf.py` — generates SDF from track.py (keeps 2D sim and Gazebo in sync)
- `src/kart_sim/worlds/autocross_track.sdf` — generated, deployed to VM
- `src/kart_sim/launch/simulation.launch.py` — `autocross` entry added to `_TRACKS`
- **2D sim validated** — track plot viewed, smooth curves, no self-intersections
- **Gazebo validated by user** — user opened Gazebo from UTM GUI, shared screenshot showing cones rendered correctly
- **Controller runs** — logs showed `neural_v2` steering with `blues=5-7 yellows=4-6`, kart navigating the track

### What's done (2026-03-01, in-VM session)
- **Claude Code running on VM** — confirmed working, no SSH/Wayland issues
- **Neural controller retrained on autocross** — 50 generations, seeded from oval weights
  - **6 laps, 1263m, avg_cte=0.24m, avg_speed=5.0m/s** on autocross
  - Also generalizes to hairpin: **8 laps, avg_cte=0.34m**
  - Weights deployed to `src/kart_sim/config/neural_v2_weights.json`, package rebuilt
- **`is_inside_track` bug fixed** — polygon self-intersection on complex tracks caused false "outside" detection at the seam (cone N-1 → cone 0). Replaced with cross-product approach.
- **`--track` arg added to `train.py`** — can now train on any track: `python3 train.py --track=autocross`
- **`--max-steps` arg added** — longer episodes for bigger tracks
- **Sigma restart in GA** — prevents premature convergence (restarts sigma after 30 stagnant gens)

### What's done (2026-03-02, training improvements)
- **CMA-ES optimizer added** — `--optimizer cma` flag in train.py. Uses `cma` Python package. Much more sample-efficient than basic GA.
- **NeuralNetV3Controller** — 2-hidden-layer net (19→24→12→2, 806 genes). Extra inputs: current steer + steer rate.
- **Perception noise** — `--noise` (std in metres) and `--dropout` (cone drop probability) flags.
- **Multi-track training** — `--track oval,hairpin,autocross` evaluates on all tracks, averages fitness.
- **5 experiments run on y540-ubuntu-local** (12 cores, 15GB RAM, venv at `~/cma_venv`):

| Experiment | Fitness | Laps | avg_cte | avg_speed | Notes |
|---|---|---|---|---|---|
| CMA-V2 autocross (seeded) | **4934.7** | 12 | **0.07m** | 9.9 m/s | Best centering |
| CMA-V2 noisy (noise=0.15, dropout=0.05) | 4917.2 | 12 | 0.10m | 9.9 m/s | **Best for real-world** |
| CMA-V3 from scratch | 4702.1 | 12 | 0.27m | 9.8 m/s | From scratch, 100 gens |
| CMA-V3 fine-tuned | **4905.8** | 12 | 0.14m | 9.9 m/s | Approaching V2 |
| CMA-V2 multitrack-avg | 4894.1 | 18* | 0.13m | 9.9 m/s | *18 laps on oval, overfit |

- **Cross-track generalization**: Single-track models don't generalize well between tracks (different geometry/spawn). This is expected — tracks are too different.
- **Deployed weights**: noise-robust CMA-V2 deployed to `neural_v2_weights.json`

### What's NOT done
- **Gazebo visual validation** — need to launch Gazebo with `track:=autocross` and confirm kart completes laps visually
- **`visualize.py` bug** — `GeometricController.control()` missing `current_speed` kwarg. Pre-existing, not blocking.
- **Recurrent controller (LSTM/GRU)** — could improve temporal decision-making

## What we want to do with the simulator

### Immediate goals
1. **Validate in Gazebo** — launch `ros2 launch kart_sim simulation.launch.py track:=autocross`, visually confirm the kart completes laps

### Longer term
- Test on real hardware (Orin) once controller is validated in simulation
- Try the YOLO perception pipeline (`use_yolo:=true`) instead of perfect perception
- Explore recurrent networks (LSTM-like) for temporal context

---

## TODO
- [x] ~~Install Claude Code on VM~~ (confirmed working 2026-03-01)
- [x] ~~Retrain neural_v2 on autocross track~~ (6 laps, fitness=2457)
- [x] ~~Fix `is_inside_track` bug~~ (cross-product approach)
- [x] ~~Add CMA-ES optimizer~~ (2026-03-02)
- [x] ~~Add NeuralNetV3 controller~~ (2-hidden-layer, 806 genes)
- [x] ~~Add perception noise/dropout~~ (for sim-to-real robustness)
- [x] ~~Multi-track training support~~ (avg fitness across tracks)
- [x] ~~Deploy noise-robust weights~~ (CMA-V2 noisy → neural_v2_weights.json)
- [ ] **Validate in Gazebo** — launch with `track:=autocross` and confirm visually
- [ ] **Document Claude Code on VM setup** (add to `vm_environment.md`)
- [ ] Fix `visualize.py` GeometricController bug (low priority)
- [ ] Explore recurrent (LSTM-like) controller for temporal context

---

## Previous: YOLOv11 Migration (2026-02-25)

### Status
- YOLO + CUDA: **WORKING** (cuBLAS fix confirmed)
- Camera (`/dev/video0`): Needs USB reset or reconnection
- Full pipeline test with real camera: **NOT YET DONE**

### What's left
1. Test with real camera — USB reset the ZED, run pipeline, verify cone detection end-to-end
2. Verify colored labels — confirm per-class colors render correctly with YOLOv11 class names
3. Measure GPU FPS — benchmark with real frames
4. Check class name mapping — YOLOv11 model has class `unknown_cone` (id 3) that YOLOv5 didn't have
