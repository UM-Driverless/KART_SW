# TODO

## Kart Still Oversteers with YOLO Pipeline (Simulation)

**Status:** CameraInfo intrinsics fixed, midpoint-angle steering implemented, but kart still doesn't drive properly.

**What was fixed (2026-02-22):**

- `camera_info_fix_node.py` corrects Gazebo's wrong intrinsics (FX 277→381.5, CX 160→320, CY 120→180) — verified working
- `cone_follower_node.py` rewritten: optical→camera_link frame conversion, midpoint-angle steering instead of PID-on-Y-offset
- `simulation.launch.py` updated: bridge CameraInfo→`_raw`, fix node republishes corrected version

**What's still broken:**

- Steering saturates at -0.5 rad (max right) continuously — kart doesn't recover
- `steering_gain=2.0` is too aggressive: any midpoint angle >14° saturates the steering clamp
- The midpoint angle is consistently ~19-45° in testing, so it's always clamped

**Next steps to try:**

1. Lower `steering_gain` to ~1.0 and retest
2. Check if the kart has already drifted off-course by the time the first YOLO detections arrive (cone_follower starts at t=6s, detections may come later)
3. Verify depth image quality — cone forward distances seem short (~1-3m for cones that should be ~5m away), which could be a depth image issue separate from CameraInfo
4. Consider adding a ramp-up delay or pausing Gazebo until all nodes are ready

## Train YOLOv11 Cone Detector (In-House)

**Status:** Dataset ready on Orin, blocked on PyTorch CUDA.

**Dataset (on Orin at `~/kart_brain/training/perception/data/`):**
- 23,450 images (19,933 train / 3,517 val), 479,934 annotations
- Sources: Prueba FSOCO (9.9k), FSAE Cone (9.9k), TBReAI (2.1k), ARECE 3 (1.5k)

**Blockers:**
- [ ] **Fix PyTorch CUDA on Orin** — torch crashes on import: `ImportError: libcudss.so.0`. Need Jetson-specific wheels from `pypi.jetson-ai-lab.dev` (was down as of 2026-02-22). Always pin `numpy<2` after reinstall.

**To train (once PyTorch CUDA is fixed):**
```bash
cd ~/kart_brain
python3 training/perception/train.py --epochs 100 --batch 16
```

**After training:**
- Evaluate and replace `models/perception/yolo/nava_yolov11_2026_02.pt` if better
- See `training/README.md` for full details

## Rebuild kart_brain on Orin

**Status:** Pending — build failed due to missing `ackermann_msgs` (now installed).

**What's needed:**
```bash
cd ~/kart_brain && colcon build
```

## ESP32 Communication

**Status:** Bridge nodes written, protocol documented, needs live testing.

**What's done:**
- `actuation_bridge_node.py` and `cmd_vel_bridge_node.py` exist
- `docs/ACTUATION_PROTOCOL.md` defines the serial protocol
- ESP32 firmware at `~/Desktop/kart_medulla`

**What's needed:**
1. Wire ESP32 via USB serial
2. Flash firmware: `cd ~/Desktop/kart_medulla && ~/.local/bin/pio run --target upload --environment esp32dev`
3. Test sending steering + throttle commands via ROS2 topics

## Investigate Zombie/Stale Process Accumulation

**Status:** Not started

**Problem:** ROS2/Gazebo processes accumulate over time (zombie processes, orphaned nodes, leftover `ign gazebo` or `ros2` processes from previous runs). This clutters the system and can cause port conflicts or resource issues.

**What's needed:**
1. Investigate which processes are leaking — ROS2 nodes, Gazebo, bridge, YOLO, etc.
2. Figure out why they aren't cleaned up on shutdown (missing signal handling? launch file issues?)
3. Consider solutions: cleanup script on start, proper `on_exit` handlers in launch files, a wrapper script that kills stale processes before launching, or a systemd-style approach
4. Implement and document whatever works

## Create Reproducible Setup Script / Guide for Orin

**Status:** Not started

**Goal:** Make it easy to set up a fresh Orin (or reinstall) by creating either an install script, a detailed step-by-step doc, or both — written so an AI agent can follow it autonomously.

**What to cover:**
1. Flash JetPack / base OS
2. Install ROS2 Humble + colcon
3. Install Jetson-specific PyTorch + torchvision (CUDA wheels, numpy<2 pin)
4. Install ZED SDK + ROS2 wrapper
5. Clone and build `kart_brain` (including `ackermann_msgs` and other deps)
6. Install Gazebo Fortress + `ros-humble-ros-gz` (optional, for sim)
7. AnyDesk / remote access setup
8. Any system config (udev rules for ESP32 serial, network, etc.)

**Format options (pick when starting):**
- `setup_orin.sh` script that does everything non-interactively
- A detailed `.md` doc in `kart_docs/` or `.agents/` written for AI agents to follow
- Both: script for the happy path, doc for context and troubleshooting

**Reference:** Consolidate info already scattered across this TODO, `.agents/` docs, and `kart_docs/`.

## Long-Term

- Explore YOLO acceleration via ONNX/TensorRT on Jetson
- Set up ZED ROS2 wrapper for proper depth + 3D cone localization (instead of webcam mode)
- Trajectory planning from cone positions
- Full autonomous loop: camera → detection → planning → actuation
