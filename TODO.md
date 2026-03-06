# TODO

## Steering Calibration (ESP32)

**Status:** Calibration firmware exists (`cal_main.c`) but needs rework.

**Current state:** AS5600 raw = **4003** when physically straight. Design assumes center = 2048. Off by ~172°.

**What to build:**
1. Drive motor left at fixed PWM (~50%) until it hits the mechanical limit, record raw AS5600 value
2. Drive motor right at fixed PWM until limit, record raw AS5600 value
3. Compute center as midpoint of the two limits
4. Save left_raw, right_raw, center to NVS
5. Real firmware reads center from NVS on boot and uses it as `centerOffset`

**The 4096→0 wraparound:** The AS5600 is a 12-bit absolute encoder (0-4095). The jump from 4095→0 can land anywhere in the steering range depending on magnet orientation. After calibration we know left_raw and right_raw — if the jump falls between them (e.g. left=3800, right=200), the center computation must handle circular math. But once the center offset is applied, the firmware converts to a signed angle range (e.g. -0.7 to +0.7 rad) and the wraparound disappears — the physical steering range is always <360°, so after centering there's no jump in the usable range. **Not an issue after mapping.**

## Real-Time Sensor Dashboard (ESP32/FreeRTOS)

**Status:** Not started.

**Problem:** Reading a single sensor value (e.g. AS5600 raw) currently requires flashing a new firmware, waiting ~1 min for build, and capturing serial output. This is too slow for debugging.

**Ideas:**
- Add a "debug mode" message type in the binary protocol — Orin sends a request, ESP32 replies with all sensor values (raw ADCs, AS5600, hall sensors, DAC outputs, PID state) in one frame
- ROS2 node on Orin that decodes and prints/publishes these values
- Or: a lightweight UART command that the ESP32 responds to without needing ROS2 (e.g. send `0xAA 0x01 0xFF 0x__` → ESP32 dumps sensor state as text for 5 seconds, then resumes binary mode)
- Consider a web dashboard via ESP32 WiFi (low priority)

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

## ESP32 Steering Hardware — Partially Working

**Status:** Sensor reads working, motor not yet verified. Firmware deployed.

**What was fixed (2026-03-03):**
- **Root cause found:** I2C timeout was 1000ms × 2 transactions = 2s per read, blocking the 100ms control task. Reduced `I2C_MASTER_TIMEOUT_MS` to 50 in `km_sdir.c`.
- **Control task reordered:** feedback + actuator commands execute BEFORE the I2C sensor read, so even if I2C is slow, frames still flow.
- **AS5600 sensor confirmed working:** I2C scan finds it at 0x36, raw angle reads ~3956/4095 (~2.93 rad). Sensor data flows to `/esp32/steering` at ~2.25 Hz.
- **Heartbeat stable:** 0xDEADBEEF on `/esp32/heartbeat`.
- **Flash baud fix:** default 460800 fails on this board. Use `PLATFORMIO_UPLOAD_SPEED=115200` for flash.

**Modified files (on Orin at `~/Desktop/kart_medulla`, and local at `~/repos/kart_medulla`):**
- `components/km_sdir/km_sdir.c` — I2C timeout 1000→50ms
- `main/main.c` — init sensor test, control task reorder, seed initial angle

**What's left:**
1. [ ] **Verify motor moves** — sent 0.5 rad target via `/orin/steering` but angle didn't change. After sustained publishing at 10 Hz the serial link dropped (ESP32 or kb_coms_micro crashed). Need to:
   - Restart kb_coms_micro: `ros2 run kb_coms_micro KB_Coms_micro`
   - Send a single target: `ros2 topic pub --once /orin/steering kb_interfaces/msg/Frame "{type: 34, payload: [1, 244]}"`
   - Physically check if the steering motor moves
   - If no movement: check H-bridge power supply, GPIO27 (PWM) and GPIO14 (DIR) wiring
2. [ ] **Investigate ~2 Hz rate** — control task should run at 10 Hz (100ms period) but only achieves ~2.25 Hz. Each I2C read succeeds but takes ~400ms somehow. Might be `vTaskDelayUntil` interaction or `KM_SDIR_ResetI2C` being called too often.
3. [ ] **Test full pipeline** — once motor verified: `cmd_vel_bridge_node.py` → `/orin/steering` → ESP32 → motor → `/esp32/steering` feedback converges
4. [ ] **Serial flooding protection** — publishing targets at 10 Hz killed the link. The comms_task (20 Hz RX) may not handle bursts well. Consider rate-limiting or investigating crash cause.

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
