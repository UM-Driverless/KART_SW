# TODO

## Immediate — Steering Debug (2026-03-07)

**Problem:** Steering angle is wildly asymmetric (~30° left vs ~140° right) despite circular wrapping fix. Need to understand why.

**What we have now:**
- Dashboard prints `STEER deg=X raw=Y` at 2 Hz (via `get_logger().warn`)
- Raw AS5600 value is sent in steering frame (4 bytes: angle_i16 + raw_u16)
- Center offset calibrated to raw 1731 via NVS

**Next steps:**
1. [ ] Read raw values at full-left, center, full-right — confirm physical range
2. [ ] If range is physically asymmetric (broken gears), adjust scaling per-side
3. [ ] If wrapping bug persists, add more debug logging in `KM_SDIR_ReadAngle`
4. [ ] Fix dashboard port 8080 conflict — add `SO_REUSEADDR` to server.py
5. [ ] Remove steering debug print once issue is resolved

## Immediate — Dashboard Fixes

- [ ] Fix `SO_REUSEADDR` on server socket (port 8080 "address already in use" on every restart)
- [ ] Verify ZED IMU data shows on dashboard (QoS fix deployed, needs ZED camera running)
- [ ] Remove raw value display from all skins once steering is calibrated

## ESP32 Firmware

- [ ] **outputLimit fix deployed** — verify clamp works by sending a target beyond limit
- [ ] **Recalibrate steering more precisely** — current offset gives ~-5° when straight
- [ ] **Steering gears are broken** — teeth stripped, limited range. Needs physical replacement.
- [ ] **test_main.c** — KM_PID_Compute→KM_PID_Calculate fixed locally, needs verification

## Kart Simulation (VM)

- [ ] Fix kart oversteering with YOLO pipeline (steering_gain too aggressive)
- [ ] Train YOLOv11 cone detector (blocked on PyTorch CUDA on Orin)

## Infrastructure

- [ ] Investigate zombie process accumulation on Orin
- [ ] Create reproducible Orin setup script/guide
- [ ] Explore YOLO acceleration via ONNX/TensorRT

## Long-Term

- Full autonomous loop: camera → detection → planning → actuation
- ZED ROS2 wrapper for 3D cone localization
- Trajectory planning from cone positions
