# TODO

## HIGH PRIORITY (Firmware & Core Perception)

- [ ] **PID tuning** -- tune kp/ki/kd now that steering gears are fixed and outputLimit works
- [ ] **Verify outputLimit clamp** -- send a target beyond limit, confirm it clamps correctly
- [ ] **Export YOLOv11n nano model to TensorRT FP16 for ~10ms inference** -- current PyTorch runs ~50ms/frame (19 Hz) at imgsz=640. Steps: (1) `yolo export model=best.pt format=engine half=True imgsz=640 device=0` on the Orin (must export on target device), (2) load the `.engine` file in yolo_detector_node (ultralytics handles it: `YOLO("model.engine")`). FP16 uses Orin's tensor cores. Target: 60-100 Hz.
- [ ] **New YOLOv11 nano model training** (y540, 300 epochs) -- will replace nava model

## MEDIUM PRIORITY (Optimization & Infrastructure)

- [ ] **Reduce ZED depth GPU load to speed up YOLO** -- ZED's `NEURAL_LIGHT` depth mode runs a neural net on GPU every frame, competing with YOLO (~45 Hz instead of 72 Hz in isolation). Try: (1) switch to stereo-based depth mode (`ULTRA`/`QUALITY`) which uses CPU instead, freeing GPU for YOLO; (2) reduce depth computation frequency (we don't need depth at 100 Hz); (3) try INT8 quantization for the YOLO TRT engine (~1.5x speedup).
- [ ] **Benchmark YOLOv10n vs v11n on Orin with TensorRT** -- export both with `yolo export model=yolo10n.pt format=engine half=True imgsz=640 device=0`, run inference on the same image, compare ms/frame. v10 removes NMS which may help.
- [ ] Investigate zombie process accumulation on Orin
- [ ] Create reproducible Orin setup script/guide

## LOW PRIORITY (Long-Term)

- [ ] Full autonomous loop: camera -> detection -> planning -> actuation
- [ ] Trajectory planning from cone positions
- [ ] Make a map of the university https://x.com/junyi42/status/2031024111716331759

---

# COMPLETED

- [x] **Fix kart oversteering with YOLO pipeline** (DONE 2026-03-11) -- steering_gain halved to 0.5, exposed as `steering_gain` launch arg in autonomous.launch.py. Diagnostic test in tests/test_steering_gain.py.
- [x] **Remove raw value display from dashboard skins** (DONE 2026-03-11) -- all four skins (Default, KITT, Tesla, HUD) updated to show calibrated degrees only.
- [x] **Include manual remote control of the kart via the dashboard** (DONE 2026-03-11) -- Gamepad API polling at 10 Hz, PWM/Angle toggle, steering/throttle/brake bars. Publishes to /kart/cmd_vel_manual.
- [x] **Investigate rviz window accumulation on Orin** (DONE 2026-03-11) -- cleanup logic added to run_live.sh, display_zed_cam.launch.py, autonomous.launch.py, gui.launch.py. Diagnostic test in tests/test_rviz_accumulation.py.
