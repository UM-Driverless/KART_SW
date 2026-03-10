# TODO

- why is ZED publishing so slow?
- include manual remote control of the kart via the dashboard, so when manual button is clicked, we get joystick input. a switch to choose pwm or angle target for the steering, and the other axis will be desired acceleration/braking (will not work yet, but keep it in the UI)


## Immediate

- [ ] **Investigate rviz window accumulation on Orin** — launch file opens multiple rviz instances, causes freezes
- [ ] Fix kart oversteering with YOLO pipeline (steering_gain too aggressive)
- [ ] Remove raw value display from dashboard skins (steering is calibrated)

## ESP32 Firmware

- [ ] **PID tuning** — tune kp/ki/kd now that steering gears are fixed and outputLimit works
- [ ] **Verify outputLimit clamp** — send a target beyond limit, confirm it clamps correctly

## Perception

- [ ] **Export YOLOv11s to TensorRT FP16 for ~10ms inference** — current PyTorch runs ~50ms/frame (19 Hz) at imgsz=640. Steps: (1) `yolo export model=best.pt format=engine half=True imgsz=640 device=0` on the Orin (must export on target device), (2) load the `.engine` file in yolo_detector_node (ultralytics handles it: `YOLO("model.engine")`). FP16 uses Orin's tensor cores. Target: 60-100 Hz. Can also try imgsz=416 for even faster.
- [ ] New YOLOv11 model training in progress — will replace nava model

## Infrastructure

- [ ] Investigate zombie process accumulation on Orin
- [ ] Create reproducible Orin setup script/guide

## Long-Term

- Full autonomous loop: camera → detection → planning → actuation
- Trajectory planning from cone positions
- Make a map of the university https://x.com/junyi42/status/2031024111716331759
