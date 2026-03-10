# TODO

## Immediate

- [ ] **Investigate rviz window accumulation on Orin** — launch file opens multiple rviz instances, causes freezes
- [ ] Fix kart oversteering with YOLO pipeline (steering_gain too aggressive)
- [ ] Remove raw value display from dashboard skins (steering is calibrated)

## ESP32 Firmware

- [ ] **PID tuning** — tune kp/ki/kd now that steering gears are fixed and outputLimit works
- [ ] **Verify outputLimit clamp** — send a target beyond limit, confirm it clamps correctly

## Perception

- [ ] New YOLOv11 model training in progress — will replace nava model
- [ ] Explore YOLO acceleration via ONNX/TensorRT

## Infrastructure

- [ ] Investigate zombie process accumulation on Orin
- [ ] Create reproducible Orin setup script/guide

## Long-Term

- Full autonomous loop: camera → detection → planning → actuation
- Trajectory planning from cone positions
- Make a map of the university https://x.com/junyi42/status/2031024111716331759
