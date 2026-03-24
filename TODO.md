# TODO

## HIGH PRIORITY (Firmware & Core)

- [x] **ESP32 comms watchdog** -- zero PWM output (steering + throttle) if no command received for 1s. Prevents holding last position when ROS nodes are killed. Also: manual mode disables power (zero PWM) on the ESP32 side.
- [ ] **PID tuning** -- tune kp/ki/kd now that steering gears are fixed and outputLimit works

## MEDIUM PRIORITY (Optimization & Infrastructure)

- [x] **Expose dashboard via Cloudflare Tunnel** (Ruben) -- `kart.rubenayla.xyz → http://localhost:9090` done. TODO: add Cloudflare Access (free) for auth.
- [ ] **Benchmark YOLOv10n vs v11n on Orin with TensorRT** -- export both, compare ms/frame. v10 removes NMS which may help.
- [ ] Create reproducible Orin setup script/guide

## LOW PRIORITY (Long-Term)

- [ ] Trajectory planning from cone positions
- [ ] Make a map of the university https://x.com/junyi42/status/2031024111716331759
