# TODO

## HIGH PRIORITY (Firmware & Core)

- [ ] **PID tuning** -- tune kp/ki/kd now that steering gears are fixed and outputLimit works

## MEDIUM PRIORITY (Optimization & Infrastructure)

- [x] **Expose dashboard via Cloudflare Tunnel** (Ruben) -- `kart.rubenayla.xyz → http://localhost:8080` done. TODO: add Cloudflare Access (free) for auth.
- [ ] **Benchmark YOLOv10n vs v11n on Orin with TensorRT** -- export both, compare ms/frame. v10 removes NMS which may help.
- [ ] Create reproducible Orin setup script/guide

## LOW PRIORITY (Long-Term)

- [ ] Trajectory planning from cone positions
- [ ] Make a map of the university https://x.com/junyi42/status/2031024111716331759
