# TODO

## HIGH PRIORITY (Firmware & Core)
- 

## MEDIUM PRIORITY (Optimization & Infrastructure)

- [ ] **PID tuning** -- tune kp/ki/kd now that steering gears are fixed and outputLimit works
    - Maybe some small integral component would be good
- [ ] Any way to determine our speed without pcb to read the hall sensors yet?
- [x] **Expose dashboard via Cloudflare Tunnel** (Ruben) -- `kart.rubenayla.xyz → http://localhost:9090` done. TODO: add Cloudflare Access (free) for auth.
- [ ] **Benchmark YOLOv10n vs v11n on Orin with TensorRT** -- export both, compare ms/frame. v10 removes NMS which may help.
- [ ] Create reproducible Orin setup script/guide

## LOW PRIORITY (Long-Term)

- [ ] Trajectory planning from cone positions
- [ ] Make a map of the university https://x.com/junyi42/status/2031024111716331759
