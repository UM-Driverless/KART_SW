# TODO

## HIGH PRIORITY (Firmware & Core)

- [ ] **PID tuning** -- tune kp/ki/kd now that steering gears are fixed and outputLimit works

## MEDIUM PRIORITY (Optimization & Infrastructure)

- [ ] **Expose dashboard via Cloudflare Tunnel** (Ruben) -- add `kart.rubenayla.xyz → http://localhost:8080` ingress to `/etc/cloudflared/config.yml` on Orin. Enables real-time dashboard from phone while kart drives. Add Cloudflare Access (free) for auth so random people can't send throttle commands.
- [ ] **Benchmark YOLOv10n vs v11n on Orin with TensorRT** -- export both, compare ms/frame. v10 removes NMS which may help.
- [ ] Create reproducible Orin setup script/guide

## LOW PRIORITY (Long-Term)

- [ ] Trajectory planning from cone positions
- [ ] Make a map of the university https://x.com/junyi42/status/2031024111716331759
