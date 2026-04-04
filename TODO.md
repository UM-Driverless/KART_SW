# TODO

## HIGH PRIORITY (Firmware & Core)

- [ ] **Troubleshoot stop→start bug on Orin** -- Fixed in code (`state_machine_node.py`: "stop" now goes to AS_READY instead of AS_OFF when auto mission is active), deployed to Orin on `dev` branch, but still not working after reboot. Need to verify: is the node actually running the new code? Check with `ros2 topic echo /kart/state` after stop+start sequence. May be a dashboard-side issue (not re-sending start command, or WebSocket dropping).


## MEDIUM PRIORITY (Optimization & Infrastructure)

- [ ] **PID tuning** -- tune kp/ki/kd now that steering gears are fixed and outputLimit works
    - Maybe some small integral component would be good
- [ ] **ESP32 heartbeat slow to reconnect after service restart** -- takes ~75s after kart-brain restart. Should be a few seconds. Investigate KB_Coms_micro serial reconnection timing.
- [x] **Measure kart speed without hall sensor PCB** -- ZED VIO positional tracking enabled, speed published to `/kart/speed`, displayed in dashboard (branch `feature/zed-speed-estimation`)
- [x] **Expose dashboard via Cloudflare Tunnel** (Ruben) -- `kart.rubenayla.xyz → http://localhost:9090` done. TODO: add Cloudflare Access (free) for auth.
- [ ] **Copy remaining SVO files to Orin** -- From Google Drive `formula 22-23/DRIVERLESS/BIG FILES/Videos ZED2/Data/` → `orin:~/kart_brain/data/svo/`. Best done on LAN (`orin-local`). Already copied: `HD720_..._18-05-10.svo`, `VGA_..._18-29-25.svo`, `HD720_..._19-15-54.svo`. Remaining (~75GB): `21_11_21/` (5 files), `21_11_19/` (3 files), `21_02_14/` (5 files), 4 large HD720s (11-22GB each). Nov 2021 files most likely to have FS cones.
- [ ] **Benchmark YOLOv10n vs v11n on Orin with TensorRT** -- export both, compare ms/frame. v10 removes NMS which may help.
- [ ] **Dashboard SVO selector UX** -- Selecting an SVO file currently requires manually clicking Restart. Make it intuitive: auto-restart when source changes, show a loading indicator, or prompt the user. The user shouldn't need to know that a restart is required.
- [ ] Create reproducible Orin setup script/guide

## LOW PRIORITY (Long-Term)

- [ ] Trajectory planning from cone positions
- [ ] Make a map of the university https://x.com/junyi42/status/2031024111716331759
