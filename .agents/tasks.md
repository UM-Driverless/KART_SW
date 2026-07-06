<!-- read in full — kept under 150 lines -->
# Agent Task Board

Actionable work items for AI agents. Derived from `TODO.md` (human roadmap).
See `AGENTS.md` → Task Management for conventions.

## Ready

- [ ] **ESP32 health frame: add per-task stack minima** — The dashboard's System page has a "Stack" card (min free stack bytes across the comms/control/heartbeat/health tasks, red <200 B) but it always shows `--`: the `ESP_HEALTH_STATUS` payload is only 4 int32s `[flags, agc, heap_kb, i2c_errors]` and carries no stack data. Work: (1) in **kart-medulla**, extend the health frame with 4 more int32s from `uxTaskGetStackHighWaterMark()` per task → `[flags, agc, heap_kb, i2c_errors, stack_comms, stack_control, stack_heartbeat, stack_health]`; (2) in **kart-brain**, extend `decode_health()` (`src/kb_dashboard/kb_dashboard/protocol.py`) to emit `stack_*` keys when `len(payload) >= 8`, staying backwards-compatible with 4-int payloads. The UI already consumes the keys unchanged (Race-skin System card `rcSysStk` + legacy healthBar `STK`). Flash, then verify the card populates and turns red when a task is squeezed.

- [ ] **Rename remaining deployment folders to hyphenated names** — Partially done: `~/kart_brain` → `~/kart-brain` on the **Orin** was completed 2026-07-06 (dir moved, `.bashrc` + systemd unit updated, clean rebuild, verified running). Remaining:
    - **Orin:** `mv ~/kart_medulla ~/kart-medulla` + update any flash scripts/aliases referencing the old path (PlatformIO flash command in docs/memory uses `~/kart_medulla`).
    - **VM (`utm`):** check whether the workspace is still `~/kart_brain` and rename to `~/kart-brain` (+ `.bashrc`) if so.

- [ ] **Pure pursuit arrow/steering mismatch — still unverified after HUD refactor** — Symptom reported 2026-04-19: with `pure_pursuit` active, the dashboard green arrow pointed right while the physical steering wheel moved left. Commit `316b5cd` eliminated *one* possible cause by making the HUD draw its arrow from the controller's actual aim point (`/kart/target`, PointStamped) instead of the HUD's independent nearest-pair midpoint — so the arrow can no longer disagree with pure pursuit's target. Not yet re-tested on the kart after that change. If the symptom persists, the mismatch is between `cmd.angular.z` and the actuator (not between arrow and controller) — which should be impossible since the geometric controller works fine through the same `cmd_vel_bridge_node.py:67` path with no sign flip. Code review of `_control_pure_pursuit` (`src/kart_control/scripts/cone_follower_node.py:484-582`) found no sign bug; `atan2(target_y, target_x)` uses the same `left` convention as geometric. Suspects if the problem returns: (a) far-lookahead target on the opposite side of a curve entry, (b) positive-feedback adaptive lookahead loop using `_last_steer` at line ~552, (c) `steering_gain=3.0` multiplying PP's already-valid angle into ±`max_steer` saturation (line ~570), (d) wide-straight cross-pairing with the 6 m cutoff (line ~527). Diagnostic: log `self._last_target` alongside nearest-pair midpoint on each frame, compare. Do NOT just disable pure pursuit — the user wants to use it, and the HUD fix may already be sufficient.
- [ ] **Validate ZED neural net inference via TensorRT** — Build ZED SDK with TensorRT support (check submodules), test that the custom YOLO model runs through ZED's built-in neural detection module instead of a separate Python node. Compare latency vs current ultralytics pipeline.
- [ ] **Benchmark C/C++ vs Python for YOLO node** — The current `yolo_detector_node.py` runs inference in Python (ultralytics). Investigate whether a C++ ROS2 node using TensorRT C API or ZED SDK's detection API gives meaningful FPS improvement. Check: inference time, pre/postprocessing overhead, GIL contention.
- [ ] **Per-cone depth: confirm and document the fast path** — Two perception launch files exist: (a) `perception_3d.launch.py` runs `yolo_detector_node` + `cone_depth_localizer_node` and consumes the **full** depth image from the ZED ROS 2 wrapper (`/zed/zed_node/depth/depth_registered`), then samples it at YOLO bounding-box centers; (b) `perception_zed_od.launch.py` uses the ZED SDK's built-in object-detection module (publishes `ObjectsStamped`, consumed via `zed_od_utils.zed_objects_to_det3d`), where the SDK runs the custom YOLO model on-device and returns 3D bounding boxes directly — the full depth image is never published to ROS in this path (commit `4b4fc0b` "Add ZED SDK built-in object detection support with custom YOLO model"). Path (b) is the actual fast path running on the kart. Two open questions: (1) is path (a) still used anywhere, or can `cone_depth_localizer_node.py` and `yolo_detector_node.py` be retired? (2) Does the SDK's object-detection path internally compute a full depth pass anyway and just hide it from us, or is it really only computing depth at detected objects? Related to the existing task at line 10 ("Validate ZED neural-net inference via TensorRT"). Document whichever path is canonical in `architecture.md` so future content posts don't have to re-derive it.

- [ ] **In-person verify: LAN-IP dashboard login after Secure-cookie fix** — Fix committed in `31d4bb7` on 2026-04-20 makes the session cookie's `Secure` flag conditional on `X-Forwarded-Proto: https` from a loopback peer (cloudflared). Not yet tested on-kart because Orin is off when Ruben is at home. Next time at the kart, after `git pull` + `sudo systemctl restart kart-brain` on Orin: (1) visit `http://<orin-lan-ip>:9090` from a phone on the same Wi-Fi, type password `0`, confirm the dashboard actually loads (not the login page again); (2) visit `https://kart.rubenayla.xyz`, confirm login still works there (regression check); (3) in Chrome/Safari devtools, inspect the `kart_session` cookie — LAN path should show `Secure: false`, Cloudflare path `Secure: true`. Close the task only after both paths are confirmed.

## In Progress

### Code shipped, awaiting on-kart validation — branch `feature/imu-corrected-ground-plane`

Both items below are coded, committed, pushed (commits `71dbdeb`, `6c50400`, `cec790d`). The remaining work is at the workshop. Boot order on the Orin:

```bash
git fetch && git checkout feature/imu-corrected-ground-plane
colcon build --packages-select kart_perception kb_dashboard --symlink-install
source install/setup.bash
# Terminal 1
ros2 launch kart_perception perception_zed_od_ground.launch.py
# Terminal 2
ros2 launch kb_dashboard dashboard.launch.py
# Browser
http://<orin-ip>:9090
```

- [ ] **(workshop) Validate the IMU-corrected ground plane** — `ground_plane_localizer_node` subscribes to `/zed/zed_node/imu/data` and `/zed/zed_node/obj_det/objects`, applies a swing-twist correction (strips yaw about gravity, keeps the residual pitch+roll), and republishes cones on `/perception/cones_3d_ground`. Validation:
    1. **Static, level kart** — `/perception/cones_3d_markers` (raw, in optical frame) and `/perception/cones_3d_ground_markers` (corrected, cyan/orange shades) should overlap exactly in RViz. If they diverge at rest, math/convention is off. First fix to try: set `invert_correction:=true` in the launch params. Second: change `gravity_axis` from `[0,0,1]` to whatever the wrapper actually publishes against.
    2. **Hand-tilt the camera ±10°** — original markers swing forward/back with the tilt; ground-corrected markers should stay roughly fixed in space. If they swing the same direction as the originals, flip `invert_correction`.
    3. **Drive forward + slam the brakes** — original cone distances will drop systematically (kart pitches forward → camera looks down → cones look closer). Corrected positions should hold roughly constant. Record a rosbag of this run — it's the receipt for the LinkedIn dashboard post and the migration evidence for the MPC controller.
    4. **Sanity-check the IMU bias under sustained braking** — `ros2 topic echo /zed/zed_node/imu/data --field orientation` during the slam-brake test. If the quaternion swings noticeably more than the kart's actual pitch, the ZED's stock fusion is biased by linear deceleration. <30% extra: ship as-is. >50% extra: file follow-up to add gyro-dominant complementary filter weighting.
    - Only after the first three pass, mark this Done and start a separate PR migrating `cone_follower_node.py` (the MPC) to consume `/perception/cones_3d_ground`.
    - Rosbag to record at the workshop:
      ```bash
      ros2 bag record -o ground_plane_validation \
        /zed/zed_node/imu/data \
        /zed/zed_node/obj_det/objects \
        /perception/cones_3d \
        /perception/cones_3d_ground
      ```

- [ ] **(workshop) Validate the top-down (cenital) cone panel in the dashboard** — Default skin now shows a "Perception — Top-down" card with cones from `/perception/cones_3d_ground` rendered as colored dots, kart at the bottom, range rings at 5/10/15 m. Validation:
    1. **Empty FOV** — only the range rings, heading line, and white kart triangle. No stray dots.
    2. **One cone at ~3 m, slightly right** — should appear ~⅕ of the way up the canvas, slightly right of center. Color should match the cone's class (blue/yellow/orange/large-orange).
    3. **Cone wall at known 5 m / 10 m / 15 m** — dots should land near the corresponding range ring.
    4. **Tilt the camera by hand** — dots should NOT move. If they do, the IMU correction (above task) isn't doing its job; that's the failure to debug, not the panel.
    5. **Wrong direction?** If cones appear *behind* the kart (below the canvas) or mirrored left-right, edit the `CENITAL_*_SIGN` / `CENITAL_*_AXIS` constants at the top of `drawCenital` in `src/kb_dashboard/kb_dashboard/index.html` and reload the browser. No rebuild — `index.html` is served fresh on each page load.
    - Mark Done when the static + cone-wall + tilt tests all behave as expected on at least one phone or laptop.

## Blocked

## Done

- [2026-04-20] **Dashboard login works on LAN IP (plain HTTP)** — Session cookie `Secure` flag is now conditional on `X-Forwarded-Proto: https`, which cloudflared stamps on tunnel-forwarded requests. Only trusted when the peer is loopback (`127.0.0.1`/`::1`) so a LAN client can't spoof the header. Cloudflare path still gets `Secure`; direct LAN HTTP gets the cookie without `Secure` so browsers retain it and the login flow completes. Implemented in `src/kb_dashboard/kb_dashboard/server.py` (~line 115 peer check, ~line 139 cookie emission). Needs on-kart verification next session.
- [2026-04-20] **Dashboard: joystick-style XY plot replaces steering/accel numeric block (default skin)** — Replaced the Actuations + Targets cards in the default skin with one "Control — Target vs Actual" card: 220×220 canvas pad (X=steering, Y=throttle−brake), orange crosshair = target (`orin_cmd_*`), blue dot = actual (`esp32_*`), dashed error line between them; compact TGT/ACT/Δ numeric column on the right with labels left-aligned on the far right (Steer / Throttle / Brake / Str PWM). X-axis follows the kart's `+rad = left turn` convention (REP 103, matches HUD). Other skins untouched. Implemented in `index.html` via new `updateControlPanel()` + `drawControlPad()` helpers. Verified in browser with mock data; deployed to Orin; user confirmed after hard-refresh.
- [2026-04-20] **Shutdown button hidden in non-autonomous missions** — `updateMissionUI()` at `index.html:287` now hides a row's children *individually* (instead of the whole row), skipping any element whose classList includes `shutdown`. All 5 skin Shutdown buttons already had that class except KITT's; added there too. Start/Stop/EBS/Restart still gated behind autonomous missions as before.
