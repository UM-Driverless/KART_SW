<!-- read in full — kept under 150 lines -->
# Agent Task Board

Actionable work items for AI agents. Derived from `TODO.md` (human roadmap).
See `AGENTS.md` → Task Management for conventions.

## Ready

- [ ] **Pure pursuit arrow/steering mismatch — still unverified after HUD refactor** — Symptom reported 2026-04-19: with `pure_pursuit` active, the dashboard green arrow pointed right while the physical steering wheel moved left. Commit `316b5cd` eliminated *one* possible cause by making the HUD draw its arrow from the controller's actual aim point (`/kart/target`, PointStamped) instead of the HUD's independent nearest-pair midpoint — so the arrow can no longer disagree with pure pursuit's target. Not yet re-tested on the kart after that change. If the symptom persists, the mismatch is between `cmd.angular.z` and the actuator (not between arrow and controller) — which should be impossible since the geometric controller works fine through the same `cmd_vel_bridge_node.py:67` path with no sign flip. Code review of `_control_pure_pursuit` (`src/kart_control/scripts/cone_follower_node.py:484-582`) found no sign bug; `atan2(target_y, target_x)` uses the same `left` convention as geometric. Suspects if the problem returns: (a) far-lookahead target on the opposite side of a curve entry, (b) positive-feedback adaptive lookahead loop using `_last_steer` at line ~552, (c) `steering_gain=3.0` multiplying PP's already-valid angle into ±`max_steer` saturation (line ~570), (d) wide-straight cross-pairing with the 6 m cutoff (line ~527). Diagnostic: log `self._last_target` alongside nearest-pair midpoint on each frame, compare. Do NOT just disable pure pursuit — the user wants to use it, and the HUD fix may already be sufficient.
- [ ] **Validate ZED neural net inference via TensorRT** — Build ZED SDK with TensorRT support (check submodules), test that the custom YOLO model runs through ZED's built-in neural detection module instead of a separate Python node. Compare latency vs current ultralytics pipeline.
- [ ] **Benchmark C/C++ vs Python for YOLO node** — The current `yolo_detector_node.py` runs inference in Python (ultralytics). Investigate whether a C++ ROS2 node using TensorRT C API or ZED SDK's detection API gives meaningful FPS improvement. Check: inference time, pre/postprocessing overhead, GIL contention.

## In Progress

## Blocked

## Done

- [2026-04-20] **Dashboard login works on LAN IP (plain HTTP)** — Session cookie `Secure` flag is now conditional on `X-Forwarded-Proto: https`, which cloudflared stamps on tunnel-forwarded requests. Only trusted when the peer is loopback (`127.0.0.1`/`::1`) so a LAN client can't spoof the header. Cloudflare path still gets `Secure`; direct LAN HTTP gets the cookie without `Secure` so browsers retain it and the login flow completes. Implemented in `src/kb_dashboard/kb_dashboard/server.py` (~line 115 peer check, ~line 139 cookie emission). Needs on-kart verification next session.
- [2026-04-20] **Dashboard: joystick-style XY plot replaces steering/accel numeric block (default skin)** — Replaced the Actuations + Targets cards in the default skin with one "Control — Target vs Actual" card: 220×220 canvas pad (X=steering, Y=throttle−brake), orange crosshair = target (`orin_cmd_*`), blue dot = actual (`esp32_*`), dashed error line between them; compact TGT/ACT/Δ numeric column on the right with labels left-aligned on the far right (Steer / Throttle / Brake / Str PWM). X-axis follows the kart's `+rad = left turn` convention (REP 103, matches HUD). Other skins untouched. Implemented in `index.html` via new `updateControlPanel()` + `drawControlPad()` helpers. Verified in browser with mock data; deployed to Orin; user confirmed after hard-refresh.
- [2026-04-20] **Shutdown button hidden in non-autonomous missions** — `updateMissionUI()` at `index.html:287` now hides a row's children *individually* (instead of the whole row), skipping any element whose classList includes `shutdown`. All 5 skin Shutdown buttons already had that class except KITT's; added there too. Start/Stop/EBS/Restart still gated behind autonomous missions as before.
