<!-- read in full — kept under 150 lines -->
# Agent Task Board

Actionable work items for AI agents. Derived from `TODO.md` (human roadmap).
See `AGENTS.md` → Task Management for conventions.

## Ready

- [ ] **URGENT — Shutdown button hidden in non-autonomous missions** — `updateMissionUI()` (`src/kb_dashboard/kb_dashboard/index.html:287-299`) sets `display:none` on `.ctrl-row` / `.t-ctrl-row` / `.h-ctrl-row` / `.a-ctrl-row` and every `.algo-select` whenever `currentMission` isn't in `AUTONOMOUS_MISSIONS`. While the kart sits in Manual (the default), the Start/Stop/EBS/Restart/**Shutdown** row is invisible, even though it's in the DOM. User hit this today trying to find the new Shutdown button (commit `42e1a31`). The Shutdown button should be reachable regardless of mission, since powering off the Orin is orthogonal to autonomous control. Likely fix: move the Restart + Shutdown buttons outside `.ctrl-row` (into their own always-visible row), or exempt them from the display:none rule — same for the other 3 skins' ctrl-row variants. Verify with Playwright after the change. Do NOT quietly re-enable the full ctrl-row in manual mode — user deliberately hid Start/Stop/EBS while manual-driving; preserve that.
- [ ] **Pure pursuit arrow/steering mismatch — still unverified after HUD refactor** — Symptom reported 2026-04-19: with `pure_pursuit` active, the dashboard green arrow pointed right while the physical steering wheel moved left. Commit `316b5cd` eliminated *one* possible cause by making the HUD draw its arrow from the controller's actual aim point (`/kart/target`, PointStamped) instead of the HUD's independent nearest-pair midpoint — so the arrow can no longer disagree with pure pursuit's target. Not yet re-tested on the kart after that change. If the symptom persists, the mismatch is between `cmd.angular.z` and the actuator (not between arrow and controller) — which should be impossible since the geometric controller works fine through the same `cmd_vel_bridge_node.py:67` path with no sign flip. Code review of `_control_pure_pursuit` (`src/kart_control/scripts/cone_follower_node.py:484-582`) found no sign bug; `atan2(target_y, target_x)` uses the same `left` convention as geometric. Suspects if the problem returns: (a) far-lookahead target on the opposite side of a curve entry, (b) positive-feedback adaptive lookahead loop using `_last_steer` at line ~552, (c) `steering_gain=3.0` multiplying PP's already-valid angle into ±`max_steer` saturation (line ~570), (d) wide-straight cross-pairing with the 6 m cutoff (line ~527). Diagnostic: log `self._last_target` alongside nearest-pair midpoint on each frame, compare. Do NOT just disable pure pursuit — the user wants to use it, and the HUD fix may already be sufficient.
- [ ] **Validate ZED neural net inference via TensorRT** — Build ZED SDK with TensorRT support (check submodules), test that the custom YOLO model runs through ZED's built-in neural detection module instead of a separate Python node. Compare latency vs current ultralytics pipeline.
- [ ] **Benchmark C/C++ vs Python for YOLO node** — The current `yolo_detector_node.py` runs inference in Python (ultralytics). Investigate whether a C++ ROS2 node using TensorRT C API or ZED SDK's detection API gives meaningful FPS improvement. Check: inference time, pre/postprocessing overhead, GIL contention.

## In Progress

## Blocked

## Done
