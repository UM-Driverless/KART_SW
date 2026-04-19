<!-- read in full — kept under 150 lines -->
# Agent Task Board

Actionable work items for AI agents. Derived from `TODO.md` (human roadmap).
See `AGENTS.md` → Task Management for conventions.

## Ready

- [ ] **URGENT — Shutdown button hidden in non-autonomous missions** — `updateMissionUI()` (`src/kb_dashboard/kb_dashboard/index.html:287-299`) sets `display:none` on `.ctrl-row` / `.t-ctrl-row` / `.h-ctrl-row` / `.a-ctrl-row` and every `.algo-select` whenever `currentMission` isn't in `AUTONOMOUS_MISSIONS`. While the kart sits in Manual (the default), the Start/Stop/EBS/Restart/**Shutdown** row is invisible, even though it's in the DOM. User hit this today trying to find the new Shutdown button (commit `42e1a31`). The Shutdown button should be reachable regardless of mission, since powering off the Orin is orthogonal to autonomous control. Likely fix: move the Restart + Shutdown buttons outside `.ctrl-row` (into their own always-visible row), or exempt them from the display:none rule — same for the other 3 skins' ctrl-row variants. Verify with Playwright after the change. Do NOT quietly re-enable the full ctrl-row in manual mode — user deliberately hid Start/Stop/EBS while manual-driving; preserve that.
- [ ] **Validate ZED neural net inference via TensorRT** — Build ZED SDK with TensorRT support (check submodules), test that the custom YOLO model runs through ZED's built-in neural detection module instead of a separate Python node. Compare latency vs current ultralytics pipeline.
- [ ] **Benchmark C/C++ vs Python for YOLO node** — The current `yolo_detector_node.py` runs inference in Python (ultralytics). Investigate whether a C++ ROS2 node using TensorRT C API or ZED SDK's detection API gives meaningful FPS improvement. Check: inference time, pre/postprocessing overhead, GIL contention.

## In Progress

## Blocked

## Done
