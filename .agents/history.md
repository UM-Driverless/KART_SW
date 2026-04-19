<!-- consult selectively — grep, never read in full -->
# History

Chronological audit trail: *what* we found, *when*, *why* we decided things. Append new entries at the TOP. Never rewrite past entries — correct via a new one.

---

## 2026-04-20 — Pure pursuit arrow/wheel mismatch: architectural fix, not yet re-tested

User reported: with pure_pursuit, dashboard green arrow pointed right while the physical steering wheel turned left. Initial investigation of `_control_pure_pursuit` (`src/kart_control/scripts/cone_follower_node.py:484-582`) found no sign bug — same `(fwd, left = -pos.x)` convention as the working geometric controller. Ruled out the downstream pipeline too: `cmd_vel_bridge_node.py:67` applies no sign flip, and geometric works end-to-end through it.

User's architectural observation: the HUD arrow and the controller shouldn't be computed independently — they should come from the same endpoint. Agreed and refactored: each controller now stores `self._last_target` after picking its aim point; `_on_detections` publishes it to `/kart/target` (`PointStamped`, camera optical frame); `steering_hud_node.py` subscribes and projects that point to draw the arrow. Arrow now *cannot* disagree with whatever the controller decided. Commit `316b5cd`.

**Not yet verified on the kart** after that change. If the symptom persists, the mismatch is between `cmd.angular.z` and the actuator — which should be impossible given geometric works the same path. Open suspects, should PP still misbehave: (a) far-lookahead target on curve-entry opposite side, (b) adaptive-lookahead positive-feedback loop using `_last_steer`, (c) `steering_gain=3.0` saturating PP's already-valid angle. Full write-up in `tasks.md`.

## 2026-04-20 — Removed ZED-VIO-derived speed; disabled ZED pos_tracking

Decision: remove `/kart/speed` entirely. Commented out `_on_odom` callback and the odom subscription + publisher in `cone_follower_node.py`; replaced `self._actual_speed` usages in MPC (now plans at `max_speed`) and neural_v2 (input set to 0.0). Then disabled `pos_tracking_enabled` in `src/kart_bringup/config/zed_overrides.yaml` since nobody consumes the ZED odometry anymore — frees CPU/GPU for YOLO. Commits `316b5cd` + `02eda4d`. Why: the kart has no actual speed sensor, the VIO-derived value was unreliable, and the prior `TODO.md` item "Measure kart speed without hall sensor PCB" was closed by a feature that didn't actually work well enough to be trusted.

## 2026-04-20 — Added "Constant + Stop" speed controller + "Shutdown Orin" button

Constant + Stop: copies the Constant speed controller but returns 0 m/s when any detected cone has class `orange_cone` or `large_orange_cone` (for acceleration-style runs where the kart must halt at the finish marker). Commit `42f5feb`. Tested and working on the kart.

Shutdown button: added to all 5 dashboard skins alongside Restart; red, double-confirmation, backend runs `sleep 3 && sudo -S poweroff` so the WS ack reaches the browser before power cuts. Commit `42e1a31`. **Known issue, logged as urgent task:** `updateMissionUI()` (`index.html:287-299`) hides the entire `.ctrl-row` when `currentMission` isn't autonomous. In Manual (the default), the Start/Stop/EBS/Restart/Shutdown row is `display:none` in the DOM. Shutdown needs to be always-visible since powering off the Orin is orthogonal to autonomous control.

## 2026-04-20 — colcon --symlink-install is a misnomer for ament_python

Stale dashboard UI after `git pull + systemctl restart kart-brain` pointed to a misunderstood build behavior. With `--symlink-install`, `ament_cmake` packages (and Python scripts installed via CMake, like `kart_control`) *do* get symlinked — editing `src/` propagates without rebuild. But `ament_python` packages like `kb_dashboard` do **not**: `build/<pkg>/<pkg>/` is a symlink to src (good) but `setup.py develop` + the colcon plugin still require a rebuild step for `package_data` / changes to be served. The egg-link points to `build/<pkg>`, which houses copies, not live symlinks.

Fast rebuild loop documented in `.agents/notes.md`: `colcon build --symlink-install --packages-select kb_dashboard && sudo systemctl restart kart-brain`. Commit `e86dd63`.

## 2026-04-20 — Recreated `dev` branch from `origin/main`

`dev` had diverged oddly: zero unique commits, but a working-tree difference in `src/kb_dashboard/kb_dashboard/dashboard_node.py` (still had the `pwm_limit` param that `main` commit `99eacf6` had deliberately removed) and in `README.md`. Safest cleanup: `git branch -D dev` + `git push origin --delete dev` + recreate from `origin/main`. Nothing meaningful discarded since `dev` had no unique commits.
