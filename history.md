<!-- consult selectively — grep, never read in full -->
# History

Chronological audit trail: *what* we found, *when*, *why* we decided things. Append new entries at the END (chronological, oldest first). Never rewrite past entries — correct via a new one.

---

## 2026-04-20 — Recreated `dev` branch from `origin/main`

`dev` had diverged oddly: zero unique commits, but a working-tree difference in `src/kb_dashboard/kb_dashboard/dashboard_node.py` (still had the `pwm_limit` param that `main` commit `99eacf6` had deliberately removed) and in `README.md`. Safest cleanup: `git branch -D dev` + `git push origin --delete dev` + recreate from `origin/main`. Nothing meaningful discarded since `dev` had no unique commits.

---

## 2026-04-20 — colcon --symlink-install is a misnomer for ament_python

Stale dashboard UI after `git pull + systemctl restart kart-brain` pointed to a misunderstood build behavior. With `--symlink-install`, `ament_cmake` packages (and Python scripts installed via CMake, like `kart_control`) *do* get symlinked — editing `src/` propagates without rebuild. But `ament_python` packages like `kb_dashboard` do **not**: `build/<pkg>/<pkg>/` is a symlink to src (good) but `setup.py develop` + the colcon plugin still require a rebuild step for `package_data` / changes to be served. The egg-link points to `build/<pkg>`, which houses copies, not live symlinks.

Fast rebuild loop documented in `.agents/notes.md`: `colcon build --symlink-install --packages-select kb_dashboard && sudo systemctl restart kart-brain`. Commit `e86dd63`.

---

## 2026-04-20 — Added "Constant + Stop" speed controller + "Shutdown Orin" button

Constant + Stop: copies the Constant speed controller but returns 0 m/s when any detected cone has class `orange_cone` or `large_orange_cone` (for acceleration-style runs where the kart must halt at the finish marker). Commit `42f5feb`. Tested and working on the kart.

Shutdown button: added to all 5 dashboard skins alongside Restart; red, double-confirmation, backend runs `sleep 3 && sudo -S poweroff` so the WS ack reaches the browser before power cuts. Commit `42e1a31`. **Known issue, logged as urgent task:** `updateMissionUI()` (`index.html:287-299`) hides the entire `.ctrl-row` when `currentMission` isn't autonomous. In Manual (the default), the Start/Stop/EBS/Restart/Shutdown row is `display:none` in the DOM. Shutdown needs to be always-visible since powering off the Orin is orthogonal to autonomous control.

---

## 2026-04-20 — Removed ZED-VIO-derived speed; disabled ZED pos_tracking

Decision: remove `/kart/speed` entirely. Commented out `_on_odom` callback and the odom subscription + publisher in `cone_follower_node.py`; replaced `self._actual_speed` usages in MPC (now plans at `max_speed`) and neural_v2 (input set to 0.0). Then disabled `pos_tracking_enabled` in `src/kart_bringup/config/zed_overrides.yaml` since nobody consumes the ZED odometry anymore — frees CPU/GPU for YOLO. Commits `316b5cd` + `02eda4d`. Why: the kart has no actual speed sensor, the VIO-derived value was unreliable, and the prior `TODO.md` item "Measure kart speed without hall sensor PCB" was closed by a feature that didn't actually work well enough to be trusted.

---

## 2026-04-20 — Pure pursuit arrow/wheel mismatch: architectural fix, not yet re-tested

User reported: with pure_pursuit, dashboard green arrow pointed right while the physical steering wheel turned left. Initial investigation of `_control_pure_pursuit` (`src/kart_control/scripts/cone_follower_node.py:484-582`) found no sign bug — same `(fwd, left = -pos.x)` convention as the working geometric controller. Ruled out the downstream pipeline too: `cmd_vel_bridge_node.py:67` applies no sign flip, and geometric works end-to-end through it.

User's architectural observation: the HUD arrow and the controller shouldn't be computed independently — they should come from the same endpoint. Agreed and refactored: each controller now stores `self._last_target` after picking its aim point; `_on_detections` publishes it to `/kart/target` (`PointStamped`, camera optical frame); `steering_hud_node.py` subscribes and projects that point to draw the arrow. Arrow now *cannot* disagree with whatever the controller decided. Commit `316b5cd`.

**Not yet verified on the kart** after that change. If the symptom persists, the mismatch is between `cmd.angular.z` and the actuator — which should be impossible given geometric works the same path. Open suspects, should PP still misbehave: (a) far-lookahead target on curve-entry opposite side, (b) adaptive-lookahead positive-feedback loop using `_last_steer`, (c) `steering_gain=3.0` saturating PP's already-valid angle. Full write-up in `tasks.md`.

---

## 2026-04-20 — GitHub org & branch-protection config after Alberto self-merged to main

Alberto merged his own PR #3 (Stanley controller) straight into `main` without review. Root cause was the org's **Base permissions = Admin**, which meant every org member had admin on every repo, including the ability to bypass any branch rule. Audit-trail of what's now configured:

**Branch protection (`main` only):**
- Require a pull request before merging — **on**
- Required approving reviews — **1**
- Dismiss stale approvals on new commits — **on**
- `require_code_owner_reviews` — off
- `require_last_push_approval` — off
- Enforce for administrators — **off** (admins can self-merge and push directly; intentional)
- Force push / branch deletion — blocked for non-admins
- No status-check requirements (no CI yet)

**Other branches (`dev`, `MPC_controller`, `autoresearch/mar15`, `feature/microros-integration`):** unprotected. Contributors can push / force-push / rebase freely.

**Org roles (`UM-Driverless`):** 20 members total. Owners visible in the People page sweep: `93Urbano`, `Alvar0P`, `rubenayla`, likely others. All rest are Members. Owners are the only people who can override branch protection regardless of repo-level role.

**Org Base permissions:** at time of writing still **Admin** — user was about to change it to **Write**, which will make the branch-protection rule actually effective for non-owners. Re-verify with `gh api repos/UM-Driverless/kart-brain/collaborators` after the change to confirm members drop to Write.

**Why this setup:** user is OK with self-merging *their own* PRs as admin, but wants at least peer review among contributors (two non-admins can approve each other's PRs — GitHub doesn't require approvers to be admins). No automatic notification is sent to users whose role changes, so any downgrade should be announced in team chat.

**Also this session:** Alberto's Stanley commit was cherry-picked onto `dev` (preserving him as author, commit `7e66399`) and the merge was reverted on `main` via `git revert -m 1 d33dd4a` (commit `c200e56`), which keeps PR #3 recorded as merged-then-reverted rather than force-rewriting history.

---

## 2026-04-20 — Dashboard offline-access: decided on phone-hotspot + LAN URL

Problem: the Orin loses WiFi from the shop router whenever the kart moves, and the team was opening `kart.rubenayla.xyz` through their phone's 4G hotspot — which silently routes via Cloudflare → cellular → back, burning mobile data even for what is physically a 2-metre link.

Explored three options:

1. **Phone hotspot + use the LAN URL** — phones on hotspot speak L2 to the Orin over WiFi-direct, no cellular round-trip, zero data cost. Requires using `http://<orin-ip>:9090` (not `kart.rubenayla.xyz`). Added a ⓘ popover in the dashboard topbar today that surfaces the live LAN IPs + clickable URLs, and a README section, so nobody has to memorize IPs. **Chosen solution.**
2. **USB WiFi dongle → Orin emits its own "kart-local" AP** with `hostapd` + `dnsmasq`. Fully offline, no phone/router needed. ~€10 hardware, ~30 min config. Kept on the shelf — not worth the setup cost right now given the hotspot path works.
3. **Built-in WiFi in AP+STA concurrent mode** — theoretically possible on the Orin's radio but driver-flaky on Realtek/MediaTek chips. Rejected as unreliable.

**Why hotspot wins for us today**: no hardware to buy, no driver babysitting, phone follows the kart so range is tied to how close the user stands, and the ⓘ button solves the "remember the IP" friction. Data cost is zero as long as everyone uses the LAN URL (the Cloudflare URL stays as a from-anywhere backup).

**Recommendation for stable LAN URL**: give the Orin a static DHCP reservation *on the phone's hotspot* (some phones support it, some don't) or a static IP via NetworkManager on the Orin. Otherwise the IP changes per session and the user has to re-open ⓘ.

Re-open this decision if: (a) hotspot reliability in the field proves worse than expected, (b) a team member actually wants to run without a phone nearby. Then pursue option 2.

---

## 2026-04-21 — Steering sun-gear failure root-cause: nylon creep on a D-flat is fundamentally wrong

The planetary reducer's sun gear keeps failing, and the failure mode is not tooth shear — it's the **nylon bore rounding out around the motor shaft's D-flat** until the sun spins freely on the shaft. Fails preferentially on sudden direction reversals.

Why nylon + D-flat is a bad pairing, independent of PID tuning:

- A D-flat transmits torque through a tiny line-contact along the flat edge. Stress concentrates sharply on the nylon at that edge.
- Nylon **creeps** under sustained stress, and creeps fast under *repeated impulsive* stress. Every direction reversal with any backlash in the coupling (which a shallow D-flat always has) loads that edge with an impact, not a smooth torque.
- Over thousands of reversals the bore plastically deforms: the flat wears round, backlash grows, impacts get harder, damage accelerates. Classic runaway failure.
- This is why software mitigations (slew limits, output deadband, D-term LPF) can *slow* the failure but not prevent it — the root cause is material + geometry, not control.

**Constraint**: the sun gear is very small — no room for a hybrid solution (steel insert pressed into the nylon bore, brass hub with nylon teeth, etc.). Whatever we use has to be a monolithic part in basic sun-gear geometry. That rules out the usual "metal hub + plastic teeth" compromise.

**Material mismatch — actually works in our favour**: the planets and ring are also nylon, so a brass sun will be the hardest part in the stack. This sounds like it migrates the problem, but the geometry makes it a genuine upgrade:

- The **sun is the highest-duty part** in a planetary (rotates fastest, every tooth engages every revolution). Making the highest-duty part the durable one is exactly correct.
- **Three planets share the load**, each rotating slower than the sun. Wear spreads over ~3× more teeth rotating several× less often per unit time → roughly an order of magnitude gentler per-tooth duty than the sun sees.
- The **ring gear is shielded** — it only meshes with nylon planets, never directly with the brass sun. Its interface stays nylon-on-nylon, same as today.
- **Planets are simple parts**: plain spur gears on a smooth bore pin, **no D-flat, no keyway, no critical torque-transfer feature**. 3D-printable, easy to batch as spares, trivially swappable. The failures we're avoiding on the sun come from the D-flat interface specifically — planets don't have that problem.
- **Failure-mode upgrade**: today, sun bore rounds out → spins freely on the shaft → sudden total loss of steering with no warning. After: planet teeth wear gradually → backlash grows → audible/visible well before functional failure → scheduled maintenance.

**Decision**: if the rest of the steering stack (firmware PID, actuator, comms) proves out well enough to be worth committing to, we CNC a **brass sun** in the same geometry. Drop-in replacement, shifts wear from a catastrophic surface to a graceful one.

**Gate on doing the CNC work**: electronic steering must demonstrate it actually works (reliable target tracking, no runaway, usable under mission modes) before investing the machining hours. Until then, software mitigations + careful operation buy test time on the current nylon parts.

Re-open if: planet wear turns out to be surprisingly fast in practice (unlikely but measure it), or the CNC access falls through and we need a plan B.

---

## 2026-06-06 — ZED coordinate system + live orientation relative to gravity (verified on Orin)

Cone 3D positions flow through `/perception/cones_3d` in the **optical frame** (`zed_left_camera_optical_frame`): **X = right, Y = down, Z = forward (depth)**, right-handed. This is fixed by `cone_depth_localizer_node.py:114-127` using `image_geometry.PinholeCameraModel.projectPixelTo3dRay` (always optical convention) — *not* a per-run/config setting. `frame_id` is inherited from the ZED `camera_info` header. So a cone dead ahead reads `(x≈0, y≈+, z=distance)`. The ROS *body* frame `zed_camera_link` uses the other REP-103 convention (X-fwd, Y-left, Z-up), but cone messages are NOT in that frame. Dashboard top-down view (`/perception/cones_3d_ground`) plots `{x, z}` (x lateral, z forward), dropping the down axis.

Live verification (`ros2 topic echo /perception/cones_3d --once`): frame_id `zed_left_camera_optical_frame`; sample cones `(x=+0.59, y=+0.49, z=+1.55)` and `(x=-0.98, y=+0.34, z=+0.97)` — signs consistent with right/down/forward.

**Orientation relative to gravity** (from `/zed/zed_node/imu/data`, kart stationary): accel `(−0.54, −0.20, 9.76) m/s²` (|a|≈9.78≈g), fused quaternion `(−0.011, 0.028, −0.001, 0.9996)` → total tilt `2·acos(w) ≈ 3.4°`. Gravity re-expressed in the optical frame ≈ `(−0.2, +9.76, +0.54)`, i.e. points almost exactly down the **+Y axis**. The camera is mounted essentially **level**: optical **+Y ≈ gravity (down)** within ~3.4°, with **~3° nose-down pitch** (small +Z/forward gravity component — sensible for looking at cones on the ground) and **~1.3° roll** (right side slightly up). Note: IMU publishing must be enabled to read this live — `zed_overrides.yaml` had `sensors_pub_rate: 0.0`; the `imu/data` topic was active during this check.

Access gotcha hit during this session: the public Cloudflare hostnames (`kart.rubenayla.xyz` HTTP 530, `orin.rubenayla.xyz` SSH `websocket: bad handshake`) intermittently fail when the Orin's `cloudflared` daemon drops its connection — a restart of `cloudflared` on the Orin fixes it.

---

## 2026-07-06 — Landscape "Race" dashboard skin (phone holder in the kart is horizontal)

Context: a phone holder was mounted in the kart that holds the phone **horizontally**. The dashboard was designed portrait-first (480px column, vertical scroll), which no longer matches how it's physically used on the kart. Decision: redesign for landscape, and replace vertical scrolling with **horizontal panel swiping** — while driving/testing you flick between full-screen pages instead of scrolling a column.

**Style source**: an HTML mockup (`kart-dashboard.html`, made outside the repo with simulated data) with a Formula-Student racing look — carbon-fibre weave background, Ü MOTORSPORT red `#e2001a`, Orbitron/Rajdhani/Titillium Web fonts, clip-path chamfered panels with rivets and a red corner accent, hazard-stripe divider, steering needle gauge (±90° SVG), huge central speed readout, throttle/brake pedal bars, status-dot warn strip, portrait "rotate your device" guard. Copied verbatim into the repo as the design reference: `src/kb_dashboard/reference/horizontal-race-mockup.html`.

**Implementation decision**: added as a new skin (`race`) in the existing skin system in `src/kb_dashboard/kb_dashboard/index.html`, not a rewrite — the portrait skins (Default/KITT/Tesla/HUD/Artemis) stay available from the same dropdown, and all the shared plumbing (WebSocket feed, mission/state helpers, `updatePedals`/`updatePills`/`drawCenital`, HUD JPEG stream) is reused.

**Data mapping — mockup fakes vs what the kart actually publishes**:
- Mockup's battery % / motor temp / motor RPM **don't exist on the kart** (no sensors). The right-hand telemetry stack shows real values instead: YOLO inference Hz, ESP32 link Hz, ESP32 free heap kB — same card style, same bars.
- Speed hero ← `esp32_speed` (m/s → km/h). Steering gauge needle ← `esp32_steering_rad`; a second thin amber needle shows the commanded target (`orin_cmd_steering_rad`) so PID tracking error is visible at a glance.
- Accel bar under the speed ← `esp32_accel_lon` (shown in g, green forward / red braking), caption also shows lateral g.
- Pedal bars ← `esp32_throttle` / `esp32_braking`; warn strip dots ← MAG/I2C/HEAP health flags + ESP32 heartbeat (replacing the mockup's BMS/INVERSOR/TELEMETRÍA).
- Steering sign: positive rad renders to the **left**, matching the data convention (REP 103 yaw, positive = left turn). Originally the gauge and the steer bars rendered positive to the right, contradicting the control pipeline; corrected 2026-07-18 — see the entry at the end of this file.

**Panel layout (swipe left/right, or tap the tabs in the bottom bar)**:
1. **TELE** — the mockup's main screen: steering gauge | speed hero | YOLO/ESP/heap stack.
2. **MISSION** — mission grid, steering/speed algorithm dropdowns, Start/Stop/EBS/Restart (reuses `misGrid`/`.ctrl-btn` classes so `updateMissionUI` gating works unchanged), state/mission/HB pills. The remote-control joystick widget is *moved* into this page while the skin is active (DOM node relocation, restored on skin switch) so its nipplejs listeners survive.
3. **VISION** — live HUD camera stream (the global `#hudStream` img node is likewise relocated into this page) + cenital top-down cone view.
4. **SYSTEM** — full health readout: magnet, I2C errors, heap, AGC, YOLO Hz, ESP Hz, heartbeat age, AS state.

Top bar is the global one restyled (brand red Orbitron title, live pulse dot, skin selector, ⓘ net info, power off) with the hazard stripe under it; bottom bar is persistent: pedals left, page tabs center, warn dots + fullscreen (with landscape orientation lock) right. Portrait shows a "rotate the phone" guard but keeps the top bar visible so you can still switch skins. All UI text is English (the mockup was Spanish; user asked for English — matches the rest of the dashboard).

---

## 2026-07-06 — Top-down cone view live on the kart: dashboard falls back to raw cones_3d

Why the top-down (cenital) view was empty while the HUD showed detections: the dashboard only listened to `/perception/cones_3d_ground` from `ground_plane_localizer`, but that node (a) only runs in the separate `perception_zed_od_ground.launch.py`, not the systemd launch, and (b) consumes **ZED built-in OD** (`ObjectsStamped`) — it sits idle under the default YOLO pipeline even if launched. The 2D HUD path (`steering_hud` ← `/perception/cones_2d`) is independent and was always fine.

Fix (commit `df3a5c9`): `dashboard_node` also subscribes to raw `/perception/cones_3d` and feeds the view from it whenever the ground-corrected topic has been silent >1 s. Corrected data automatically wins when the workshop validation launch is running. Verified live on the kart: workshop cone at (0.84 m left, 0.74 m fwd) renders in the Race skin's top-down panel.

Also merged `feature/imu-corrected-ground-plane` into `dev` (merge `f604ff3`): ground-plane localizer + validation launch, Race skin, cenital view, kart_brain→kart-brain path renames. Deployed on the Orin (`dev`), all nodes up. **`main` untouched — waits for the kart physically driving with visual validation, per AGENTS.md.**

---

## 2026-07-06 — Kart Wi-Fi AP is live and is now the DEFAULT operating mode

The Orin's Wi-Fi (`wlP1p1s0`, RTL8822CE) now runs as its own access point instead of joining lab/phone networks: NM connection `kart-ap`, SSID **`kart`**, WPA2 password **`umotorsport`**, `ipv4.method shared`, autoconnect priority 200 (wins over every client profile on boot). Verified working by the user in person: the network appears (the Mac's Wi-Fi list just took a while to refresh — the AP had been beaconing fine on channel 1/2.4 GHz all along), and **internet sharing to AP clients works** through the chain iPhone (USB-C tether, Personal Hotspot) → Orin → NAT (`MASQUERADE 10.42.0.0/24`) → Wi-Fi clients.

**Default operating mode from now on:**
- Dashboard for everyone at the kart: join Wi-Fi `kart` → `http://10.42.0.1:9090`. Zero internet required.
- SSH at the kart: `ssh orin-local` (now pointing at `10.42.0.1` in the Mac's `~/.ssh/config`).
- Internet for the Orin and remote access (`orin-remote` / Cloudflare tunnel / `kart.rubenayla.xyz`): plug Ruben's iPhone in via USB-C with Personal Hotspot on. Without it, everything local still works.
- Expect double NAT and cellular speeds for AP clients' internet — fine for the dashboard and browsing.

Rollback if ever needed: `sudo nmcli connection down kart-ap` and `sudo nmcli connection up "<old wifi>"`. Full details in `.agents/orin-environment.md`.

---

## 2026-07-06 — Race skin: cockpit dials, targets on the instruments, Control page removed

Series of UI decisions from today's experimentation session (all local, `?demo=1` simulated telemetry; Orin off — deploy pending):

- **Telemetry page became an aircraft-style instrument cluster**: speed is a round 270° dial (0–50 km/h after the user corrected the kart's ~45 top speed; redline from 45) with a small digital readout kept below it; the old YOLO/ESP/heap bar-cards became mini round gauges with green/amber/red range arcs; an analog clock fills the fourth cell (experimental — swap for a lap timer if it earns nothing). New reusable canvas painter `rcDrawDial`. **Dials must never get a percentage `height`** — only max-width/max-height, or they squash into ellipses on odd-shaped screens (bug found on a 704×562 window).
- **Targets ride the same instruments instead of a separate panel**: the steering gauge already had the amber command needle; the pedal bars got amber target ticks (`orin_cmd_throttle/brake`); steering PID PWM is a small readout under the gauge. With that, the "Target vs Actual" XY pad + TGT/ACT/Δ table was redundant and was deleted.
- **Control page removed entirely** (back to 4 pages): its G-G diagram moved into the telemetry mini-cluster, replacing the ESP32-link dial — rationale (user): ESP32 link rate should be high and boring, it's health data, not telemetry; it stays on the System page only.
- **Health consolidated into a self-alarming System tab**: every legacy health-bar field is a System-page card with its explanation printed on it (AGC folded into the magnet card; new Stack card — shows `--` until the firmware sends per-task stack minima, task filed in `tasks.md`). The System tab pulses red when anything is unhealthy; the four MAG/I2C/HEAP/HB bottom-bar dots were removed.
- **`?demo=1` mode added**: simulated random-walk telemetry (speed, steering, pedals, drifting cones) so UI work needs no kart. This is the standing way to iterate on the dashboard from home.

---

## 2026-07-07 — Why the dashboard telemetry broadcast stays at 10 Hz (don't raise it)

Question: is it worth updating the dashboard faster than 10 Hz?

**Where the rates live:**
- Telemetry broadcast to browsers: `broadcast_loop()` in `src/kb_dashboard/kb_dashboard/server.py:417`, set by `await asyncio.sleep(0.1)` → 10 Hz. This is an asyncio loop, **not** a ROS timer.
- Command publish (joystick → kart): `create_timer(0.01, self._publish_pending)` in `dashboard_node.py:180` → 100 Hz, a real ROS timer on the spin thread (cross-thread-safe bridge from the asyncio WS handler).
- HUD JPEG stream already throttled to 20 Hz to avoid stealing CPU from YOLO.

**Decision: keep the broadcast at 10 Hz.** Reasoning:
1. **Binding constraint is Orin CPU, not bandwidth.** YOLO runs at 70–84 Hz and is the critical job; each broadcast serializes state to JSON and sends to every connected client. Tripling the rate triples that cost for near-zero benefit. The JSON snapshot is small — bandwidth was never the limit; the JPEG HUD frames are.
2. **Human perception saturates ~10–15 Hz** for reading numbers/gauges — the eye can't read digits faster.
3. **Source-rate ceiling** — if the ESP/IMU data doesn't arrive faster than 10 Hz, a faster WebSocket just resends the same value.

**Where faster *would* help, and the cheaper fix:**
- Visual smoothness of moving elements (G-G dot, needles) looks "steppy" at 10 Hz. Fix in the **browser for free**: interpolate in JS between received values and paint at 60 Hz with `requestAnimationFrame`. Separate data rate (10 Hz, cheap) from render rate (60 Hz, smooth). Don't touch the network rate.
- Fast transients (accel/G spikes on impact, vibration) alias at 10 Hz. Capture those in a **log on the Orin at the sensor's source rate**, not the live dashboard. Dashboard = human live monitoring; fine-grained analysis = separate recording.

---

## 2026-07-08 — ZED Tracking vs. SLAM architectural decision

**Context:** Designing the algorithmic roadmap for transitioning the kart from a reactive controller (pure pursuit/geometric) to a global trajectory planner capable of mapping the track and running multi-lap optimizations.

**Decision & Architectural Clarity:**
We need to clearly separate what the ZED SDK does internally versus what we need to build in ROS 2. 

1. **ZED Object Tracking (Enabled):**
   - **What it is:** A Kalman Filter / Nearest Neighbor tracker that assigns IDs to cones (e.g., "Cone #5").
   - **Why we need it:** It acts as an "invisible shield" against YOLO dropping frames due to glare or motion blur, and it filters out 1-frame ghost detections. It does **not** perform loop closures or map the track, but it dramatically cleans up the data sent to the downstream nodes.
   - **Gotcha:** Never assume this builds a map. If you see Cone #5 again a minute later on a second lap, the ZED will think it's a brand new cone.

2. **ZED Positional Tracking (VIO / Area Memory):**
   - **What it is:** The camera's internal visual odometry (which we already use for `/kart/speed`). If Area Memory is enabled, it *does* run an internal GraphSLAM on visual features (buildings, trees) to correct odometry drift.
   - **Gotcha:** It corrects the kart's trajectory but **does not map the cones**. We cannot rely on the ZED to give us a track map.

3. **Our ROS 2 SLAM Node (The Missing Piece):**
   - **What we need to build:** A lightweight GraphSLAM or EKF node that consumes the ZED's high-quality odometry and the ZED's tracked 3D cones.
   - **Why:** This node will perform the actual "springs" matching (Data Association + Least Squares optimization) on the cones themselves, detect when we cross the start/finish line (Loop Closure), and output the final, globally consistent track map for the trajectory planner.

---

## 2026-07-08 — Dashboard dead after reboot: empty sysctl persist file reverted the port-80 bind

**Symptom:** `kart.rubenayla.xyz` and `http://10.42.0.1/` stopped serving after an Orin reboot, even though `cloudflared` was active, the Orin had internet (iPhone tether up), and `systemctl is-active kart-brain` returned `active`. Nothing was listening on port 80.

**The two facts that pinned it:**
- `net.ipv4.ip_unprivileged_port_start = 1024` — the sysctl had reverted to the kernel default.
- `/etc/sysctl.d/99-kart-dashboard.conf` was **empty**.

**Root cause:** The dashboard was migrated from port 9090 to port 80 (commit "Dashboard binds port 80 directly — no iptables redirect"). Binding a port <1024 as a non-root process requires `net.ipv4.ip_unprivileged_port_start=80`. That value had only ever been set *live* (`sysctl -w`); the attempt to persist it wrote an empty file. The persist command was malformed — `echo "net…=80" | (echo 0 | sudo -S tee /etc/sysctl.d/99-kart-dashboard.conf)` — the inner `echo 0` (the sudo password) hijacked `tee`'s stdin, so `tee` wrote nothing. The live value held until the next reboot; then `systemd-sysctl` found an empty file, left the default 1024 in place, and the dashboard's non-root bind to :80 failed with permission denied and the node exited. Remote access was silently lost — nothing in the UI or `systemctl` state hinted at it, because the ROS launch as a whole stayed "active".

**Fix (permanent, verified):** Rewrote the file with `echo 0 | sudo -S sh -c 'echo net.ipv4.ip_unprivileged_port_start=80 > /etc/sysctl.d/99-kart-dashboard.conf'` (password on stdin, file content inside the command string — no stdin collision). Verified boot-proof: `sudo sysctl --system` (the exact path `systemd-sysctl.service` takes at boot) yields `net.ipv4.ip_unprivileged_port_start = 80`; `systemd-sysctl.service` is `static`/active and ordered before `kart-brain`, so the setting is in place before the dashboard binds. Dashboard back on :80, `kart.rubenayla.xyz` → 200.

**Prevention:**
- Never combine a `sudo -S` password pipe with a content pipe into the same command — they fight over stdin and the content silently loses. Use `sudo -S sh -c '...'` and put the file content inside the command string.
- After writing any `/etc/sysctl.d/*.conf`, verify with `sudo sysctl --system` (reproduces boot behaviour), not just `sysctl -w` — that catches an empty or ineffective file immediately.
- Note for reference docs: the dashboard is now on **port 80** (needs this sysctl), not `:9090`. `.agents/orin-environment.md` still says `:9090` in places — stale.

---

## 2026-07-08 — Battery BMS is readable over Bluetooth (JBD/Xiaoxiang protocol)

The 13S4P Molicel P42A pack has a **JBD/Xiaoxiang smart BMS** that advertises over BLE, and the Orin's built-in Bluetooth (`hci0`, IMC Networks radio) connects to it directly — no ESP32/CAN needed. This is a clean, independent source for the dashboard's battery gauge (which was showing `--` because it only reads battery from the now-dead `/esp32/health` link).

**How it was found:** `bluetoothctl scan on` from the Orin. Most hits are phones with randomized MACs and no name; the BMS is the one device advertising a **model-code name**: `SP22S003BP21S100A` at address **`A5:C2:37:39:58:5D`**.

**How to read it (reproducible):**
- Tooling: Python **bleak** (`pip3 install --user bleak`; `gatttool`/`bluetoothctl` also present). Orin BT service active, `hci0 UP RUNNING`.
- GATT: standard JBD layout — service `0000ff00`, **notify** char `0000ff01`, **write** char `0000ff02` (write-without-response).
- Protocol (JBD): write a command to `ff02`, read the reply as notifications on `ff01`. Frames are `DD <reg> <status> <len> <payload…> <chk> <chk> 77`, big-endian.
  - Basic info: send `DD A5 03 00 FF FD 77`. Payload: volt=`u16/100` V, current=`s16/100` A, remain=`u16/100` Ah, nominal=`u16/100` Ah, cycles=`u16`, protection=`u16@16`, **SOC=`byte@19` %**, cells=`byte@21`, ntc count=`byte@22`, then each temp `u16` in 0.1 K (`(t-2731)/10` °C).
  - Cell voltages: send `DD A5 04 00 FF FC 77`. Payload = N × `u16` mV.
- Reader script archived at `.agents/imports/` idea / lives on the Orin at `/tmp/read_bms.py` (to be promoted into a ROS node — see below).

**First reading (idle pack):** 52.84 V, SOC 99 %, 0.00 A, 13.26/16.80 Ah, 6 cycles, temps 28.9/25.9/27.2 °C, protection 0x0000; 13 cells 4056–4066 mV, **10 mV spread** (well balanced). Confirms 13S4P (16.8 Ah = 4 × 4.2 Ah P42A).

**Next step (organize/autostart):** fold this into the one clean startup rather than a side script. `kart-brain.service` → `ros2 launch kart_bringup launch.py` is the single entry point; add a **`kb_bms` ROS node** (bleak reader in a thread + ROS timer publishing `sensor_msgs/BatteryState`) to `launch.py`, and have the dashboard subscribe to it. Then it autostarts with everything else under the same service.

---

## 2026-07-11 — AS5600 steering encoder: decided on PWM-out (sensor mounted far), OTP burn justified

**Context / goal:** the **AS5600** magnetic rotary encoder (12-bit, addr `0x36`) is the kart's **steering-angle sensor**, mounted off-board on the steering shaft. It will sit **< 2 m** from the medulla PCB. Decision this session: read it as **PWM on the `OUT` pin**, *not* over I²C, because **I²C won't survive the cable run** to where the sensor is mounted (the handoff doc itself warns I²C doesn't tolerate long/noisy runs). `kart_medulla` (ESP32-S3 firmware) is the consumer. Handoff doc: `~/Downloads/as5600-esp32s3-handoff.md`.

**Bench wiring already done (4 wires, I²C — for configuration only):** `SCL→CN4-1`, `SDA→CN4-2`, `VCC→CN1-1 (+3V3)`, `GND→CN10-3`. **Powered at 3.3 V, not 5 V** (the module's 10 kΩ SDA/SCL pull-ups would otherwise push 5 V into the S3 GPIOs). GND on CN10-3 (CN1-3 occupied); CN9-3 avoided on purpose (steering/hydraulic actuator ground = noisy). USB alone powers 3V3 for bench work — no kart battery needed. On the S3 the I²C bus is **`SDA = GPIO 8`, `SCL = GPIO 9`** (per `kart_medulla/.agents/esp32s3-pinmap.md`, from the KiCad netlist); shared with on-board **PCF8574 `U25` at `0x20`**, so an I²C scan should show both `0x20` and `0x36`.

**Why PWM + why burn (the key decision — reverses my earlier "do not burn"):** OUTS defaults to `00` = analog. PWM is enabled by writing **CONF** (`0x08`, low byte) with **OUTS bits 5:4 = `10`** and **PWMF bits 7:6** = frequency. But CONF in RAM is **volatile** — and since I²C won't reach the sensor in the field, the ESP32 **cannot re-apply it every boot**. So the config must be made permanent by **burning the OTP** (`BURN_SETTING`, `0x40`, **once** — burns CONF+MANG). This is exactly the standalone-PWM-without-a-configuring-micro case that justifies burning. Burn procedure (irreversible): **on the bench**, 3.3 V with a **10 µF cap on VDD3V3→GND**, supply resistance **< 1 Ω** (short/thick wires), magnet present + detected (`STATUS 0x0B` bit5 MD=1); write CONF → `BURN_SETTING 0x40` → wait ≥1 ms → **power-cycle** → re-read CONF to confirm `OUTS==0b10`. Optional `BURN_ANGLE 0x80` (ZPOS/MPOS, ≤3× per ZMCO) to set a steering zero — not required; offset/scale can live in firmware instead.
```c
uint8_t lo = readReg8(0x08);
lo &= ~0xF0; lo |= (0b00 << 6) /*PWMF=115 Hz, robust over cable*/ | (0b10 << 4) /*OUTS=PWM*/;
writeReg8(0x08, lo);   // then BURN_SETTING to make it permanent
```

**Where the PWM lands on medulla — checked against the `dv-hardware` schematic:** **no free CN terminal** exists that reaches a PWM-capable ESP32 GPIO (the `EXP_P*` pins on CN3/CN5 are **PCF8574 expander outputs**, not raw GPIOs — useless for reading a pulse). The only unconstrained free S3 GPIOs are **38, 39, 13(MISO)**, and they come out on the **dev-module LEFT_HEADER, not on any CN**. GPIO 38 is earmarked for the EBS compressor PWM, so:
> **Route `OUT` → GPIO 39 (LEFT_HEADER Pin 14).** Solder to the header pin; flag hardware to break a spare GPIO out to a CN in the next PCB rev.

**Electrical (distance < 2 m):** PWM push-pull 3.3 V goes **direct** — no buffer needed at this length. Still: twist the OUT wire with GND, add a ~100–330 Ω series resistor at the source, and use the **low PWM frequency (115 Hz)** — edges degrade less over cable and steering needs almost no bandwidth. (Only past ~10 m would a Schmitt buffer / differential driver / I²C extender like P82B715 be worth it.)

**Firmware still to write (does not exist):** `km_medulla/components/km_sdir` is the existing AS5600 driver but it is **I²C-only** (reads `RAW_ANGLE 0x0C/0x0D`) and hardcodes the **classic-ESP32 pins SDA=21/SCL=22** in `KM_SDIR_ResetI2C`. For the PWM path we need **new pulse-width-capture** code — cleanest on the S3 is an **MCPWM capture unit** timestamping both edges → high-time / period → duty → angle. Calibrate duty→angle empirically (absorbs the steering zero). **Also blocking:** the **S3 build doesn't exist yet** — `platformio.ini` has only `esp32dev`/`native`, `km_gpio.h` is still the WROOM-32E map, and (per `.agents/error-log.md` 2026-07-10) that classic map puts `STEER_PWM` on GPIO 18, which on the S3 board is the safety-critical SDC MOSFET gate — so porting must guard the pin map with `#if CONFIG_IDF_TARGET_ESP32S3`, never flat-reuse it.

### 2026-07-11 (addendum, Rubén + Claude) — plan for the OTP burn session (tomorrow)

Continuing the entry above. Agreed procedure and open points so tomorrow's flash session has context:

- **Tomorrow's physical step:** solder a **10 µF cap VDD3V3→GND** right at the sensor module, then Rubén pings me to walk through the flash. The cap is a local energy reservoir so VDD3V3 doesn't droop during the ~1 ms burn current pulse (OTP is one-shot — a mid-burn voltage dip = corrupted, unrecoverable config). It is **not** about the ESP32's current capacity — the S3 devkit 3V3 LDO (hundreds of mA) is plenty; the risk is voltage droop from *wire/connector resistance*, which the local cap + short thick wires (<1 Ω source) fix. Do the burn with the sensor on **short bench wires next to the S3**, not through the 1.2–2 m harness.

- **Test-in-RAM-then-burn (mandatory ordering, because burn is irreversible):** first write `OUTS`=PWM to the **volatile CONF register only** and confirm real PWM is coming out of the `OUT` pin (scope / logic analyzer — does NOT need the not-yet-written MCPWM capture firmware). Only once PWM output is verified do we send `BURN_SETTING 0x40` to make it permanent. Then power-cycle and re-read CONF to confirm `OUTS==0b10`.

- **PWM-mode burn vs angle-zero are independent commands** — `BURN_SETTING (0x40)` burns CONFIG (holds `OUTS`) + MANG; `BURN_ANGLE (0x80)` burns ZPOS/MPOS (the zero/range). Burning PWM mode does not touch or require the angle zero. **OPEN QUESTION to resolve before we burn:** *where does the "good" steering zero currently live?* Rubén says the zero "came good, don't change it," but it matters which of these it is:
  - **(a) firmware software-offset** (Jorge's stated plan) → not in the chip at all, burn can't affect it, nothing extra to do. Clean case.
  - **(b) AS5600 `ZPOS` register set over I²C but never burned** → that's *volatile RAM too*, so in PWM-standalone (no I²C at boot) it is **lost on power-cycle** and PWM output reverts to raw magnet position. Would need to *also* `BURN_ANGLE`, or move the zero into firmware.
  - Regardless: the "good zero" was calibrated on the **I²C** read path. The PWM path is new firmware (duty→angle), so the zero must be **re-verified against the PWM reading** once that firmware exists — the number won't necessarily carry over 1:1. Note MANG can only be written if ZPOS/MPOS were never burned, so if we go firmware-zero we simply never touch BURN_ANGLE and avoid that interaction entirely.

- **Pre-burn sanity checks:** I²C scan should show `0x20` (PCF8574 U25) + `0x36` (AS5600); confirm magnet detected (`STATUS 0x0B` bit5 MD=1) before burning; read ZMCO to see if any angle burn was already done.

- Supply at **3.3 V not 5 V** (module ties VDD3V3/VDD5V; also its 10 kΩ SDA/SCL pull-ups would push 5 V into S3 GPIOs). USB alone powers 3V3 for bench work — no kart battery needed.

- **Pin correction 2026-07-11:** the entry above says "GPIO 38 earmarked for compressor, so steering → GPIO 39." The steering→39 conclusion stands, but the premise is stale — the compressor PWM was finalized on **CN8.2 / GPIO 3** (ex-buzzer MOSFET), so GPIO 38 is *also* free. GPIO 39 (LEFT_HEADER pin 14) is a **header solder** for the prototype, not on a CN. Cleaner permanent option (next PCB rev): repurpose a spare **pressure** input CN — we over-provisioned (3 pneumatic + 2 hydraulic; keeping both hydraulics, freeing one pneumatic). Best candidate **PRESSURE_3 (CN5.2, GPIO 1)** — straight to the ESP32, divider is 10 kΩ series + 10 k∥10 k pulldown, so **keep the 10 kΩ series as the PWM series resistor, remove the pulldown**. Full analysis + designators in `dv/kart/steering/as5600-pwm-burn-runbook.md`.

Hardware-side companion note lives in the DV vault `kart/steering/history.md` (2026-07-11) — the two entries are the hardware and firmware halves of the same decision.

## 2026-07-11 — Dashboard Battery tab + pack current limits

Added a 5th tab to the race skin (Telemetry · Mission · Vision · System · **Battery**). All BMS fields already reached the browser via `dashboard_node._on_battery`; the tab just renders them (plus I added `battery_charge` = remaining Ah and `battery_temps` = all NTCs to the forward). Layout: SOC progress ring + bidirectional current gauge side by side, 3 stat readouts (V / Ah / Temp), and a compact per-cell "battery-pill" grid (green level-fill with the voltage printed inside, min cell outlined red / max amber, min/max/Δ header, `13S` count shown so a dead cell channel is obvious).

**Pack current (from `~/dv/kart/README.md`) — the dial is whole-PACK current, not per-cell:** pack is **13S4P Molicel P42A**. Sign per the JBD BMS: **positive = charge, negative = discharge** (the dial states this itself: "PACK AMPS" + a dynamic CHARGE/DISCHARGE/IDLE caption). Discharge: 2000 W motor / (13 × 3.2 V) ≈ **48 A pack** (12 A/cell) at full power, nickel strips sized right at that peak → gauge amber −40…−50, red beyond −50. Charge: **~8 A/cell max** (Ruben's internal figure — this P42A charges unusually well for a Li-ion cell, still less than discharge) × 4P ≈ **32 A pack** → gauge amber +24…+32 (6–8 A/cell), red beyond +32. Capacity 13S4P ≈ 16.8 Ah / 786 Wh. Charge-side numbers are still an internal estimate — pin to the BMS charge-current limit if it's ever specified.

**Design direction — Ferrari-Luce hybrid gauge, now the standard for the race skin.** `rcDrawGauge`: tick scale + coloured zone arcs + short rim needle on the ring, big digital value in the CENTRE (only amber/red danger zones tint the number/needle — a green "ok" band is visual-only, else a healthy reading goes unreadable). `rcGaugeAngle` gives a longer 300° sweep (vs the old 270°). **Migrated all race round dials to it** (speed, YOLO, battery-mini, tank) and removed the now-dead `rcDrawDial` + `rcDialAngle` + the redundant separate speed digital readout (`rcSpeedVal`, the dial centre shows it now). SOC on the Battery tab stays a `rcDrawRing` donut (simple 0–100 %, reads like a fuel gauge). Tank keeps *dim* red/green bands, not bright — its below-min danger region is a big slice of the 0–10 scale and bright red looked alarming when healthy.

---

## 2026-07-18 — Dashboard steering sign corrected: positive rad now renders LEFT

The dashboard rendered a positive steering angle as a **right** deflection, while every
other part of the system treats positive as **left** (ROS REP 103 yaw; the joystick sender
does `sendAxes(-a.x, a.y)  // negate x: our positive=left`, and
`src/kb_dashboard/test/test_joystick_pipeline.py` asserts that a LEFT input produces a
positive `steer_rad`). So the dial disagreed with the data by a sign.

Affected widgets, all in `src/kb_dashboard/kb_dashboard/index.html`:
- Race-skin steering gauge — needle + target ghost needle rotation, and the LEFT/RIGHT/CENTER
  label and direction bar derived from it.
- Legacy-skin and Artemis-skin `steerBarFill` bars.
- The shared `updateSteerBar()` detail bar (actual fill + commanded tick).

The XY control pad (`drawControlPad`) was already correct — it maps positive to the left
(`CX - nAx*RX`) and labels the left edge "L" — which is what made the dial the outlier.

Fix: one hoisted helper, `steerScreen(rad) { return -(rad || 0) }`, applied at each point
where a steering value is converted to screen space. Rendering code downstream of it is
unchanged, so a positive screen degree still means "to the right of the viewer" and the
LEFT/RIGHT label logic needed no edit. Keeping the flip in a single function is the reason
this can't drift back out of sync skin by skin.

Verified in a browser (Playwright, page served from the package dir): feeding
`esp32_steering_rad = 0.5` gives `rotate(-28.6 100 108)` on the needle, the label reads
LEFT, and the direction bar fills from 34% to 50% — left of centre. Screenshot of the
gauge confirmed the needle tips up-and-left.

Possibly related: the open task "Pure pursuit arrow/steering mismatch" reports an arrow
pointing right while the wheel moved left. That task is about the HUD arrow rather than
this gauge, so this fix does not close it, but a viewer comparing the two would have seen
the gauge contradict the wheel for the same reason.

## 2026-07-18 — Battery gauge blank: two BlueZ behaviours, and a self-heal that caused the lockout

The dashboard's battery data had been dead, and the only known cure was a human restarting things.
Root-caused and fixed properly; `kb_bms` now recovers unattended. Fix in commit `52dc3b9`; the
operational summary lives in `.agents/orin-environment.md`.

### What was actually wrong

Two independent BlueZ behaviours stacked on each other. Neither is a fault in the pack, the radio or
the config — all three were fine the whole time.

**1. `bleak` could not see the pack at all.** Its BlueZ backend sets a discovery filter, which makes
`bluetoothd` issue `MGMT_OP_START_SERVICE_DISCOVERY` instead of a plain discovery. On BlueZ 5.64
that path reports *nothing* when it has no UUIDs to match against, and this pack advertises only AD
flags plus its name — no service UUIDs. The decisive comparison: `btmgmt find` and
`bluetoothctl scan on` both saw `SP22S003BP21S100A` at `A5:C2:37:39:58:5D`, RSSI -67, in the same
minute that `BleakScanner` returned **zero devices** — with the rest of the stack stopped, so nothing
was competing for the adapter. Filters made no difference: an explicit `RSSI: -100` and an explicit
`service_uuids=[ff00]` both still returned zero (the `ff00` filter cannot match, because `ff00` is a
GATT service exposed after connecting, not something in the advertisement).

**2. BlueZ destroys the device object as soon as discovery stops.** A discovered-but-unpaired device
is *temporary*, so when the scan exits the D-Bus object goes with it. A connect attempted a moment
after a scan therefore fails with `Device with address ... was not found` against a cache that was
populated seconds earlier — which reads like a caching bug and is actually documented behaviour.
Trusting the device does not, on its own, keep the object alive.

### The self-heal was the reason it never recovered

`kb_bms`'s recovery ran `bluetoothctl disconnect` + `remove` every third consecutive failure. But
`remove` deletes the BlueZ cache entry, and connect-by-MAC — the node's primary path — depends on
that entry existing. Its only fallback was `BleakScanner.find_device_by_name`, i.e. the blind path
from (1). So the recovery step *created* a state it could not get out of: the remove guaranteed the
lockout it was written to clear. Before this, every failure needed a human.

Worth keeping as a general lesson: the recovery routine had been reasoned about but never tested
against the failure it claimed to fix. It ran, it logged "clearing stale BlueZ state", and it made
things strictly worse — a self-heal that is never exercised is a plausible-looking guess.

The older "stale BlueZ holds `Connected: yes`" theory in `.agents/orin-environment.md` was also not
what was happening here. `busctl introspect` on the device path came back **empty** — BlueZ had no
record of the device at all, stale or otherwise, which is why the self-heal's disconnect/remove had
nothing to act on and changed nothing across nine attempts.

### The fix

- Discovery is driven by `bluetoothctl`, never `BleakScanner`.
- The scan runs as a **background process held open across the whole connect attempt**, then is
  terminated in a `finally`. This is what defeats behaviour (2).
- The pack is `trust`ed automatically once its object exists. Trust persists across reboots, so
  after the first success a cold start connects straight by MAC without needing the scan path.
- Any `remove` is now followed by the repopulating scan.

Deliberately **not** used: restarting the `bluetooth` service. It does clear the state, but Wi-Fi and
BT share one radio on this Orin, so it drops the `kart` AP with it — and a fix that needs a human to
run it was the thing being removed.

### Verification

Wiped every trace of the pack from BlueZ (`untrust`, `disconnect`, `remove`, confirmed until
`bluetoothctl info` reported "not available"), then a plain `systemctl restart kart-brain` and no
further intervention. It reconnected on its own in **~36 s** (node start 12:55:17 → `BMS connected`
12:55:53) and `/battery/state` published 52.28 V at 97%, `present: true`. Re-checked afterwards:
`Trusted: yes`, `Connected: yes`, 52.26 V.

An earlier iteration of the fix — repopulate the cache, then connect after the scan had exited —
still failed, and that failure is what exposed behaviour (2). Keeping the scan alive across the
connect is the part that matters; repopulating alone is a race.

### Environment notes worth reusing

- `bluetoothctl` non-interactively over SSH worked fine here, including `--timeout N scan on`,
  despite the warning in `.agents/orin-environment.md` that it tends to hang. `busctl` was still the
  better tool for asking whether a device object exists.
- `btmgmt find` needs the adapter free: it returns `status 0x0a (Busy)` while `kb_bms` is running its
  own retry scans, so stop `kart-brain` before using it to diagnose.
- The pack's address is **LE Public**, so it does not rotate. Connect-by-MAC is the reliable path and
  the name lookup is only a safety net for a replaced pack.

---

## 2026-07-18 — Black band at the top of the dashboard on iPhone Home Screen

Reported as a black margin above the topbar, visible on the phone but not in desktop Chrome.

Cause: added to the iOS Home Screen, the dashboard launches standalone, and `index.html` had
no `apple-mobile-web-app-*` metas. iOS then gives the app the *default* opaque status bar —
a band above the page that the page never paints, so it reads as black. Desktop browsers
never reserve that band, which is why it looked fine on the Mac. `viewport-fit=cover` was
already in the viewport meta but no rule used `env(safe-area-inset-*)`, so it did nothing.

Fix: declare `apple-mobile-web-app-capable` + `apple-mobile-web-app-status-bar-style:
black-translucent` (page runs under the status bar rather than below it), add `theme-color`
for Android, and inset `body` by `env(safe-area-inset-*)`. Padding rather than margin —
backgrounds paint across the padding box, so the band takes the skin's colour instead of
showing bare black. Left/right insets are included because in landscape the notch is on a
side. No skin overrides body padding, so all six inherit it.

**iOS caches the launch config when the icon is added.** Changing these metas has no effect
on an existing Home Screen icon — it must be deleted and re-added. Plain Safari tabs pick
the change up on reload. Confirmed fixed on the kart the same day.

---

## 2026-07-18 — Dashboard reduced to one skin, navigation moved to a left wheel, EBS tab added

Several related changes to `src/kb_dashboard/kb_dashboard/index.html`, all in the Race skin,
which is now the only skin.

**Five skins deleted.** Legacy, KITT, Tesla, HUD and Artemis are gone, along with the
selector that switched between them and six helpers left orphaned by their removal
(`updateControlPanel`, `drawControlPad`, `updateCenitalPanel`, `updateSteerBar`,
`updatePedals`, `updateTargets`). `applySkin` no longer takes an id or reads localStorage.
The file went from 3366 to about 2050 lines.

**Navigation moved from a bottom bar to a left rail, and pages now slide vertically.** The
dashboard is used on a phone in landscape, 844x390: width is the plentiful axis and height
the scarce one, and the round gauges are height-limited, so a 40px bottom bar cost dial size
on every page while an 88px rail costs nothing that matters. The EBS tank dial grew from
229px to 259px on that measurement.

**The rail is an endless wheel**, like a camera's mode dial: the selected name sits in a
fixed centre slot and the whole ring is dragged past it, wrapping in both directions.
Selecting by dragging needs no pointing accuracy, which is the reason for it — picking a
page with gloves on a moving kart should not require hitting a 36px target. Three things
this needed that are not obvious:
- The window is capped at five slots against six pages. The rail is ~334px but six names are
  only 216px, so a wrapped list in the full rail would show one name at the top *and* bottom
  at once, which destroys the illusion. Strictly fewer slots than items is the rule.
- Snapping animates with `requestAnimationFrame`, not a CSS transition, because the snap has
  to take the shortest arc; a transition interpolates the raw number and travels the long way
  round whenever the short way crosses the seam.
- The pages are a ring too, driven by the *same* position value as the wheel. An earlier
  version kept a linear page track and skipped the animation when wrapping, which also caught
  every ordinary multi-page jump, so some page changes slid and others snapped. Sharing one
  position makes the two incapable of disagreeing.

**Tap-to-select needs the pointer position, not `e.target`.** `setPointerCapture` retargets
every later pointer event to the element that captured, so `e.target` is never the button
that was tapped. Deriving the index from the pointer's Y coordinate is what makes tapping
work at all.

**Power button.** The complaint was that it was hard to hit. It is now 76x46 flush into the
top-right corner, 2.4x its original area, while the top bar got *smaller* (54px to 46px).
Two things made that possible: pinning the bar's height so it is not driven by its tallest
child, and putting the button in the corner — a screen corner can be hit by overshooting
rather than aiming, which is worth more than extra pixels in open space. Its chamfer is on
the bottom-left only, because `clip-path` clips hit-testing as well as painting and a
top-right chamfer would cut away the exact corner the approach depends on.

**An EBS tab** now holds the tank dial, piston and compressor bars, and rows for EBS state
and valve. Those two are not wired to anything yet and read NOT WIRED in grey. They are
backed by `ebs_state` and `ebs_valve_on` in `protocol.py`, defaulting to `None` rather than
`False` deliberately: a safety indicator that looks healthy because a field defaulted is
worse than one that reads unknown. States follow FS 2026 T 14.8 — unavailable / armed /
activated.

---

## 2026-07-18 — kb_dashboard test suite: six stale tests and one real bug in the test client

The suite was failing 6 to 9 of ~154 tests. Verified against unmodified `origin/dev` in a
scratch worktree before assuming anything, since recent dashboard work was the obvious
suspect and was in fact innocent — those commits touched only `index.html`, which no Python
test exercises. Two unrelated problems.

**Six deterministic failures were stale tests, not product bugs.** `decode_steering_raw` has
returned three values since commit `ae802d9` added the PID term, and production
(`dashboard_node.py`) already unpacked three; the tests still unpacked two. Separately,
`test_expected_missions` listed eight missions while commit `f41f1b7` had added
`autonomous`, which is a live button in the dashboard. In both cases the code was right and
the test was behind.

**The intermittent failures were a bug in the test's WebSocket client, not in the server.**
A WebSocket starts life as an HTTP request asking to upgrade; the server answers `101
Switching Protocols` and the connection speaks WebSocket from then on. The test read that
answer with `recv(1024)` — but `recv` means "up to this many bytes of whatever has arrived",
not "one message", because TCP is a byte stream with no message boundaries. The 101 response
is about 130 bytes and the server sends its `{"your_id": ...}` welcome immediately after, so
the two often arrive coalesced in one segment. The test checked the buffer contained "101",
then discarded the whole thing — welcome included — and afterwards waited five seconds for a
message it had already thrown away. When the two arrived separately the same code passed,
which is what made it intermittent, and a busier machine made it worse because more delay
before reading means more chance both are waiting together.

The fix is to read exactly to the end of the HTTP headers (the blank line `\r\n\r\n`) one
byte at a time and leave the rest of the stream untouched. 154/154 then passed on thirteen
consecutive runs, at a flat 7.4s instead of a jittery 8-22s — the spread had been nothing but
tests burning five-second timeouts.

Two hardening changes were made while chasing wrong theories and were kept, neither being the
cause: the fixtures now bind port 0 and learn the real port from the server via
`ready_callback`, rather than picking a free port and binding it moments later (two fixtures
could pick the same one); and `stop()` joins the server thread, since cancelling the task only
asks it to stop and every finished test was leaking a live thread. `ready_callback` has no
production caller, so its new argument is test-only, and the server now logs the port it
actually bound rather than the one it was asked for.

**Found underneath:** `encode_act_steering` builds a two-element payload while
`decode_steering_raw` reads three, and the ESP32 simulator uses that encoder — so the
dashboard's steering PWM readout is permanently 0% in simulation. Real firmware sends all
three. Logged in `tasks.md`.

---

## 2026-07-25 — Orin unreachable from home, and reopening the "Wi-Fi when no tether" question

### The Orin was not reachable, and the evidence says it was not running

Checked from the Mac with the Orin sitting on the desk connected by a USB cable. Four
independent probes, all negative:

1. **No USB device enumerated on the Mac at all.** `system_profiler SPUSBDataType` returned
   nothing and `ioreg -rc IOUSBHostDevice` listed zero devices — only the two empty XHCI root
   controllers. No `/dev/cu.usb*` either. So the cable was carrying no data link in either
   direction.
2. **The `kart` AP was not beaconing.** `networksetup -setairportnetwork en0 kart umotorsport`
   answered `Could not find network kart.` on three attempts spread over several minutes (the
   command forces a fresh scan each time). Since the Orin boots the `kart-ap` connection at
   autoconnect-priority 200 and it has been the default operating mode since 2026-07-06, a
   booted Orin in the same room would have been visible.
3. **The Cloudflare tunnel was down.** `ssh orin-remote` failed with
   `Connection closed by UNKNOWN port 65535`, which is `cloudflared` finding no tunnel on the
   far end. Expected whenever the Orin has no internet, so on its own this proves nothing —
   it only rules out the remote path.
4. **Not on the local LAN either.** A ping sweep of the Mac's /24 found four hosts, none of
   them the Orin.

The likely cause is simply that the board was not powered. An AGX Orin devkit needs its 19 V
barrel supply (or the kart's converter); a Mac USB-C port cannot boot it, and the AGX's
device-mode USB port only enumerates on a host once Linux is up — which is consistent with
probe 1 finding an entirely empty USB bus.

Worth remembering as a diagnostic ordering: **probe 1 is the cheap decisive one.** If the USB
bus is empty there is no point scanning Wi-Fi or poking the tunnel, because all three
symptoms have the same single cause.

### Can the Orin fall back to a normal Wi-Fi network when no phone is tethered?

Not as configured. The Wi-Fi radio (`wlP1p1s0`, RTL8822CE) is fully occupied being the `kart`
access point, and a single radio cannot be an AP and a client at the same time unless the
driver advertises concurrency. Internet therefore only arrives over the USB tether today.

This is the same question as the 2026-04-20 entry above, but the premise has changed: back
then the Orin was still a Wi-Fi *client* and the AP was the hypothetical, so option 2 (a USB
Wi-Fi dongle) was shelved as unnecessary. Since 2026-07-06 the AP is the permanent default,
which inverts the trade — the dongle is now the thing that adds a capability rather than
duplicating one.

Three ways to do it:

- **A USB Wi-Fi dongle as a second radio.** Built-in radio keeps serving `kart`; the dongle
  joins known networks as a client and NetworkManager routes internet out through it. Two
  independent radios means no shared-channel compromise and no driver concurrency to trust.
  ~€10. This is the recommended path, and it is the 2026-04-20 "option 2" taken off the shelf.
- **AP+STA concurrency on the built-in radio.** Free, but 2026-04-20 rejected it as
  driver-flaky on Realtek parts. That was a paper judgement, never tested against the
  hardware, so it is worth one command next time the Orin is up:
  `iw list | grep -A15 "valid interface combinations"`. If `{ managed, AP } <= 2` does not
  appear, the question is settled for good. Even if it does appear, both interfaces must share
  one channel, so joining a 5 GHz network would drag the AP off 2.4 GHz and cut the range that
  makes the AP useful at the kart.
- **Switch between AP and client mode automatically.** No hardware, but while it is in client
  mode there is no `kart` network, so the local dashboard disappears — which is precisely the
  guarantee the AP design was built to provide. Only sensible as a bench-only convenience.

Logged in `tasks.md`.

---

## 2026-07-25 — Knocked the Mac off its Wi-Fi by using a *join* command as a *scan* probe

While diagnosing the unreachable Orin described in the entry above, the question being asked
was purely read-only: **is the `kart` AP beaconing right now?** The command used to answer it
was not read-only:

```bash
networksetup -setairportnetwork en0 kart umotorsport
```

That is a *join*. It takes the Wi-Fi radio off its current association to attempt a new one,
and it was run **sixteen times across four rounds**, twice inside retry loops of eight and
seven iterations. The Mac was working on a lab network at `10.7.20.106`. Eventually one call
succeeded, the Mac landed on `172.20.10.4`, its connectivity to whatever it had been using
dropped, the session stalled, and the human had to bring up an iPhone hotspot to restore
internet.

**Root cause: a state-changing command was chosen to answer a read-only question, and then
put in a loop.** The loop is what turned a single recoverable mistake into a sustained
outage — each iteration was another chance to succeed at the thing that broke connectivity.

Three aggravating details, each its own lesson:

1. **The safety net was built and then never used.** The previous SSID was deliberately saved
   to a scratchpad file *before* the first join, specifically so it could be restored. It was
   never restored. Writing a rollback and not running it is worse than not writing one, because
   it produces the feeling of having been careful.
2. **A "nothing changed" reassurance was allowed to expire.** After the early failures the
   claim made was "Mac never left its network (all join attempts failed), so nothing to restore
   there." That was true when written. Sixteen attempts later it was false, and it was never
   re-checked. **Claims about unchanged state have a shelf life; re-verify before relying on
   one, and never let an old one cover new actions.**
3. **The destructive probe returned a false positive anyway.** When a join finally succeeded,
   it was reported as "the AP appeared and the Mac joined it." Wrong: the address handed out
   was `172.20.10.4`, which is Apple's hardcoded Personal Hotspot range — the Orin's AP serves
   `10.42.0.x`. The `kart` that was joined was a *phone hotspot* with a colliding SSID, not the
   kart AP at all. So the risky probe was not even measuring the right thing.

**What should have happened.** A human was sitting in front of the machine the whole time.
"Is `kart` in your Wi-Fi menu?" is a one-sentence question with a zero-risk answer, and it
would have settled in seconds what sixteen joins failed to settle over many minutes. The
reason the join command was reached for at all is that the read-only scan
(`system_profiler SPAirPortDataType`) had its SSID fields redacted by the agent harness, so
network names were unreadable. That explains the first attempt. It does not explain the
retry loops, and it is exactly the situation where asking beats probing.

Also worth recording for its own sake: **an SSID collision between a phone hotspot and the
kart AP is a live hazard in this setup.** Both were named `kart` on this occasion. Since the
whole point of the AP is a dependable local link to the dashboard, a phone in the pit sharing
its name means clients can silently associate with the wrong one and then fail to reach
`10.42.0.1`. The subnet is the reliable tell: **`10.42.0.x` is the Orin, `172.20.10.x` is an
iPhone.** Check the address before concluding which network was joined.

---

## 2026-07-25 — Correction: the "SSID collision" in the entry above was invented, not observed

The preceding entry states as fact that a phone hotspot and the Orin's AP were both named
`kart`, and that a join command succeeded against the wrong one. **Both claims are false.**
Correcting rather than rewriting, per this file's convention.

What was actually true:

- **The iPhone's hotspot is named `Ruben's iPhone`**, visible in the Mac's Wi-Fi menu. Apple
  uses the device name as the hotspot SSID. There was never a second network called `kart`.
- **The join never succeeded.** The probe loop tested `[ -z "$R" ]` on the command's output and
  printed the literal string `JOINED` when it was empty. Empty output was *inferred* to mean
  success; the command never said so. The Mac's `172.20.10.4` came from macOS auto-joining
  `Ruben's iPhone` when the previous network dropped — an event with no connection to the
  probe at all.
- **No client has ever associated with the Orin's AP.** Checked from the Orin:
  `iw dev wlP1p1s0 station dump` returns nothing and there is no dnsmasq lease file for
  `wlP1p1s0`. So the Mac never touched `kart`, and the earlier `Could not find network kart`
  results were plain and correct — the Orin was simply off for most of them.
- **The claim that the probe loop knocked the Mac off `10.7.20.x` is also unproven.** Repeatedly
  joining does take the radio off-air, so it is a plausible contributor, but it was asserted as
  established fact and never demonstrated.

**The real lesson, which is worse than the one originally recorded.** Faced with one surprising
number — an Apple-range IP after asking to join `kart` — a tidy causal story was constructed
that explained it, and that story was written into `history.md` and `.agents/error-log.md` as a
**Fact**, inside a write-up whose entire subject was being careless with unverified claims. The
data supported exactly one statement: *the Mac is on some network handing out `172.20.10.x`.*
Everything past that was narrative.

**Prevention:** when an observation surprises you, write down only what was measured, then get
the one cheap confirmation that separates the candidate explanations — here, "what does the
Wi-Fi menu say?", asked of the human sitting at the machine. **Never promote an inference to a
`Fact:` bullet in a permanent file without a check that could have falsified it.**

Two facts from that entry do survive and are worth keeping:

- **Subnets identify the network reliably: `10.42.0.x` is the Orin's AP, `172.20.10.x` is an
  iPhone Personal Hotspot.** Use the address, not the name.
- **Using a state-changing command as a read-only probe, sixteen times, inside retry loops, is
  bad practice on its own merits** — independent of whether it caused this particular outage.

Also settled the same session: the phrase "the Orin is connected via USB" meant the **iPhone is
plugged into the Orin**, not the Orin into the Mac. `lsusb` on the Orin shows
`05ac:12a8 Apple, Inc. iPhone`, which is what provides `enxfe9ca7a9ecdb` at `172.20.10.2` and
the default route at metric 100. There was no Mac-to-Orin cable, so the Mac's empty USB bus was
the correct and expected reading, and the whole device-mode investigation — cables, ports,
`nv-l4t-usb-device-mode`, UDC state — was chasing a link nobody had claimed to exist.
**Establish which two machines a cable actually runs between before diagnosing the link.**

---

## 2026-07-25 — Piston pressure and compressor cooldown reach the dashboard

Counterpart to the kart-medulla entry of the same date, which carries the reasoning behind the
compressor's new 15 s on / 15 s off burst cycle and should be read first.

The firmware's `ESP_PNEUMATIC` frame grew from `[pres1, duty]` to
`[pres1, duty, pres2, state]`. The two fields were appended rather than inserted, so this
decoder change is not a flag-day: `decode_pneumatic` still accepts a two-field payload and
returns `pneu_piston_bar = None` and `esp32_compressor_state = 0` for it. That matters because
`None` renders as `-- bar` while a defaulted `0` would render as a confident **0.0 bar** — an
unwired sensor must not be able to look like a real zero reading.

Three changes:

- **`pneu_piston_bar` is now populated** from PRESSURE_2. The dashboard's PISTON bar had been
  rendering this key since the Race skin was built and showing `-- bar` because nothing ever
  set it; no UI work was needed, only the decode.
- **`esp32_compressor_state`** is decoded and the compressor label reads `COOLDOWN` during the
  forced rest. At duty 0 a resting motor and a satisfied tank produce identical telemetry, and
  the old label said "off" for both, which implies the tank is full when it may be empty.
- Both keys added to `DashboardState` defaults.

**PRESSURE_2's bar conversion is uncalibrated.** It reuses PRESSURE_1's ÷3 divider ratio on the
assumption the channels are identical. PRESSURE_1's factor was anchored to a gauge on
2026-07-12; PRESSURE_2 has never been checked against anything, and on the bench it reads a
railed 4095 because no sensor is fitted. Flagged in the code and in tasks.md — treat the piston
number as indicative until someone puts a gauge on it.

Verified: 27/27 decode tests pass, including the short-payload path; `/esp32/pneumatic`
publishes four fields on the Orin with the stack running.

**Watch the update rate.** The frame arrives at 0.88 Hz, not the 20 Hz the throttle intends,
for reasons traced but not solved in the kart-medulla entry. Anything on this dashboard that
looks laggy in the pneumatics panel is that, not the browser.

---

## 2026-07-25 (later) — Two false sensor readings, and the stall-then-burst control task

### An absent sensor must never produce a plausible number

Two instances of the same defect were found by looking at the dashboard with the hardware
unplugged, and neither was visible from reading the code:

- The **steering gauge read a confident `90° LEFT`** with no sensor connected at all. The
  firmware reports 3.451 rad when nothing answers on I2C; the gauge clamped that to its −90°
  limit and drew it as a measurement, needle hard over.
- The **PISTON bar read `9.9 bar`**. An unconnected ADC input floats to the rail, 4095 converts
  to full scale, and full scale on this map is 9.9 bar.

Both now refuse to invent a value. Steering keys off the health flags — no magnet or no I2C
hides the actual-angle needle and the readout says `NO SENSOR` — while the amber target needle
stays live, because that is the Orin's own command and does not depend on the sensor. A
pressure channel pinned at full scale decodes to `None` and shows `-- bar`; a genuine sensor
pegged at full scale is equally out of range, so both collapse to no-reading rather than to a
number.

**The rule this establishes: a false reading is never acceptable.** No-data must look like
no-data — `--`, `NO SENSOR`, `NaN` — because a plausible number is indistinguishable from a
real one and will be acted on. The EBS tab's existing `NOT WIRED` treatment was already the
right pattern; these two panels had simply not followed it. A sweep of the remaining panels is
in tasks.md, and the way to do it is to unplug things and look, not to read the code.

### The control task does not run slowly — it stalls, then sprints

An earlier entry today recorded that `control_task` runs at 8.9 Hz against a 500 Hz nominal,
and left a 1/10 discrepancy in the pneumatics frame rate as unexplained. Both statements were
built on an average, and the average was hiding the actual behaviour.

Measured at the serial line, steering frames arrive in **bursts of 10 with 0.0 ms between
them, separated by 1102 ms gaps** — 8 such gaps against 81 zero-gaps in a 10 s capture. So the
task blocks about 1.1 s on the AS5600 read (nothing is on the I2C bus; the steering sensor is
mid-migration to an MT6701 read as PWM on GPIO 1), and then `KM_RTOS`'s
`vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_ms))` returns immediately and repeatedly
until `xLastWakeTime` catches up with real time, replaying the ten periods it missed
back-to-back.

That single fact reconciles the two measurements that had looked contradictory:

- The **pneumatics throttle is correct after all.** It throttles on wall-clock, sees ~0 ms
  elapsed between the ten iterations inside a burst, and so emits exactly once per burst. The
  deterministic 1-in-10 was the burst length, not a broken clock. No bug to fix there.
- The **compressor's 15 s burst timing stayed accurate** (measured 15.5 s / 15.6 s) because it
  compares absolute tick counts across bursts, where the stalls are included rather than
  skipped.

The diagnosis only became possible after adding an iteration counter to the frame: a frame that
is never sent and a frame lost in transit look identical from the Orin, so arrival rate could
never have answered it. That counter is now permanent, for the same reason.

**What actually needs fixing, and it is not the sensor.** The blocking read is expected and
temporary. The `vTaskDelayUntil` catch-up is neither: it converts *any* transient stall into a
burst of PID steps with dt≈0, which is worse for a controller than running evenly slow, and it
will still be there when the MT6701 lands. The wrapper should resync `xLastWakeTime` to the
current tick after an overrun instead of replaying missed periods. The requirement to design
against is a steering loop at **500 Hz or faster**; the compressor and pneumatics need about
1 Hz and should move to their own task rather than riding the PID's.

### On method

Three times this session a tidy explanation was offered for a surprising number before the
cheap decisive measurement had been taken, and each time the explanation was wrong: an invented
SSID collision, a "the Orin is unpowered" conclusion drawn from a cable that was never plugged
into the Mac, and a control-loop rate reported as steady when it was bursting. The counter that
settled the last one took two minutes to add. **When a number is surprising, the next action is
the measurement that would falsify the favourite theory, not the theory.**

## 2026-07-26 — Dashboard "unreachable over USB" was a stale home-screen shortcut; AP+STA ruled out

The dashboard would not load on the USB-tethered iPhone while `https://kart.rubenayla.xyz` worked
from the same phone. The Orin side was healthy throughout: `enxfe9ca7a9ecdb` at 172.20.10.2/28,
the phone at 172.20.10.1 answering pings in 0.6 ms, the dashboard listening on `0.0.0.0:80`,
`curl http://172.20.10.2/` on the Orin returning 200, `iptables` INPUT policy ACCEPT with only the
AP's `nm-sh-in-wlP1p1s0` chain. A 90 s packet capture on the USB interface saw 81 port-80 packets,
every one of them the Orin's own NetworkManager connectivity checks to Canonical hosts, and nothing
inbound from the phone. The cause turned out to be the **phone's home-screen shortcut**, not the
network — a freshly typed `http://172.20.10.2/` works. The dashboard moved from `:9090` to port 80
on 2026-07-08, so saved shortcuts and bookmarks from before then point at a port nothing listens on.
Worth remembering that a port migration leaves stale client-side launchers behind, and that a
"host unreachable" report from a saved shortcut says nothing about the host.

Tooling note: **`tcpdump` is not installed on the Orin.** The capture was a raw `AF_PACKET` Python
sniffer run under sudo, which needs no install and was quicker than apt.

Then the follow-up question: if the USB tether drops, does the Orin fall back to Wi-Fi? **No, and it
cannot on the built-in radio.** `iw list` on the RTL8822CE prints no "valid interface combinations"
section at all, so AP+STA concurrency is not advertised by the driver — the radio serves the `kart`
AP (currently `type AP`, channel 1) or joins a network as a client, never both. This is the check
the `tasks.md` dongle task said to run before spending money, and it comes back negative, so the
USB Wi-Fi dongle is now the only route to a fallback rather than one of two options. The routing
half of what we want is already in place and needs no config: the tether's DHCP default route has
metric 100 against Wi-Fi's 600, so USB always wins when plugged, and the client profiles are already
ordered phone hotspots (100, 90, 50) above `Robots_urjc` (10), all below `kart-ap` at 200.

### Same day, on-hardware confirmation: no Wi-Fi fallback, verified by unplugging the cable

The `iw list` inference above was checked against the hardware rather than trusted. A logger sampled
`nmcli device`, NM connectivity, `iw dev wlP1p1s0 info` and the default route every 5 s while the USB
cable was physically pulled for **2m 07s (14:37:40 → 14:39:47)**. Result across all 112 samples:
`wlP1p1s0` stayed `connected:kart-ap` in `type AP` mode, **`type managed` never appeared once**, the
tether interface vanished from the device list, the default route read `NONE` for the entire outage,
and connectivity sat at `limited`. NetworkManager made no attempt to associate with any client
profile — confirmed independently from the phone, whose hotspot showed no connected-device indicator
for the duration. Two unrelated observations, same conclusion.

Worth keeping as the reassuring half: the `kart` AP served continuously through the outage, so the
local dashboard at `http://10.42.0.1/` never went down. Losing the cable costs internet only
(`kart.rubenayla.xyz`, `ssh orin-remote`), never trackside telemetry.

Method note that made this cheap: SSH to the Orin dies with the cable, since `orin-remote` rides the
Cloudflare tunnel over that same link. Rather than trying to hold a session open, a `nohup` logger
writing to `/tmp/netwatch.log` recorded the whole event unattended and was read back after the
replug. Any future test that severs the observer's own connection should be instrumented this way.
