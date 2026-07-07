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
- Steering sign follows the **existing dashboard visual convention** (positive rad renders to the right, same as every other skin's steer bar). If that is ever found to mismatch the physical left/right, fix it across all skins at once, not per-skin.

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
