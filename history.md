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
