# Battery state-of-charge: make the dashboard's BATT gauge trustworthy

Status: partly built and deployed, not yet switched on. Two pieces of work remain, and
they are independent — the first makes the gauge useful today, the second makes it accurate.

Everything below is self-contained. Read it and you have the whole problem.

---

## 1. The problem

The kart's dashboard shows a BATT dial whose percentage comes straight from the pack's
JBD/Xiaoxiang smart BMS. That number is wrong by a large margin, and the error is not noise
— it is systematic and always in the optimistic direction.

Measured on 2026-08-10:

| What the dial said | Pack voltage | Per-cell | Charge that voltage implies |
|---|---|---|---|
| 93% | 46.40 V at rest | 3.57 V | roughly 25% |
| 98% | 47.89 V at +2.4 A | 3.69 V | roughly 40% |

The pack is a **13S4P Molicel P42A** (13 cells in series, 4 in parallel; 48 V nominal,
16.8 Ah nameplate, about 41.6 V empty to 54.6 V full). Cell voltages at the second reading
were 3.682–3.690 V across all 13 channels — an 8 mV spread, so the pack is well balanced and
no single bad cell explains anything.

**Why the BMS is wrong.** Its state-of-charge figure is an open-loop coulomb count. It
integrates current, and it only re-zeros when the pack reaches a full-charge termination or a
low-voltage cutoff. Between those two events nothing corrects it, so error accumulates. This
pack does not routinely see either event, so the count has drifted far from reality.

**Why this matters.** A driver deciding whether to start another session reads that dial. At
93% it says "plenty"; the pack was actually near a quarter. There is no other charge
indication on the kart.

---

## 2. What the BMS actually gives us

Parsed in `src/kb_bms/kb_bms/bms_node.py`, function `_parse_basic`. The relevant fields, with
their real resolution:

| Field | Encoding | Resolution | Notes |
|---|---|---|---|
| `voltage` | u16 / 100 | 0.01 V | whole pack |
| `current` | s16 / 100 | 0.01 A | **positive = charging**, negative = discharging |
| `remain_ah` | u16 / 100 | 0.01 Ah | remaining charge |
| `nominal_ah` | u16 / 100 | 0.01 Ah | reads 16.80 — the design figure |
| `soc` | single byte | **1%** | integer percent, the number on the dial |
| `cells` | u16 each | 1 mV | 13 per-cell voltages, separate command |

Two things to know before you design anything around these.

**The SOC byte is coarse.** One byte, whole percent. `remain_ah` is about 17× finer
(0.01 Ah on a ~16 Ah pack). Anything integrating charge should use `remain_ah`, not the byte.

**The SOC byte is `remain_ah` divided by a learned full-charge capacity, and that capacity is
not the 16.8 Ah design figure.** Confirmed on two independent readings taken months apart:

```
13.11 Ah remaining at 98%  ->  13.11 / 0.98 = 13.38 Ah full
13.26 Ah remaining at 99%  ->  13.26 / 0.99 = 13.39 Ah full
```

So the BMS believes the pack holds about **13.38 Ah**, not 16.8 Ah. Both figures come from the
same coulomb counter; neither is derived from voltage.

Whether 13.38 Ah is correct is genuinely open. It is 80% of nameplate. Against it: the pack
had only 6 cycles logged, and 20% degradation that early would be alarming. For it: the 4.2 Ah
per-cell rating is measured at low rate down to 2.5 V/cell, whereas usable capacity between
this BMS's real cutoffs, at kart currents (48 A pack is about 3C per cell), is legitimately
lower. **Do not assume either number is the truth — task B measures it.**

---

## 3. Why voltage alone is not the answer

The obvious fix is to throw the coulomb count away and read charge off the voltage. That does
not work on this pack, for two independent reasons.

**The curve is flat where you need it most.** Between roughly 3.6 and 3.9 V/cell the
open-circuit voltage barely moves with charge. Across most of the usable range a 30 mV/cell
error becomes a 20-point charge error.

**Load destroys the reading.** Pack resistance is about 65 mΩ (roughly 20 mΩ per cell, ×13
series, ÷4 parallel). The 2 kW motor draws around 48 A at full power, which sags the terminals
by about 3 V — wider than the entire flat region. A driver on the throttle would watch the
gauge collapse and recover with every corner.

Voltage is a reliable *voltage*. It is a weak charge sensor except at rest and near the ends of
the range.

---

## 4. What already exists

All committed on branch `dev` and deployed to the Orin.

### `src/kb_bms/kb_bms/soc_model.py` (commit `6c679b5`)

The charge model and a scalar Kalman filter that fuses the two sources:

- **Prediction** advances the estimate by the change in the BMS's own `remain_ah`. That change
  is trustworthy even though its absolute value is not — it is the BMS's internal high-rate
  integration. Note the deliberate decision *not* to integrate current in kart-brain: the BLE
  poll loop completes only about every 2.2 s, far too slow to integrate the spiky current a
  throttle produces without aliasing badly.
- **Update** corrects against charge read off the OCV curve, after compensating the terminal
  voltage for resistive sag.
- **The update's noise is scheduled** on the local slope of the OCV curve and on the current
  being drawn. This is the part that makes it work without any hand-tuned mode switch: on the
  flat plateau and under heavy load the implied noise is enormous, so voltage is effectively
  ignored and the estimate coasts on the coulomb count; at rest and near either end the slope
  steepens, noise collapses, and voltage pulls the estimate back onto the truth.
- **Boot seeding**: a resting pack seeds its estimate from voltage rather than inheriting the
  BMS's drifted figure. The kart is nearly always stationary at startup, so this alone catches
  most of the failure. A pack already under load falls back to the BMS figure with wide
  uncertainty and gets corrected at the next quiet moment.

### `src/kb_bms/kb_bms/bms_node.py`

Runs the filter once per BLE poll and publishes the result as `std_msgs/Float32` on
**`/battery/soc_fused`** (fractional, 0–1). It logs which source seeded the estimate at boot.

`/battery/state.percentage` still carries the **raw** BMS figure, unchanged, whatever the
fusion setting is — so the two can always be compared.

### `src/kb_bms/kb_bms/log_battery.py`

A node that records `/battery/state` to CSV. This is the instrument for task B. Its docstring
carries the extraction procedure.

### `src/kb_dashboard/` (`dashboard_node.py`, `index.html`)

Subscribes `/battery/soc_fused` and, in the Battery tab, prints `V says NN%` under the SOC ring
whenever the two figures differ by more than 10 points (`RC_SOC_DISAGREE_POINTS`,
`rcBatRingSub`). The disagreement is deliberately shown rather than averaged away.

### `src/kb_bms/test/test_soc_model.py`

14 tests, pure Python — no ROS, no Bluetooth — so they run on a laptop:

```bash
cd src/kb_bms && python3 -m pytest test/test_soc_model.py -q
```

They test behaviour, not pinned numbers (seeds from voltage at rest, resists a sagging pack
under load, follows the coulomb count across the plateau, stays in range), precisely so that
recalibrating the constants in task B does not break them.

### What is switched off

The ROS parameter **`soc_fusion` on the `kb_bms` node defaults to `false`.** While false, the
filter still runs and still publishes, but `/battery/state.percentage` — and therefore the
dial — keeps showing the raw BMS number.

It is off because `OCV_TABLE` and `CELL_RESISTANCE_OHM` in `soc_model.py` are **generic NMC
21700 figures, not measurements of this pack.**

---

## 5. Task A — make the gauge useful now

Goal: get the dial to within roughly ±10 points, today, without waiting for a full discharge.

A generic curve is already good enough to beat a 55-point error by a wide margin, and Molicel
publishes a real discharge curve for the P42A. Using the published curve rather than the
generic NMC placeholder is a small change with most of the benefit.

1. Find Molicel's published P42A discharge curve (their datasheet / product page). Take the
   lowest-rate curve available, since it is closest to open-circuit.
2. Replace `OCV_TABLE` in `src/kb_bms/kb_bms/soc_model.py` with points read off it. Keep the
   existing format: `(fractional_charge, cell_volts)`, ascending. Normalise charge so 0.0 is
   the discharge cutoff and 1.0 is full.
3. Replace `CELL_RESISTANCE_OHM` with the datasheet DC internal resistance if it is published
   (the current 0.020 Ω is a plausible placeholder).
4. Update the comments marking both as provisional — say which document they came from and
   that they are datasheet figures, not measurements of this pack.
5. Run the tests. They should still pass; if one fails, it is telling you the new curve broke a
   behaviour, so read which one before adjusting anything.
6. Set `soc_fusion` to `true` so the dial's number becomes the fused estimate.
7. Deploy (section 7) and confirm on the dashboard.

**Sanity check before you believe it:** with the kart at rest, the fused figure should land near
what the cell voltages imply by eye. At 3.69 V/cell expect roughly 40%, not 98%.

## 6. Task B — measure the pack properly

Goal: replace datasheet figures with this pack's real behaviour, and settle the 13.38-vs-16.8 Ah
capacity question.

1. Run the logger across **one full discharge and one full recharge**:

   ```bash
   ros2 run kb_bms log_battery --ros-args -p path:=/tmp/battery-run.csv
   ```

   Discharge until the BMS cuts off, then charge to termination. Both endpoints matter — they
   are what anchor the ends of the curve, and reaching them also re-zeros the BMS's own counter.

2. **True usable capacity** is the total charge moved between those two endpoints. This is the
   number the filter should divide by, and it decides whether the BMS's 13.38 Ah or the 16.8 Ah
   nameplate is closer to the truth. Feed it to `SocFilter` in place of `design_capacity`
   (`bms_node._fuse_soc` currently passes `nominal_ah`).

3. **The OCV curve** comes from samples where the current is near zero, so the terminals are
   close to open circuit. Plot cell voltage against cumulative charge, normalise charge to
   0–1 across the two endpoints, and read off replacement `OCV_TABLE` points. Resting samples
   during the recharge are the cleanest.

4. **Internal resistance** comes from consecutive sample pairs where the current stepped sharply
   while charge barely moved: dV/dI across such a pair is almost entirely resistive. Divide by
   13 (series) and multiply by 4 (parallel) to get the per-cell figure for
   `CELL_RESISTANCE_OHM`.

   One caveat when reading the CSV: the BMS is polled at only about 0.45 Hz, so the current
   column is a sparse sample of a much faster signal. Individual current readings while driving
   are not reliable as instantaneous truth. The resting samples and the charge column carry the
   useful information.

5. Re-run the tests, deploy, and confirm the estimate now tracks sensibly across a whole session
   rather than only at the endpoints.

---

## 7. Deploying to the kart

The Orin lives in the kart and is powered down when nobody is there, so `ssh orin-remote`
failing with "Connection closed" simply means the kart is off — it is not a fault to debug.

```bash
ssh orin-remote
cd ~/kart-brain && git pull
colcon build --packages-select kb_bms kb_dashboard
echo 0 | sudo -S systemctl restart kart-brain
```

**`colcon build` is not optional for dashboard changes.** `kb_dashboard/index.html` ships via
`package_data`, and the server reads the installed copy under
`install/kb_dashboard/lib/python3.10/site-packages/kb_dashboard/`. A `git pull` and restart
alone will silently keep serving the old page.

Verify:

```bash
ros2 topic echo /battery/state --once      # raw BMS figure
ros2 topic echo /battery/soc_fused --once  # fused estimate
journalctl -u kart-brain -f | grep kb_bms
```

Dashboard: `http://10.42.0.1/` at the kart, or `kart.rubenayla.xyz` when the Orin has internet.

### Expect a gap in battery data after every restart

Restarting `kart-brain` currently costs about two minutes with no battery readings. This is a
known separate bug, tracked in the repo's root `tasks.md` — do not mistake it for something task
A or B broke. Briefly: `kb_bms` drops its Bluetooth link without disconnecting, which leaves
BlueZ (Linux's Bluetooth daemon) recording the pack as still connected. While it believes that,
the pack is unreachable both ways — connecting by address hits the stale entry, and scanning
cannot find it either, because a BLE peripheral that thinks it holds a connection stops
advertising. The node's own `_bluez_recover()` clears it, but only after three consecutive
failures.

Part of it is fixed (commit `bace59f`: the systemd unit was missing `exec`, so no node in the
stack had ever run its shutdown code — everything was SIGKILLed). The disconnect itself is
still not happening, and that part is open.

---

## 8. Ground truth for sanity checks

- Pack: 13S4P Molicel P42A, 48 V nominal, roughly 41.6 V empty to 54.6 V full.
- Nameplate capacity 16.8 Ah (4 × 4.2 Ah). BMS-learned capacity 13.38 Ah. Which is right is open.
- Current sign, JBD convention: **positive = charging**, negative = discharging.
- Health flags: this pack's BMS is read over BLE by `kb_bms`, entirely independently of the
  ESP32 serial link, so battery data survives an ESP32 outage.
- A resting pack at 3.57 V/cell is roughly a quarter charged; at 3.69 V/cell roughly 40%. If a
  change makes either read near 100%, it is wrong.
