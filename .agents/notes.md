<!-- consult selectively — grep, never read in full -->
# Notes

## Steering planetary reducer — filament picks for nylon planet gears

Context: the failing part on the steering planetary is the **nylon sun gear** (D-flat on motor shaft rounds out under direction-reversal impacts — see history.md 2026-04-21). Plan is brass sun + printed nylon planets. Planet gears mesh against the brass sun and a nylon ring, so planet material needs low creep, good wear against brass, and low moisture swelling. See history.md 2026-04-21 entry for the full wear-distribution reasoning.

### Printer constraints
- Bambu Lab with **hardened nozzle** — confirmed 2026-04-21. CF filaments are fine.
- User has a filament drier — use it (nylons are hygroscopic; wet nylon prints as foam).
- Enclosed chamber needed for nylons (X1C / P1S fine; A1-series would warp).

### Filament ranking for planet gears (best → baseline)
| Tier | Filament | Approx €/kg | Notes |
|---|---|---|---|
| **Top pick** | Bambu Lab **PAHT-CF** | 75–90 | Tuned profile in Bambu Studio, RFID, "just press print." CF-reinforced PA, dramatically lower creep than plain nylon. |
| **Cost-effective** | Polymaker **PolyMide PA6-CF** | 55–70 | Same performance tier, generic CF-nylon profile, well-regarded. |
| Alt | Polymaker **PolyMide PA6-GF** or Fiberlogy **Nylon PA12 CF15** | 50–70 | GF is gentler on the nozzle than CF; performance close to CF-nylons for gears. |
| Budget | **eSUN ePA-CF** / **Sunlu PA-CF** | 30–50 | OK first attempt; expect more QC variation. |
| Baseline | plain **PA6 / PA12 nylon** | 25–40 | What's currently in use. Creeps under sustained torque — don't pick this if there's a filled alternative available. |
| **Avoid** | PLA, PETG, ABS | — | PLA + PETG creep badly under load; ABS wears too fast. Not for high-cycle gears. |

### Why not POM/Delrin (the "industrially correct" answer)
POM is the textbook plastic for meshing with brass (self-lubricating, near-zero creep, 0.2% moisture). But it is **effectively unprintable on FDM** — very low surface energy (won't stick to anything), heavy warp on cooling, poor layer adhesion. Bambu has no profile. **If you want POM, you machine it.** If we ever CNC the sun anyway, machining 3 POM planets on the same setup is the ideal long-term solution.

### Print settings (rules of thumb for planet gears)
- **Dry** filament 8 h at 70 °C before load; keep in drier during print.
- **Profile**: start with factory/default, don't tune from scratch.
- **Walls**: ≥5. **Infill**: 80–100% (parts are tiny; filament saving is pennies, solid wears better).
- **Orientation**: gear axis **vertical**, so tooth loads are in-plane, not peeling layers apart.
- Batch-print a few spares per session — consumables.

### Decision gate
Current status (2026-04-21): **not buying yet.** Current nylon planets are adequate; if they start failing after brass sun is installed, revisit and pick one of the filled-nylon options above. Top two candidates: **Bambu PAHT-CF** (zero-setup) or **Polymaker PA6-CF** (cheaper).

## Orin availability — it's a kart computer, not a server

The Orin lives **in the kart**. It is powered off whenever Ruben is not physically at the kart (e.g. working from home). `ssh orin-remote` returning `Connection closed by UNKNOWN port 65535` is usually **"the Orin is off"**, not "the Cloudflare Tunnel has a problem."

**Don't retry in a loop.** Before scheduling a retry or polling SSH:
- If the user is working from the Mac with no mention of being at the kart → Orin is almost certainly off. Do not deploy. Report the commit is on `origin/dev` and will deploy when they're next at the kart.
- If the user just said "I'm at the kart" / "I just booted Orin" / similar → then a single retry after a short delay is reasonable.

When deploy is blocked by Orin being off, **that's fine** — code + push is the "done" state for home sessions. Physical-world next step belongs to the human, not the agent.

## colcon `--symlink-install` gotcha (ament_python)

The flag is **misleadingly named**. It only creates symlinks for `ament_cmake` packages (and Python scripts installed via CMake, like `kart_control`). For **`ament_python` packages** — including `kb_dashboard` — it does NOT symlink. Files in `build/<pkg>/<pkg>/` are plain copies refreshed at build time. The egg-link just redirects `import <pkg>` from `install/` to `build/`, which still holds copies.

### Consequences
- Editing `src/kb_dashboard/kb_dashboard/index.html` (or `server.py`, `dashboard_node.py`) does **not** propagate until you rebuild the package. `git pull` + `systemctl restart kart-brain` alone is NOT enough. This has caused "I don't see my change" confusion multiple times.
- Editing `src/kart_control/scripts/*.py` DOES propagate without rebuild — `kart_control` is `ament_cmake` and its scripts are real symlinks.

### Fast rebuild loop for dashboard edits on Orin
```bash
cd ~/kart-brain && colcon build --symlink-install --packages-select kb_dashboard && sudo systemctl restart kart-brain
```
Incremental build is ~3 s; service restart ~4 s. Use `--packages-select` to avoid rebuilding everything.

### How to tell if a ROS 2 install is a copy or a symlink
```bash
ls -la install/<pkg>/lib/<pkg>/       # for ament_cmake scripts: symlinks (-> src/...)
ls -la build/<pkg>/<pkg>/             # for ament_python: always plain copies
cat install/<pkg>/lib/python3.10/site-packages/*.egg-link  # where imports resolve
```

### If you see stale UI after a `git pull`
The most likely cause is this gotcha. Rebuild the affected package, then restart the service. Do NOT assume the browser is caching — hard-refresh is cheap but won't fix a stale served file.

## Autonomous Agent Orchestrator (idea — 2026-03-14)

Run a headless Claude Code loop that picks tasks from `tasks.md` (repo root) autonomously:

```bash
# orchestrator.sh — run in tmux on Mac
while true; do
  claude -p "
    Read tasks.md (repo root). Pick the first Ready task.
    Move it to In Progress. Do the work. Move to Done or Blocked.
    Commit if you made code changes.
  " --allowedTools Edit,Read,Write,Bash,Grep,Glob
  sleep 300
done
```

Could also explore: GitHub Agentic Workflows, `/loop` skill, or a custom MCP-based orchestrator (e.g. AGINEAR, Flux). Key question: how to handle tasks that need human feedback (Blocked state) without stalling the loop.

## ESP32-S3 module to buy — ESP32-S3-WROOM-1-N8R2 (R8 BANNED)

Exact part number for the next-revision Kart Medulla PCB: **ESP32-S3-WROOM-1-N8R2** (8 MB flash, 2 MB quad PSRAM).

**Octal-PSRAM (R8) variants are BANNED, not "fallback."** On R8 modules the 8 MB octal PSRAM is hard-wired inside the module package to GPIO 33–37. Espressif's datasheet marks those pins as "not available" on R8 variants. This is a physical constraint: disabling PSRAM in firmware does NOT reclaim the pins. Treating an R8 board as if it were an R2 board is a hardware error.

**GPIO 33–37 are NOT reserved by our pinout.** We try to leave them free where convenient, but that is a courtesy, not a commitment, and it is not the standard we follow. Therefore the module must be a non-R8 variant — always.

**Valid upgrade path if flash fills up: ESP32-S3-WROOM-1-N16R2** (16 MB flash, 2 MB quad PSRAM — zero GPIO cost, zero pinout change). **Never N16R8.**

Decision recorded in `kart-docs/history.md` (2026-04-23), `kart-docs/docs/assembly/electronics/bom.yaml`, and `kart-docs/docs/assembly/electronics/kart-medulla/index.md`.

## ESP32 ↔ Orin Serial Protocol

### Frame format
```
| 0xAA | LEN | TYPE | PAYLOAD | CRC8 |
```
- SOF: 0xAA
- LEN: payload length (1 byte)
- TYPE: message type (1 byte)
- PAYLOAD: protobuf-encoded or raw bytes
- CRC8: polynomial 0x07 over LEN + TYPE + PAYLOAD

### Sending a protobuf message (standard pattern)
```c
// 1. Encode protobuf into buffer
uint8_t pb_buf[64];
kart_ActSteering msg = kart_ActSteering_init_zero;
msg.angle_rad = 0.5f;
msg.raw_encoder = 1234;

pb_ostream_t stream = pb_ostream_from_buffer(pb_buf, sizeof(pb_buf));
pb_encode(&stream, kart_ActSteering_fields, &msg);

// 2. Send with framing + UART
KM_COMS_SendMsg(ESP_ACT_STEERING, pb_buf, stream.bytes_written);
```

`pb_encode` serializes the struct into bytes, `KM_COMS_SendMsg` wraps in frame and calls `uart_write_bytes`.

### Message list

**ESP32 → Orin:**
- 0x01 ACT_SPEED — speed_mps (pb)
- 0x03 ACT_BRAKING — effort (pb)
- 0x04 ACT_STEERING — angle_rad, raw_encoder (pb)
- 0x05 MISION — 1 byte enum
- 0x06 MACHINE_STATE — 1 byte enum
- 0x07 ACT_SHUTDOWN — 1 byte
- 0x08 HEARTBEAT — uptime_ms (pb)
- 0x0B HEALTH_STATUS — magnet_ok, i2c_ok, heap_ok, agc, heap_kb, i2c_errors, stacks (pb)

**Orin → ESP32:**
- 0x20 TARG_THROTTLE — effort (pb)
- 0x21 TARG_BRAKING — effort (pb)
- 0x22 TARG_STEERING — angle_rad (pb)
- 0x23 MISION — 1 byte enum
- 0x24 MACHINE_STATE — 1 byte enum
- 0x25 HEARTBEAT — empty
- 0x26 SHUTDOWN — 1 byte
- 0x27 COMPLETE — throttle, braking, steering_rad, mission, machine_state, shutdown (pb)
