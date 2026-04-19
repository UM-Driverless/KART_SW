<!-- consult selectively — grep, never read in full -->
# Notes

## colcon `--symlink-install` gotcha (ament_python)

The flag is **misleadingly named**. It only creates symlinks for `ament_cmake` packages (and Python scripts installed via CMake, like `kart_control`). For **`ament_python` packages** — including `kb_dashboard` — it does NOT symlink. Files in `build/<pkg>/<pkg>/` are plain copies refreshed at build time. The egg-link just redirects `import <pkg>` from `install/` to `build/`, which still holds copies.

### Consequences
- Editing `src/kb_dashboard/kb_dashboard/index.html` (or `server.py`, `dashboard_node.py`) does **not** propagate until you rebuild the package. `git pull` + `systemctl restart kart-brain` alone is NOT enough. This has caused "I don't see my change" confusion multiple times.
- Editing `src/kart_control/scripts/*.py` DOES propagate without rebuild — `kart_control` is `ament_cmake` and its scripts are real symlinks.

### Fast rebuild loop for dashboard edits on Orin
```bash
cd ~/kart_brain && colcon build --symlink-install --packages-select kb_dashboard && sudo systemctl restart kart-brain
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

Run a headless Claude Code loop that picks tasks from `.agents/tasks.md` autonomously:

```bash
# orchestrator.sh — run in tmux on Mac
while true; do
  claude -p "
    Read .agents/tasks.md. Pick the first Ready task.
    Move it to In Progress. Do the work. Move to Done or Blocked.
    Commit if you made code changes.
  " --allowedTools Edit,Read,Write,Bash,Grep,Glob
  sleep 300
done
```

Could also explore: GitHub Agentic Workflows, `/loop` skill, or a custom MCP-based orchestrator (e.g. AGINEAR, Flux). Key question: how to handle tasks that need human feedback (Blocked state) without stalling the loop.

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
