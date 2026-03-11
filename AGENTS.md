# Agent Quick Reference

## Key Files (Read Before Working)
**These `.agents/` files are the source of truth for this project.** Always consult them before answering technical questions or making changes — they are version-controlled and stay in sync with the code. Never rely on auto-memory or prior conversation context for project-specific technical state (architecture, protocols, hardware, conventions), as memory can go stale after commits.

Before making any changes to the kart_brain workspace, consult:
1. **`.agents/architecture.md`** — System architecture, packages, topic map, message types
2. **`.agents/error_log.md`** — Past errors and added preventions
3. **Environment-specific guide:**
   - Working on **real hardware (Orin)?** → `.agents/orin_environment.md`
   - Working on **simulation (VM)?** → `.agents/simulation.md` + `.agents/vm_environment.md`

## Environments

### Jetson Orin (Real Hardware)
- **Connection:** `ssh orin` (WiFi 10.7.20.142) or AnyDesk
- **Workspace:** `~/kart_brain`
- **Camera:** ZED 2 stereo (USB)
- **sudo password:** `0`
- **Full details:** `.agents/orin_environment.md`

### UTM VM (Simulation)
- **Connection:** `ssh utm` (192.168.64.3, static IP)
- **Workspace:** `~/kart_brain/`
- **Simulator:** Gazebo Fortress (headless, CPU rendering)
- **sudo password:** `0`
- **Full details:** `.agents/vm_environment.md`

## Working Style
- **Use subagents for parallel work.** When tasks are independent (e.g., reading multiple files, searching codebase, running builds while editing), use the Agent tool to delegate to subagents. This keeps the main context clean and speeds up work.
- **Use background tasks for long-running ops.** Builds (~30s–100s), flashing, serial reads, SSH commands — run these in the background and check results later.
- **Don't block on things you can parallelize.** If you need to edit 3 files and deploy, edit them all, then deploy. If you need info from 2 different places, query both at once.

## Definition of Done
A change is NOT done until it's **validated on the target machine**:
- Code pushed? → **Pull on Orin/VM too.**
- ESP32 firmware? → **Flash it.**
- Python/launch change? → **Restart the affected nodes.**
- Never claim something is fixed if you only pushed — deploy and verify.

## Critical Rules
- **Environment is in `.bashrc`** — ROS, workspace, and `IGN_GAZEBO_RESOURCE_PATH` are all sourced in `.bashrc` on every machine. **Never tell the user to source or export these manually.**
- **After creating/modifying files under `src/`, scp them to the VM and rebuild via SSH — don't just tell the user.** Use: `scp <files> utm:~/kart_brain/...` then `ssh utm "source /opt/ros/humble/setup.bash && cd ~/kart_brain && colcon build --packages-select <pkg>"`. Note: `.bashrc` is NOT sourced in non-interactive SSH — always source ROS explicitly.
- **Gazebo Fortress uses `ign` CLI**, not `gz`. Message types are `ignition.msgs.*`, not `gz.msgs.*`.
- **No `<cone>` geometry** in SDF — use `<cylinder>` instead (Fortress limitation).
- **Odom is relative to spawn** — always account for the kart's initial world position.
- **No hardware GPU on the VM** — CPU rendering via llvmpipe (OpenGL 4.5). Gazebo GUI works on `DISPLAY=:0` but headless EGL fails. Keep camera resolution at 640x360.
- **Clean up after yourself** — delete temporary files, screenshots, debug artifacts, and tool-generated directories before finishing. Don't leave untracked trash in the repo.
- When something goes wrong, document it in `.agents/error_log.md`.

## Build & Run
```bash
# Build everything
cd ~/kart_brain && colcon build

# Build single package
colcon build --packages-select kart_perception

# Live perception on Orin
~/kart_brain/run_live.sh

# Simulation in VM
ros2 launch kart_sim simulation.launch.py
```

## Key Paths
| Path | Description |
|---|---|
| `src/kart_perception/` | Perception pipeline (YOLO + depth + viz) |
| `src/kart_sim/` | Gazebo simulation package |
| `src/kart_bringup/` | Launch files and config for real hardware |
| `src/joy_to_cmd_vel/` | Joystick teleop (C++) |
| `src/msgs_to_micro/` | ESP32 serial comms (C++) |
| `models/perception/yolo/best_adri.pt` | YOLO weights |

## Cone Class IDs
Used everywhere — YOLO class names, Detection messages, visualization:
- `blue_cone` — left track boundary
- `yellow_cone` — right track boundary
- `orange_cone` — start/finish markers
- `large_orange_cone` — large start/finish markers

## Documentation Rules
- **Document every decision.** When a version is chosen, a workaround is found, or an approach is selected over alternatives, write it down in the relevant `.agents/` file with the date and reasoning.
- **Document every error.** When something breaks or doesn't work as expected, add it to `.agents/error_log.md` with what happened and the prevention rule.
- **Document every version.** Software versions, SDK versions, wheel sources, compatibility notes — all go in `.agents/orin_environment.md` or the relevant environment file.
- **Official docs live in kart_docs.** The `.agents/` directory is for AI agent workflow. Official project documentation goes to https://github.com/UM-Driverless/kart_docs.

## Commit Protocol
1. `git status` — check what will be committed
2. `git diff --cached` — review changes
3. Build and verify before committing
4. If a mistake occurred, document it in `.agents/error_log.md`
5. If recurring, create a postmortem in `.agents/postmortems/`

## Documentation
The official documentation for the kart project lives in a separate repo:
- **Repo:** https://github.com/UM-Driverless/kart_docs
- **Site:** https://um-driverless.github.io/kart_docs/

The `.agents/` directory in this repo is for AI agent workflow only — not official project docs.
