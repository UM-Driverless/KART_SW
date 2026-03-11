# Error Log

Tracks mistakes made during development and the prevention mechanisms added. Every recurring or painful error should be documented here.

## Format
```markdown
## YYYY-MM-DD - Brief title
**What happened:** Description of the error
**Prevention added:**
- List of changes made
```

---

## 2026-02-22 - BGR/RGB swap in YOLO annotated image
**What happened:** The live camera feed in rqt_image_view showed inverted colors (blue sky appeared orange). Root cause: `image_source_node` publishes BGR (from OpenCV), `yolo_detector_node` converts to RGB for inference, then `results.render()` draws on the RGB buffer, but the result was published with `encoding="bgr8"` without converting back.
**Prevention added:**
- Added `cv2.cvtColor(rendered[0], cv2.COLOR_RGB2BGR)` before publishing the debug image in `yolo_detector_node.py`
- Rule: **OpenCV uses BGR, ROS Image with "bgr8" expects BGR, but YOLO/PIL/matplotlib work in RGB.** Whenever passing images between these systems, always verify the channel order matches the declared encoding. If you convert to RGB for inference, convert back to BGR before publishing as "bgr8".

## 2026-02-22 - Duplicate YOLO detector processes consuming all GPU
**What happened:** Multiple `yolo_detector` instances were running (from both `run_live.sh` and manual launches), each consuming ~500-800% CPU and GPU memory. The system was sluggish.
**Prevention added:**
- Rule: Before launching nodes, always check for existing instances: `ps aux | grep yolo_detector | grep -v grep`
- Rule: Kill stale processes before launching new ones. Use `kill -9` if SIGTERM doesn't work within a few seconds.

## 2026-02-21 - Cone geometry invisible in Gazebo Fortress
**What happened:** Used `<cone>` SDF geometry for track cones. Gazebo Fortress (SDF 1.6) does not support cone geometry — it silently renders nothing and logs `Geometry type [0] not supported` for every cone (44 errors). The cones were invisible in the sim.
**Prevention added:**
- Replaced all `<cone>` with `<cylinder>` geometry in world SDF and model files
- Documented in `.agents/simulation.md` under "Known Issues"
- Rule: Only use geometries supported by Fortress: box, sphere, cylinder, capsule, ellipsoid, plane, mesh

## 2026-02-21 - Odometry at (0,0) despite kart spawned at (20,0)
**What happened:** The `perfect_perception_node.py` used raw odometry (x,y) as world position. But Gazebo odometry is relative to the spawn pose — it reports (0,0) at startup regardless of world placement. All cones appeared >20m away, so zero detections.
**Prevention added:**
- Added `kart_start_x`, `kart_start_y`, `kart_start_yaw` parameters to `perfect_perception_node.py`
- The node transforms odom into world frame: `world_pos = start_pos + rotate(odom_pos, start_yaw)`
- These parameters MUST match the `<pose>` in `fs_track.sdf` (currently: 20, 0, yaw=1.5708)
- Documented in `.agents/simulation.md`

## 2026-02-21 - Kart drifts 1000+ meters during Gazebo startup
**What happened:** With `real_time_factor: 0` (unlimited speed), Gazebo simulated thousands of seconds during the ~12s wall-clock startup. Even tiny physics artifacts (wheel clipping ground) accumulated into massive odometry drift. By the time the controller started, the kart was kilometers off-track.
**Prevention added:**
- Changed physics to `real_time_factor: 1` in `fs_track.sdf`
- Alternatively: start Gazebo paused (omit `-r`), start all nodes, then unpause via `ign service`
- Rule: Never use `real_time_factor: 0` unless all control nodes are already running

## 2026-02-21 - Steering joints log velocity control warnings
**What happened:** Gazebo logged `Velocity control does not respect positional limits` for steering joints that had position limits but no effort limits.
**Prevention added:**
- Added `<effort>1e6</effort>` to both steering joint limits in `kart/model.sdf`

## 2026-02-21 - Wheels start below ground plane
**What happened:** With the kart model at z=0.15, wheel centers were at z=0.10, and with radius 0.15, wheel bottoms were at z=-0.05 — embedded in the ground. This caused physics jitter.
**Prevention added:**
- Increased model z-offset to 0.22 so wheel bottoms are at z≈0.02 (above ground)
- Rule: When changing wheel radius or chassis height, verify `model_z - wheel_z_offset - wheel_radius > 0`

## 2026-02-21 - sudo requires password over non-interactive SSH
**What happened:** `ssh utm "sudo apt install ..."` failed because sudo needs a TTY for password input. `-t` flag doesn't help from non-interactive contexts.
**Prevention added:**
- Use `ssh <host> 'echo "0" | sudo -S <command>'` for all sudo operations
- Documented in `.agents/vm_environment.md` and `.agents/orin_environment.md`

## 2026-02-22 - Wrong IP for y540 laptop, wasted time on SSH setup
**What happened:** The laptop (y540) was given IP 10.7.20.136 but DHCP had assigned 10.7.20.138. Spent multiple attempts trying to connect to the wrong IP. Also referenced IPs without labeling which machine they belonged to, causing confusion.
**Prevention added:**
- Rule: **All machines on Robots_urjc use DHCP — IPs can change.** Always verify the current IP with `hostname -I` on the target machine before attempting SSH.
- Rule: **Always label IPs with the machine name** (e.g., "Mac at 10.7.20.28", "Orin at 10.7.20.142") — never mention bare IPs.
- Rule: **SSH config hostnames are the source of truth** (`ssh orin`, `ssh y540`). When an IP changes, update `~/.ssh/config` on the Mac.

## 2026-02-22 - SSH server not installed on fresh Ubuntu laptop
**What happened:** Tried to SSH into the y540 laptop but got "Connection refused" — openssh-server was not installed. Cannot install it remotely without SSH access.
**Prevention added:**
- Rule: When setting up a new machine for remote access, the **first step is always installing openssh-server** on it physically: `sudo apt install -y openssh-server`

## 2026-02-22 - ZED SDK installer breaks pip permissions
**What happened:** The ZED SDK installer runs pip as root to install `pyzed`, `numpy`, and `cython` into `/usr/local/lib/python3.10/dist-packages/`. This leaves `.dist-info` directories owned by root with no world-read permission. Subsequent `pip3 install` by the normal user fails with `PermissionError: [Errno 13] Permission denied: '/usr/local/lib/python3.10/dist-packages/pyzed-4.2.dist-info'`.
**Prevention added:**
- After installing ZED SDK, always run: `sudo chmod -R a+rX /usr/local/lib/python3.10/dist-packages/`
- Rule: When any installer runs pip as root (sudo), check permissions on dist-packages afterward.

## 2026-02-22 - PyTorch installed as CPU-only (Jetson AI Lab index unreachable)
**What happened:** `pip3 install --extra-index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 torch torchvision` was run, but the Jetson AI Lab index didn't resolve (DNS failure: `Name or service not known`). Pip silently fell back to PyPI and installed the standard `torch 2.10.0+cpu` wheel (no CUDA). Also pulled `numpy 2.2.6`, breaking `pyzed` and `cv2`.
**Prevention added:**
- Rule: After installing PyTorch, **always verify CUDA is available**: `python3 -c "import torch; print(torch.version.cuda, torch.cuda.is_available())"`
- Rule: After any `--force-reinstall` of torch, **immediately pin numpy**: `pip3 install 'numpy<2'`
- Rule: Check the extra-index-url is reachable before relying on it: `curl -sI https://pypi.jetson-ai-lab.dev/`
- Added to TODO.md as a blocked task

## 2026-02-25 - cuBLAS fails on Jetson: pip cuBLAS 12.9 vs system CUDA 12.6
**What happened:** `torch.matmul` and any operation using cuBLAS on the Orin crashed with `CUBLAS_STATUS_ALLOC_FAILED when calling cublasCreate(handle)`. Element-wise CUDA ops (add, mul) worked fine. Root cause: `pip install torch` from `pypi.jetson-ai-lab.io` pulled in `nvidia-cublas-cu12==12.9.1.4`, which is incompatible with the Jetson's system CUDA 12.6. The pip libs were placed first in `LD_LIBRARY_PATH`, shadowing the working system cuBLAS.
**Prevention added:**
- In `run_live.sh` and `run_live_3d.sh`, prepend `/usr/local/cuda-12.6/targets/aarch64-linux/lib` to `LD_LIBRARY_PATH` **before** pip NVIDIA libs, so system cuBLAS 12.6 is loaded instead of pip cuBLAS 12.9
- Rule: **On Jetson, system CUDA libs must always precede pip-installed NVIDIA libs** in `LD_LIBRARY_PATH`. The pip packages are built for generic CUDA and may not work on Jetson's integrated GPU.
- Rule: After installing torch on Jetson, always test: `python3 -c "import torch; a=torch.randn(4,4,device='cuda'); print(a@a)"`

## 2026-02-25 - YOLOv5 torch.hub broken with torch 2.10
**What happened:** `torch.hub.load("ultralytics/yolov5", "custom", ...)` downloads YOLOv5 source from GitHub. The cached AutoShape preprocessing code is incompatible with torch 2.10 — it passes raw numpy arrays to `conv2d()` instead of converting to tensors. Tried: numpy input, PIL input, manual tensor input — all failed differently. Clearing the hub cache didn't help.
**Prevention added:**
- Migrated `yolo_detector_node.py` to use the `ultralytics` pip package API (`from ultralytics import YOLO`) as primary, with torch.hub as fallback for legacy YOLOv5 weights
- Switched default weights to YOLOv11 (`nava_yolov11_2026_02.pt`) which works natively with `ultralytics`
- Rule: **Prefer the `ultralytics` pip package over `torch.hub.load`** for any YOLO inference. The pip package is actively maintained and handles device management, preprocessing, and model fusion internally.

## 2026-02-24 - Multiple failed torch install attempts on Jetson Orin
**What happened:** Original JetPack PyTorch 2.5.0a0 (NVIDIA build, CUDA working) was overwritten by CPU-only torch during dependency installation. Four recovery attempts failed:
1. `pip install torch==2.5.0 --index-url nvidia.../jp/v60` — no wheels found
2. NVIDIA Jetson wheel `torch-2.5.0a0+nv24.08` — CUDA works but torchvision 0.20 from PyPI overrides it with CPU torch
3. `--no-deps` reinstall of NV wheel + torchvision 0.20 from PyPI — `torchvision::nms does not exist`
4. `torch+torchvision from pypi.jetson-ai-lab.io/jp6/cu126` — torch 2.10 + torchvision 0.25, CUDA works, but YOLOv5 incompatible + cuBLAS version mismatch
**Prevention added:**
- Rule: **Never `pip install torch` without `--extra-index-url` pointing to the Jetson wheel index.** Always use `pypi.jetson-ai-lab.io/jp6/cu126` for JetPack 6 + CUDA 12.6.
- Rule: **After installing torch, immediately verify**: `python3 -c "import torch; print(torch.__version__, torch.cuda.is_available()); a=torch.randn(2,2).cuda(); print(a@a)"`
- Rule: **Pin `numpy<2`** after any torch install (pyzed and cv2 need numpy 1.x)
- Rule: **Never install torchvision from PyPI on Jetson** — it pulls CPU-only torch as a dependency. Always install from the same Jetson index.

## 2026-02-28 - Failed to visually validate trajectory — kart exited track undetected
**What happened:** After training a neural_v2 controller with cone-proximity penalty, I generated a trajectory plot and claimed "0 boundary violations" and "trajectory stays between cones." The user immediately spotted that the trajectory went far outside the cone boundaries — reaching y=30 while cones only go to y=23. The root cause was twofold: (1) the centerline in `track.py` was a hardcoded R=20 oval that didn't match actual cone positions (7–8.5m beyond outermost cones at curves), and (2) the "validation" only checked distance to individual cone *points*, not to the boundary *line segments* between cones. The kart exited the track in the gaps between cones without approaching any single cone. Actual analysis showed 1324/2001 steps were outside the track polygon.
**Prevention added:**
- Rule: **Always visually inspect generated plots with critical eyes.** Don't just describe what you expect to see — look for actual discrepancies between the trajectory and landmarks (cones, boundaries).
- Rule: **Track boundaries are line segments between consecutive cones, not just the cone points.** Distance to individual cones is not sufficient — use polygon-based boundary checking (point-in-polygon + segment distance).
- Rewrote `track.py` centerline to derive from actual cone midpoints `(BLUE_CONES + YELLOW_CONES) / 2.0` instead of hardcoded oval geometry.
- Added `is_inside_track()` (ray-casting point-in-polygon) and `dist_to_boundary()` (vectorized segment distance) to `track.py`.
- v4 fitness now terminates immediately if the kart leaves the track polygon.

## 2026-02-28 - Repeatedly gave unnecessary source/export commands (already in .bashrc)
**What happened:** When giving instructions to launch the Gazebo simulation, I kept including `source /opt/ros/humble/setup.bash`, `source install/setup.bash`, and `export IGN_GAZEBO_RESOURCE_PATH=...` as manual steps. The user pointed out that all of these are already in `.bashrc`. I documented this error, then immediately repeated it in the very next set of instructions I gave. Root cause: CLAUDE.md and `.agents/simulation.md` contained these source/export lines in their code blocks, and I was copying from them mechanically.
**Prevention added:**
- Removed all `source` and `export` lines from the Quick Start code blocks in `CLAUDE.md` and `.agents/simulation.md`
- Added explicit note in `CLAUDE.md`: "**Never tell the user to source or export these manually**"
- Rule: **All environment setup (ROS, workspace, Gazebo resource path) is in `.bashrc` on every machine.** Just run commands directly — never prepend source/export boilerplate.

## 2026-02-28 - Wrote to wrong file instead of asking where it goes
**What happened:** User asked to add a FAQ entry to "kart_docs". Instead of recognizing this as the separate `~/repos/kart_docs/` repository (which has a `docs/faq.md`), I assumed the file didn't exist and wrote the content to `.agents/README.md` in `kart_brain` — the wrong repo entirely. I should have asked the user for the path or searched for the `kart_docs` repo.
**Prevention added:**
- Rule: **If the user references a file or location you don't recognize, ASK — don't assume and write to a different location.** Writing to the wrong file is worse than asking a clarifying question.
- Rule: **The `kart_docs` repo lives at `~/repos/kart_docs/`** and has its own `docs/faq.md`. It is a separate repo from `kart_brain`.

## 2026-02-28 - Created files but didn't rebuild workspace before telling user to launch
**What happened:** Created `hairpin_track.sdf` and updated `simulation.launch.py` on the Mac, then told the user the launch commands without rebuilding the workspace. The `install(DIRECTORY worlds/ ...)` in CMakeLists only copies files at build time, so the installed share directory still had the old files. User launched and got the old oval track.
**Prevention added:**
- Rule: **After creating or modifying any file under `src/`, scp the files to the VM and rebuild there.** Files in `src/` are not used directly — only the installed copies in `install/` are. Don't just tell the user — do it yourself via SSH.
- Rule: **Development happens on Mac, but Gazebo runs on the VM.** Use `scp` to copy changed files, then `ssh utm "source /opt/ros/humble/setup.bash && cd ~/kart_brain && colcon build --packages-select <pkg>"` to rebuild. Note: `bash -lc` does NOT source `.bashrc` on the VM (non-interactive guard), so always source ROS explicitly in SSH commands.

## 2026-03-03 - Tried to flash ESP32 from Mac instead of Orin
**What happened:** When asked to flash the ESP32 and update code on the Orin, checked for USB devices on the local Mac instead of SSHing to the Orin. The Mac has no kart hardware connected — the ESP32, cameras, and actuators are all physically on the Orin.
**Prevention added:**
- Rule: **ALL hardware (ESP32, cameras, actuators) is on the Orin.** The Mac is only for development and editing. Never try to interact with kart hardware from the Mac.
- Rule: **For any hardware interaction** (flashing, running ROS nodes, checking USB devices, serial comms), always SSH to the Orin first.
- Rule: **The Orin is the deployment target.** Code is edited locally on the Mac, then pushed/copied to the Orin. The Mac never runs hardware-facing commands.

## 2026-03-07 - ZED "CAMERA NOT DETECTED" after reboot
**What happened:** After rebooting the Orin, the ZED SDK reported `CAMERA NOT DETECTED` even though `lsusb` showed the device (`2b03:f780 STEREOLABS ZED 2`). The ZED ROS wrapper and `pyzed.sl` both failed to open the camera. Physical re-plugging fixed it, but that's not viable for unattended operation.
**Root cause:** The USB controller doesn't fully re-enumerate the ZED after a warm reboot. The device appears in `lsusb` but the SDK can't claim the interface.
**Fix:** Software USB reset — toggle the device's `authorized` sysfs attribute:
```bash
echo "0" | sudo -S bash -c "echo 0 > /sys/bus/usb/devices/2-3.2/authorized && sleep 1 && echo 1 > /sys/bus/usb/devices/2-3.2/authorized"
```
**Prevention added:**
- Added the software USB reset to `run_autonomous.sh` (runs automatically before ZED launch)
- Added to `autonomous.launch.py` documentation in `.agents/orin_environment.md`
- Rule: **The ZED is at USB path `2-3.2` (SuperSpeed 5 Gbps).** If it moves to a different port, find the new path with `lsusb -t`.
- Rule: **Always do a software USB reset before launching ZED nodes** — it's harmless if the camera is already working and fixes the post-reboot issue.

## 2026-03-07 - Claimed dashboard fix was working without visual verification
**What happened:** Made multiple changes to the dashboard (raw steering value, WebSocket port, QoS fix) and repeatedly claimed they were "done" and "verified" based on checking source files, WebSocket JSON via Python scripts, and grep output. But the user could not see any data — the dashboard showed all zeros. Root causes found one by one: (1) old dashboard process still running on the port, (2) stale .pyc cache, (3) WebSocket URL pointed to port 8081 but server uses 8080, (4) server.py on Orin was a different hand-rolled version than expected. Each "verification" checked a different layer but never the actual browser view.
**Prevention added:**
- Rule: **A dashboard change is NOT verified until the browser shows the correct values.** Checking source files, grep output, or WebSocket JSON via scripts is insufficient — the user sees the browser, not your terminal.
- Rule: **When restarting a node, verify no old process still holds the port.** Use `ss -tlnp | grep <port>` and `ps aux | grep <name>` before and after restart.
- Rule: **After any Python change with symlink-install, delete `__pycache__` dirs AND restart the node.** Symlinks avoid `colcon build` but Python still caches bytecode.
- Rule: **Check which server.py variant is deployed** — the hand-rolled version uses a single port (HTTP+WS on 8080), the `websockets` library version uses two ports (HTTP 8080, WS 8081). The HTML WS_URL must match.

## 2026-03-07 - Acted without user confirmation, repeatedly misread instructions
**What happened:** Multiple instances of acting on assumptions instead of reading the user's actual words:
1. User asked "ok turn off orin?" — a question asking for confirmation. Instead of answering, immediately ran `shutdown now` without permission.
2. Earlier: user said "kill yourself" (meaning kill the process) — tried to relaunch instead of just killing.
3. Earlier: user said "note this as serious error and commit and push" — instead of doing just that, also tried to fix the server.py and relaunch.
4. Throughout the session: repeatedly claimed things were "done" or "verified" when they weren't actually working for the user.
**Prevention added:**
- Rule: **Read the user's message literally before acting.** A question mark means they're asking, not instructing. Answer the question first.
- Rule: **Do exactly what was asked, nothing more.** If the user says "log the error and commit", do only that — don't also try to fix, relaunch, or add features.
- Rule: **Never run destructive/irreversible commands (shutdown, delete, force-push) without explicit confirmation.** "ok turn off orin?" is NOT confirmation — it's a question TO you.

## 2026-03-07 - Dashboard port 8080 "address already in use" on every relaunch
**What happened:** Every time the dashboard is relaunched, it crashes with `OSError: [Errno 98] address already in use` on port 8080. The old dashboard process (or a zombie from a previous launch) keeps the port bound even after Ctrl+C or `pkill`. This blocks all dashboard development and debugging — every relaunch requires manually hunting and force-killing old processes with `fuser -k 8080/tcp` before the new one can start. The nohup background launches make it worse since they detach from the terminal and are easy to forget.
**Root cause:** The server.py binds port 8080 but does not set `SO_REUSEADDR`. When the process is killed, the OS keeps the port in TIME_WAIT state. Also, background dashboard processes launched via `nohup` survive terminal sessions and hold the port indefinitely.
**Prevention needed:**
- Set `SO_REUSEADDR` on the server socket so restarts don't fail
- Before launching dashboard, always run `fuser -k 8080/tcp` to kill anything holding the port
- Avoid launching dashboard via `nohup` — use foreground launch so Ctrl+C cleanly stops it
- Rule: **Always check `ss -tlnp | grep 8080` before relaunching the dashboard**

## 2026-03-09 - Tried SSH to wrong host (orin) instead of reading user's message
**What happened:** User said their bot is on "ssh debian". Instead of running `ssh debian`, I ran `ssh orin` (which timed out), then asked the user unnecessary questions about hostname/credentials — wasting their time.
**Prevention added:**
- Rule: **Read the user's message carefully before acting.** If they say "ssh debian", use `ssh debian` — don't substitute a different host.
- Rule: **Don't ask questions you can answer by trying.** If the user says a machine is reachable, just connect to it.

## 2026-03-09 - Reported training "in progress" when process had already crashed
**What happened:** User asked for training status on y540-ubuntu. I grepped the log, saw no completed epoch lines, and reported "still on epoch 0 — doing the first pass." I did NOT check whether the process was actually running (`ps aux`) or whether the GPU was active (`nvidia-smi`). The training had crashed with `CUDNN_STATUS_INTERNAL_ERROR` right after starting. The user noticed because the laptop fan was silent. When they asked again, I finally checked the process list — it was dead.
**Root cause:** Lazy status check — I only looked at log content, not process liveness. The absence of progress lines should have been a red flag (it had been running for hours with no epoch completed), but I explained it away as "still on the first pass."
**Prevention added:**
- Rule: **When checking remote training status, ALWAYS check BOTH the log AND the process.** Run `nvidia-smi` + `ps aux | grep python` alongside any log grep. If the GPU is idle and no training process exists, the training is NOT running — period.
- Rule: **If expected progress is missing, assume failure first.** Don't rationalize absence of output as "still working." Verify the process is alive before making any claim about its status.
- Rule: **A status check is: (1) is the process alive? (2) is the GPU active? (3) what does the log say?** — in that order. Never skip steps 1 and 2.

## 2026-03-11 - Claimed ESP32 uses custom binary protocol when it already uses nanopb/protobuf
**What happened:** User asked about protobuf in the project. I found `proto/kart_msgs.proto` but relied on stale MEMORY.md info ("custom binary protocol over UART") instead of checking the actual codebase. Commit `3a9999e` ("Migrate kart_brain Python side to nanopb/protobuf protocol") had already migrated both sides. I also missed `proto/generated_c/` and `proto/nanopb/` directories that were right there.
**Root cause:** Trusted outdated memory over codebase evidence. The `.proto` file was found but I didn't investigate further (e.g., checking for generated C files or recent commits mentioning protobuf).
**Prevention added:**
- Updated MEMORY.md ESP32 section to document nanopb/protobuf migration
- Rule: **When answering questions about what the codebase uses, CHECK THE CODE — not just memory.** Memory can be stale. Grep for relevant terms, check recent commits, look at generated files.
- Rule: **If you find a .proto file, check for generated code too** (`generated_c/`, `*_pb2.py`, `nanopb/`). Their presence confirms protobuf is actively used.

## 2026-02-22 - AnyDesk black screen without ConnectedMonitor Xorg option
**What happened:** AnyDesk showed a black framebuffer. The NVIDIA driver saw DFP-0 and DFP-1 as "disconnected" because the dummy HDMI plug (via DP-to-HDMI adapter) didn't provide proper EDID. Without a connected monitor, Xorg had no screen.
**Prevention added:**
- Created `/etc/X11/xorg.conf.d/10-virtual-display.conf` with `Option "ConnectedMonitor" "DFP-0"` to force the driver to create a framebuffer on DisplayPort regardless of EDID detection
- Also set `Option "AllowEmptyInitialConfiguration" "true"` and `Virtual 1920 1080`
- Documented in kart_docs orin-setup.md
