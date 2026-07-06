<!-- reference — read when relevant -->
# Jetson Orin Environment

## Hardware
| Property | Value |
|---|---|
| Board | NVIDIA Jetson Orin (AGX) |
| Architecture | aarch64 (ARM64) |
| RAM | 62 GB |
| CPUs | 12 cores |
| GPU | Ampere (CUDA 12.6) |
| Storage | 476 GB NVMe SSD (root filesystem) + 57 GB eMMC (bootloader only) |
| Display | DisplayPort only (no HDMI). DP-to-HDMI adapter + dummy plug for AnyDesk |
| Camera | ZED 2 stereo (USB, SN 21983349) |
| ESP32 | ESP32-D0WD-V3 via USB (`/dev/ttyUSB0`), 115200 baud (CP2102 limit) |
| Steering sensor | AS5600 magnetic encoder (I2C: SDA=GPIO21, SCL=GPIO22) |
| Actuators | Steering H-bridge, throttle DAC, brake DAC |
| CAN bus | `can0`, `can1` interfaces (unused, was for old ESP32 comms) |

## Access
```bash
ssh orin-local    # LAN — direct WiFi IP (10.7.20.142). Only works on same network.
ssh orin-remote   # WAN — Cloudflare Tunnel (orin.rubenayla.xyz). Works from anywhere.
# Dashboard: https://kart.rubenayla.xyz (password: "0", configurable via ROS param `password`)
#   Cloudflare Tunnel config: /etc/cloudflared/config.yml (system-level, needs sudo)
#   Routes kart.rubenayla.xyz → localhost:9090 on Orin
# There is NO "ssh orin" alias. Always use orin-local or orin-remote.
# Try orin-local first (faster), fall back to orin-remote if unreachable.
# AnyDesk for GUI (needs dummy HDMI plug)
# sudo password: 0
```

## WiFi Networks (priority order)
| Priority | Connection Name | SSID | Password | Notes |
|---|---|---|---|---|
| 100 | Ruben's iPhone | Ruben\u2019s iPhone | 00000000 | Ruben's phone hotspot. SSID has curly apostrophe (U+2019) — use `nmcli dev wifi connect` or Python to pass correct UTF-8. |
| 50 | iPhone de JBA | iPhone de JBA | — | Jorge's phone hotspot. |
| 10 | Robots_urjc | Robots_urjc | — | Lab WiFi. |

## USB tethering (iPhone → Orin) — verified working 2026-07-05
Plugging Ruben's iPhone into the Orin over USB (with Personal Hotspot on) creates an ethernet interface `enxfe9ca7a9ecdb` (name derives from the phone's MAC, so it can change if the phone presents a different MAC). NetworkManager auto-activates it as `Wired connection 2`, DHCP on the hotspot subnet 172.20.10.0/28: the iPhone is the router at 172.20.10.1, the Orin gets 172.20.10.2. Its default route has **metric 100, which beats Wi-Fi's 600** — so when tethered, all internet traffic (including the Cloudflare tunnel / `orin-remote` SSH) goes over USB automatically, no config needed. Verified by sudo-disconnecting Wi-Fi entirely: ping, DNS, and curl all worked over USB alone, and the remote SSH session survived.

**The USB link is a two-way IP network, not just an internet pipe.** While the iPhone provides internet, it can simultaneously reach the Orin directly at its link IP — dashboard from Safari on the tethering phone: `http://172.20.10.2:9090`. Routing verified from the Orin side (Orin ↔ 172.20.10.1 pings over the USB interface); the Safari-on-phone check itself is still pending.

Gotcha: `nmcli device disconnect/connect` over SSH fails with "not authorized" (polkit) — needs sudo.

### Network architecture (decided 2026-07-05, IMPLEMENTED 2026-07-06)
Goal: dashboard must work even when the phone has no internet. Design:
- **Orin Wi-Fi (`wlP1p1s0`) becomes its own access point** (`nmcli device wifi hotspot ifname wlP1p1s0 ssid kart password <pwd>`, set autoconnect). Dashboard for everyone who joins: `http://10.42.0.1:9090` (NM hotspot default IP). Zero internet needed.
- **Ruben's iPhone: USB cable only, no Wi-Fi role.** Provides cellular internet to the Orin and browses the dashboard at `172.20.10.2:9090` over the same cable. (iPhones can't be Wi-Fi client + Wi-Fi hotspot at once — single radio — but that's irrelevant here since the phone doesn't need Wi-Fi.)
- Other phones join the kart AP; NM `shared` mode NATs the Orin's USB internet to them by default, so they even get internet when the tether is plugged in.
- Consequence: Orin no longer joins lab/phone Wi-Fi → `orin-local` means "join the kart AP, ssh 10.42.0.1". `orin-remote` works whenever the USB tether is up.

**Implemented 2026-07-06.** Verified: `iw list` shows AP mode ✓; dashboard binds `0.0.0.0:9090` ✓; hotspot up with dashboard answering on it ✓; internet + Cloudflare tunnel still flow over the USB tether ✓; `MASQUERADE 10.42.0.0/24` NAT rule active so AP clients get internet when the tether is plugged ✓. Human-verified 2026-07-06: devices see and join the `kart` network and get internet through the USB tether. **This is the default operating mode** — the Orin always boots as the kart AP.

Config that was applied (all persistent, survives reboot):
- NM connection **`kart-ap`**: SSID **`kart`**, WPA2 password **`umotorsport`**, `mode ap`, `ipv4.method shared` (NM runs dnsmasq + NAT automatically), `autoconnect yes`, `autoconnect-priority 200` (beats "Ruben's iPhone" at 100 and everything else — the Orin no longer joins lab/phone Wi-Fi on boot).
- Dashboard for anyone on the kart AP: **`http://10.42.0.1:9090`**. SSH: `ssh orin@10.42.0.1` — the Mac's `orin-local` alias now points there.
- Disable it if ever needed: `sudo nmcli connection down kart-ap` (or `... modify kart-ap connection.autoconnect no`); reconnect old Wi-Fi with `sudo nmcli connection up "Robots_urjc"` etc.

For non-interactive sudo:
```bash
ssh orin-local 'echo "0" | sudo -S <command>'
```

## Software
| Software | Version / Path |
|---|---|
| OS | Ubuntu 22.04 (L4T R36.5, JetPack 6.2.2) |
| ROS 2 | Humble (full desktop + vision_msgs + dev tools) |
| CUDA | 12.6 (via nvidia-jetpack) |
| cuDNN | 9.3 (via nvidia-jetpack) |
| TensorRT | 10.3 (via nvidia-jetpack) |
| PyTorch | 2.10.0 (CUDA works — libcudss.so.0 registered via ldconfig, see Environment Setup) |
| Gazebo | Fortress 6.16.0 (`ros-humble-ros-gz`) |
| ZED SDK | 4.2 (`/usr/local/zed/`, L4T 36.4 build, compatible with L4T 36.5) |
| Python | 3.10.12 (system) |
| numpy | 1.26.4 (must be <2, cv2 compiled against numpy 1.x) |
| ultralytics | 8.4.14 |
| PlatformIO | 6.1.19 (`/home/orin/.local/bin/pio`) |
| AnyDesk | Installed |

## Environment Setup

The following are already in `~/.bashrc` and sourced automatically on login/terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/kart-brain/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix kart_sim 2>/dev/null)/share/kart_sim/models
```

**Not in `.bashrc`** — must be set manually when running PyTorch/YOLO (the `run_live_3d.sh` script handles this):
```bash
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/targets/aarch64-linux/lib:$(find ~/.local/lib/python3.10/site-packages/nvidia -name "lib" -type d 2>/dev/null | tr "\n" ":"):$LD_LIBRARY_PATH
```

**Note:** After `colcon build`, you need to re-source `install/setup.bash` (or open a new terminal) for changes to take effect.

## Workspace
| Path | Description |
|---|---|
| `/home/orin/kart-brain` | Main ROS2 workspace (this repo) |
| `~/Desktop/kart-medulla` | ESP32 firmware (PlatformIO project) |
| `~/Desktop/KART_SW` | Old copy of kart_sw (can be deleted) |

## ZED Camera
- USB device: `2b03:f780 STEREOLABS ZED 2`
- Appears as `/dev/video0` + `/dev/video1` when connected
- **OpenCV webcam mode**: 1344x376 stereo (left+right side by side). With `stereo_crop=true`, left eye only = 672x376
- **ZED SDK mode** (`pyzed.sl`): Full HD + depth maps
- May need re-plugging after reboot
- Calibration file already downloaded for SN 21983349

## Live Perception Pipeline
```bash
# All-in-one script
~/kart-brain/run_live.sh

# Or manually:
ros2 run kart_perception image_source --ros-args \
  -p source:=/dev/video0 -p stereo_crop:=true -p publish_rate:=10.0 &
ros2 run kart_perception yolo_detector --ros-args \
  -p weights_path:=models/perception/yolo/best_adri.pt &
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  ros2 run rqt_image_view rqt_image_view /perception/yolo/annotated &
```

## ESP32 Firmware (kart-medulla)

```bash
# Build and flash (from Orin)
cd ~/Desktop/kart-medulla
~/.local/bin/pio run --target upload --environment esp32dev --upload-port /dev/ttyUSB0

# Test firmware variants (see flash_test.sh):
./flash_test.sh a|b|c|d|normal

# Serial monitor:
~/.local/bin/pio device monitor -b 115200 -p /dev/ttyUSB0
```

- **UART0** (USB, `/dev/ttyUSB0`): Binary protocol at **115200** baud — logs suppressed (`esp_log_level_set("*", ESP_LOG_NONE)`)
- **UART2 removed** — GPIO17/GPIO16 reserved for hall sensors on PCB
- `KB_Coms_micro` ROS node bridges `/dev/ttyUSB0` ↔ ROS2 Frame topics
- **Steering PID output is negated** (`-KM_PID_Calculate(...)`) — motor wiring is reversed
- **`g_steering_target_received` guard** — motor stays off until first steering command from Orin
- **`outputLimit`** on steering actuator: float 0.0-1.0 (was uint8_t — caused hardware damage, see error_log)
- See `architecture.md` for protocol details and message types

## TensorRT Export

**`trtexec` does NOT embed ultralytics metadata** (class names, task type, input size). Engines built with `trtexec` will show generic `class0..class998` names when loaded by ultralytics. The proper export is `model.export(format='engine')` from the ultralytics Python API, but this fails on the Orin with `CUBLAS_STATUS_ALLOC_FAILED` during `model.fuse()` (cuBLAS 12.9 pip vs system CUDA 12.6 conflict — see error_log entry "cuBLAS fails on Jetson").

**Workaround**: `yolo_detector_node.py` has `EXPECTED_CLASS_NAMES` that auto-overrides wrong/generic names at runtime. If you add/change classes, update both `EXPECTED_CLASS_NAMES` in the node and `dataset.yaml`.

**Export procedure** (on Orin):
```bash
cd ~/kart-brain
# 1. Export .pt → ONNX (CPU-safe, no CUBLAS needed)
python3 -c "from ultralytics import YOLO; YOLO('models/perception/yolo/<model>.pt').export(format='onnx', imgsz=640, device='cpu')"
# 2. Convert ONNX → TensorRT engine (uses trtexec, avoids PyTorch fuse)
/usr/src/tensorrt/bin/trtexec --onnx=models/perception/yolo/<model>.onnx --saveEngine=models/perception/yolo/<model>.engine --fp16
# 3. Clean up ONNX
rm models/perception/yolo/<model>.onnx
```

## Known Issues
1. **numpy must be <2** — cv2 was compiled against numpy 1.x, numpy 2 breaks it
2. **ZED camera "NOT DETECTED" after reboot**: The ZED ROS wrapper may fail even though `lsusb` shows the device. Fix with a software USB reset (no physical re-plug needed): `echo "0" | sudo -S bash -c "echo 0 > /sys/bus/usb/devices/2-3.2/authorized && sleep 1 && echo 1 > /sys/bus/usb/devices/2-3.2/authorized"`. The ZED is at USB path `2-3.2` (SuperSpeed 5 Gbps).
3. **AnyDesk display**: Requires Xorg config at `/etc/X11/xorg.conf.d/10-virtual-display.conf` with `Option "ConnectedMonitor" "DFP-0"` to force a framebuffer on the DisplayPort output. Without this, the NVIDIA driver sees DFP-0 and DFP-1 as "disconnected" (dummy plug via DP-to-HDMI adapter doesn't provide proper EDID), so Xorg has no screen and AnyDesk gets a black framebuffer.
4. **ZED SDK installer breaks pip permissions**: The installer runs pip as root, leaving `.dist-info` dirs with bad permissions. Fix: `sudo chmod -R a+rX /usr/local/lib/python3.10/dist-packages/`
5. **WiFi SSH dropouts**: WiFi power saving causes intermittent SSH disconnects. Fix applied: `iw dev wlP1p1s0 set power_save off`, persisted via NetworkManager dispatcher script at `/etc/NetworkManager/dispatcher.d/99-disable-wifi-powersave`. If SSH starts timing out, check with `iw dev wlP1p1s0 get power_save` and re-apply if needed. May also need physical access (AnyDesk/monitor) if WiFi is fully down.
6. **libcudss.so.0 for PyTorch**: The NVIDIA pip packages install CUDA libs under `~/.local/lib/python3.10/site-packages/nvidia/cu12/lib/`. This path is registered in `/etc/ld.so.conf.d/nvidia-pip.conf` and ldconfig'd. If torch fails to import with `libcudss.so.0: cannot open shared object file`, re-run: `echo "0" | sudo -S bash -c "echo /home/orin/.local/lib/python3.10/site-packages/nvidia/cu12/lib > /etc/ld.so.conf.d/nvidia-pip.conf && ldconfig"`
7. **Orin display is :1** — when launching GUI apps via SSH, set `export DISPLAY=:1` and `export XAUTHORITY=/run/user/1000/gdm/Xauthority`. The `run_live.sh` script already does this. `XAUTHORITY` is optional (X has `localuser:orin` access) but recommended.
8. **Port 9090 (dashboard)**: If dashboard fails with "address already in use", kill the stale process: `fuser -k 9090/tcp`

## Launching the Autonomous Pipeline

**Autostart (systemd):** The full stack launches automatically on boot via `kart-brain.service`.
Starts in manual mode; tap an autonomous mission in the dashboard to switch.

```bash
# Service management
sudo systemctl status kart-brain       # Check status
sudo systemctl stop kart-brain         # Stop
sudo systemctl restart kart-brain      # Restart
journalctl -u kart-brain -f            # View logs

# Service file lives at: tools/kart-brain.service (repo) → /etc/systemd/system/ (Orin)
# To update after editing: sudo cp ~/kart-brain/tools/kart-brain.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl restart kart-brain
```

**Manual launch** (if service is stopped):
```bash
cd ~/kart-brain && source install/setup.bash
ros2 launch kart_bringup launch.py

# This starts: ZED camera → YOLO perception → cone_follower → cmd_vel_bridge → KB_Coms_micro → dashboard

# To view YOLO detections with bounding boxes:
DISPLAY=:1 ros2 run rqt_image_view rqt_image_view /perception/yolo/annotated

# Dashboard + comms only (no perception):
ros2 launch kart_bringup dashboard.launch.py
```
