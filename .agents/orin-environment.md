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
| Battery | 13S4P Molicel P42A (48 V nom; ~41.6 V empty → ~54.6 V full). **JBD/Xiaoxiang smart BMS readable over Bluetooth LE** — see "Battery BMS over BLE" below |
| Bluetooth | On-board combo Wi-Fi+BT radio (`hci0`). **Shared with Wi-Fi — see warning below** |
| CAN bus | `can0`, `can1` interfaces (unused, was for old ESP32 comms) |

## Access
```bash
ssh orin-local    # LAN — the kart Wi-Fi AP, Orin is 10.42.0.1. Join SSID `kart` (pwd `umotorsport`) first.
ssh orin-remote   # WAN — Cloudflare Tunnel (orin.rubenayla.xyz). Needs internet on the Orin: the USB
                  # tether, or a known Wi-Fi network that `wifi-watchdog` fell back to (see below).
# Dashboard: https://kart.rubenayla.xyz (password: "0", configurable via ROS param `password`)
#   Cloudflare Tunnel config: /etc/cloudflared/config.yml (system-level, needs sudo)
#   Routes kart.rubenayla.xyz → localhost:80 on Orin (was :9090; moved to :80 on 2026-07-08)
# There is NO "ssh orin" alias. Always use orin-local or orin-remote.
# Try orin-local first (faster), fall back to orin-remote if unreachable.
# AnyDesk for GUI (needs dummy HDMI plug)
# sudo password: 0
```

## Dashboard on port 80 — needs a sysctl (else it dies silently after reboot)
Since 2026-07-08 the dashboard binds **port 80** directly (default `port` param = 80 in `dashboard_node.py`), so URLs need no `:9090` suffix. Binding a port <1024 as the non-root `orin` user requires:

```
net.ipv4.ip_unprivileged_port_start=80
```

persisted in **`/etc/sysctl.d/99-kart-dashboard.conf`** (applied at boot by `systemd-sysctl`, before `kart-brain` starts). The Cloudflare tunnel (`/etc/cloudflared/config.yml`) routes `kart.rubenayla.xyz → localhost:80`.

**Failure mode to recognize:** if the sysctl is missing/reverts to 1024, the dashboard can't bind :80, the node exits, and nothing listens on 80 — yet `systemctl is-active kart-brain` still says `active` (the ROS launch as a whole stays up). The dashboard just vanishes. Verify the file with `sudo sysctl --system` (reproduces boot behaviour), not just `sysctl -w`. See `history.md` 2026-07-08.

## The Wi-Fi radio has one role at a time, chosen by `wifi-watchdog` (since 2026-07-30)
The single Wi-Fi radio `wlP1p1s0` (RTL8822CE) **cannot be an access point and a client at the same time**, and it **cannot scan at all while in AP mode**: with the `kart` AP running, `nmcli -f SSID,SIGNAL dev wifi list --rescan yes` returns only `kart` itself at signal 0, and a direct `sudo iw dev wlP1p1s0 scan` never returns (measured 2026-07-30; killed after 2 minutes). So the role has to be chosen blind, and `wifi-watchdog.service` chooses it:

| USB tether | Radio role | Dashboard address | Remote access |
|---|---|---|---|
| plugged in | AP, SSID `kart` | fixed `http://10.42.0.1/` | works (tunnel over the tether) |
| unplugged | client on a known network | whatever that network assigned | works (tunnel over Wi-Fi) |

Source of truth is the repo: **`tools/wifi-watchdog.sh`** and **`tools/wifi-watchdog.service`**, installed to `/usr/local/bin/` and `/etc/systemd/system/`. Progress is logged: `journalctl -t wifi-watchdog`.

How the handover works, and why it is written this way:
- **Releasing the radio is `nmcli connection down kart-ap`, never "join network X".** NetworkManager treats a manually deactivated connection as ineligible for autoconnect, so once the AP is down it picks the highest-priority known network that is actually in range by itself. `nmcli connection up kart-ap` clears that state and takes the radio back.
- **The AP is never dropped while a station is associated to it** (`iw dev wlP1p1s0 station dump`). Anyone on the `kart` network is presumably watching the dashboard.
- **One client attempt per tether-unplug event, never a repeating timer.** Because the radio cannot scan in AP mode, every attempt costs a blind window of up to 30 s with no `kart` network. If nothing joins, the AP is restored.
- **No client attempt at boot.** An Orin powered up without a tether keeps the AP, because in the paddock a dashboard at a fixed `10.42.0.1` matters more than remote access.
- Force one attempt without unplugging anything: `sudo touch /run/kart-wifi-try-client`.

**Interaction to know about: if a phone auto-joins the `kart` Wi-Fi, the fallback will not fire** — the station guard sees a client and correctly refuses to drop the AP. Ruben's iPhone has taken a DHCP lease on `kart` before (`DHCPACK ... 10.42.0.128 ... iPhone`), so for the fallback to work the phone must be tethered over USB only, not also joined to `kart` over Wi-Fi.

**Why "Ruben's iPhone" is a poor fallback target:** an iPhone cannot serve Personal Hotspot over Wi-Fi while it is itself a Wi-Fi client — single radio, same limitation as the Orin. With the phone joined to `kart`, its hotspot SSID is not on air at all; a scan on 2026-07-30 saw `eduroam`, `Robots_urjc` and nothing else. The lab network `Robots_urjc` is the reliable target and gives real internet (verified: `10.7.20.142/24`, ping 1.1.1.1 at 22–35 ms, HTTPS 200, `generate_204` → 204, so no captive portal).

**The old version of this script was actively harmful and is worth recognizing if it ever comes back.** It ran `nmcli device connect wlP1p1s0 || nmcli connection up "iPhone de JBA" || nmcli connection up "Robots_urjc"` every 5 s whenever the device was not `connected`. The first command always succeeds — it brings up `kart-ap`, which holds `autoconnect-priority 200` — so the `||` branches were unreachable dead code and there was no fallback. It also made the AP look like it had an unkillable autoconnect: any manual attempt to join a network was yanked back mid-association, because an association takes longer than one 5 s poll. A backup sits at `/usr/local/bin/wifi-watchdog.sh.bak-20260730`.

## ⚠️ Wi-Fi and Bluetooth share ONE combo radio
The Orin's on-board Wi-Fi and Bluetooth are the same chip. **Restarting the `bluetooth` service, `hciX reset`, or a BT adapter power-cycle drops the Wi-Fi too** — which kills the `kart` AP and any `orin-local` SSH going through it. Consequences:
- Don't `systemctl restart bluetooth` while connected via `orin-local`; you'll cut your own link.
- To clear a stuck BLE connection/bond, use **D-Bus** (`busctl call org.bluez /org/bluez/hci0 org.bluez.Adapter1 RemoveDevice o /org/bluez/hci0/dev_<MAC-with-underscores>`) or just **reboot** — not a bluetooth-service restart.
- `bluetoothctl` run non-interactively over SSH (no TTY) tends to hang; prefer `busctl`/`journalctl`/filesystem checks for BT status.

## Battery BMS over BLE
The pack's **JBD/Xiaoxiang smart BMS** advertises over BLE as **`SP22S003BP21S100A`** (address seen: `A5:C2:37:39:58:5D`, but scan by name — BLE MACs can rotate). The `kb_bms` ROS node (in `launch.py`, autostarts with `kart-brain`) reads it and publishes `sensor_msgs/BatteryState` on **`/battery/state`**; the dashboard's BATT gauge reads that. Independent of the ESP32 link.

Protocol (JBD, via `bleak`): GATT service `ff00`, **notify** `ff01`, **write** `ff02`. Write `DD A5 03 00 FF FD 77` for pack summary (voltage, SOC@byte19, current, temps), `DD A5 04 00 FF FC 77` for per-cell mV. Frames `DD <reg> <status> <len> <payload> <chk> <chk> 77`, big-endian. Full parser in `src/kb_bms/kb_bms/bms_node.py`; see `history.md` 2026-07-08. `bleak` is a pip `--user` install for the `orin` user (the service runs as `orin`). A connected BLE device stops advertising, so only one client at a time — the node reconnects forever.

**Battery tab blank / "NO CELL DATA" — two BlueZ behaviours, both handled in `kb_bms` since
2026-07-18.** Diagnosed live on the Orin; do not re-derive from the older stale-BlueZ theory below.

1. **`bleak` cannot see the pack at all.** Its BlueZ backend sets a discovery filter, so `bluetoothd`
   issues `MGMT_OP_START_SERVICE_DISCOVERY` instead of a plain discovery. On BlueZ 5.64 that path
   reports *nothing* when it has no UUIDs to match against, and this pack advertises only flags plus
   its name — no service UUIDs. Confirmed: `btmgmt find` and `bluetoothctl scan on` both saw it at
   RSSI -67 in the same minute `BleakScanner` returned zero devices, with and without filters, and
   with the rest of the stack stopped so nothing was competing for the adapter. So the radio, the
   pack and BlueZ are all fine — only the filtered D-Bus scan path is blind. `kb_bms` therefore
   drives discovery with `bluetoothctl`, never `BleakScanner`.
2. **BlueZ destroys the device object as soon as discovery stops.** A discovered-but-unpaired device
   is *temporary*: when the scan exits, the D-Bus object goes with it, so a connect a moment later
   fails with `Device with address ... was not found` against a cache that was just populated.
   Trusting alone does not keep it alive. `kb_bms` keeps `bluetoothctl scan on` running as a
   background process for the whole connect attempt and terminates it afterwards.

The node also `trust`s the pack automatically once the object exists. Trust persists across reboots,
so after the first success a cold start connects straight by MAC without needing the scan path.

**Verified end-to-end on 2026-07-18:** with the pack fully wiped from BlueZ (`untrust`, `disconnect`,
`remove`, until `bluetoothctl info` said "not available"), a plain `systemctl restart kart-brain`
reconnected on its own in ~36 s and `/battery/state` published 52.28 V at 97%. No human step, and
no `bluetooth` service restart — which matters, because the Wi-Fi and BT share one radio, so
restarting `bluetooth` would drop the `kart` AP (see the warning above).

**Do NOT "fix" this by restarting the `bluetooth` service, and treat the older advice below with
care:** `kb_bms`'s recovery used to run `bluetoothctl disconnect` + `remove`, and `remove` deletes
the very cache entry the connect-by-MAC path depends on. Combined with the blind `BleakScanner`
fallback, that made the node unable to ever recover on its own — the remove guaranteed the lockout
it was meant to clear. Any `remove` must now be followed by the repopulating scan described above.

## WiFi Networks (priority order)
| Priority | Connection Name | SSID | Password | Notes |
|---|---|---|---|---|
| 100 | Ruben's iPhone | Ruben\u2019s iPhone | 00000000 | Ruben's phone hotspot. SSID has curly apostrophe (U+2019) — use `nmcli dev wifi connect` or Python to pass correct UTF-8. |
| 50 | iPhone de JBA | iPhone de JBA | — | Jorge's phone hotspot. |
| 10 | Robots_urjc | Robots_urjc | — | Lab WiFi. |

## Addresses cheat-sheet (default operating mode)

| Address | What | When it works |
|---|---|---|
| `http://kart/` | Dashboard, memorable name | On the `kart` Wi-Fi (pwd `umotorsport`). Needs the dnsmasq+port-80 setup below — NOT applied yet (Orin was off) |
| `http://10.42.0.1` | Dashboard, bare IP | On the `kart` Wi-Fi. Always works, zero internet needed. **Now port 80** (no `:9090` suffix) |
| `ssh orin@10.42.0.1` (`orin-local`) | SSH | On the `kart` Wi-Fi |
| `http://172.20.10.2` | Dashboard from the USB-tethered iPhone itself | Phone plugged in via USB-C with Personal Hotspot on. **Now port 80** |
| `https://kart.rubenayla.xyz` | Dashboard from anywhere | While the Orin has internet: USB tether plugged, or `wifi-watchdog` fell the radio back to a known Wi-Fi network. **In that fallback state the `kart` AP is down, so `10.42.0.1` stops working** |
| `ssh orin-remote` | SSH from anywhere (Cloudflare) | Same condition as the row above |

Why these numbers: the Orin's hotspot uses NetworkManager's fixed shared-mode subnet (`10.42.0.x`, AP = `.1`); Apple hardcodes `172.20.10.1`/`.2` for every iPhone USB tether (phone = `.1`, first device = `.2`). The device that creates a network is always `.1`.

## USB tethering (iPhone → Orin) — verified working 2026-07-05
Plugging Ruben's iPhone into the Orin over USB (with Personal Hotspot on) creates an ethernet interface `enxfe9ca7a9ecdb` (name derives from the phone's MAC, so it can change if the phone presents a different MAC). NetworkManager auto-activates it as `Wired connection 2`, DHCP on the hotspot subnet 172.20.10.0/28: the iPhone is the router at 172.20.10.1, the Orin gets 172.20.10.2. Its default route has **metric 100, which beats Wi-Fi's 600** — so when tethered, all internet traffic (including the Cloudflare tunnel / `orin-remote` SSH) goes over USB automatically, no config needed. Verified by sudo-disconnecting Wi-Fi entirely: ping, DNS, and curl all worked over USB alone, and the remote SSH session survived.

**The USB link is a two-way IP network, not just an internet pipe.** While the iPhone provides internet, it can simultaneously reach the Orin directly at its link IP — dashboard from Safari on the tethering phone: `http://172.20.10.2/`. Verified working 2026-07-26 from the phone itself, after one false alarm: the phone's **home-screen shortcut** was the thing failing, not the network. If that URL ever fails again, re-check the saved shortcut's target before suspecting the network — the dashboard has been on **port 80** since 2026-07-08, so any shortcut still pointing at `:9090` (or at `https://`) will fail while a freshly typed `http://172.20.10.2/` works. Any `:9090` URL further down this file is historical.

Server-side facts confirmed while chasing that false alarm, useful as a baseline: `enxfe9ca7a9ecdb` holds 172.20.10.2/28, the phone at 172.20.10.1 pings in 0.6 ms, the dashboard listens on `0.0.0.0:80`, `curl http://172.20.10.2/` on the Orin returns 200, and `iptables` INPUT policy is ACCEPT with only the AP's `nm-sh-in-wlP1p1s0` chain. **`tcpdump` is not installed on the Orin** — to watch packets, use a raw `AF_PACKET` Python sniffer under sudo.

Gotcha: `nmcli device disconnect/connect` over SSH fails with "not authorized" (polkit) — needs sudo.

### Network architecture (decided 2026-07-05, IMPLEMENTED 2026-07-06)
Goal: dashboard must work even when the phone has no internet. Design:
- **Orin Wi-Fi (`wlP1p1s0`) becomes its own access point** (`nmcli device wifi hotspot ifname wlP1p1s0 ssid kart password <pwd>`, set autoconnect). Dashboard for everyone who joins: `http://10.42.0.1:9090` (NM hotspot default IP). Zero internet needed.
- **Ruben's iPhone: USB cable only, no Wi-Fi role.** Provides cellular internet to the Orin and browses the dashboard at `172.20.10.2:9090` over the same cable. (iPhones can't be Wi-Fi client + Wi-Fi hotspot at once — single radio — but that's irrelevant here since the phone doesn't need Wi-Fi.)
- Other phones join the kart AP; NM `shared` mode NATs the Orin's USB internet to them by default, so they even get internet when the tether is plugged in.
- Consequence: Orin no longer joins lab/phone Wi-Fi → `orin-local` means "join the kart AP, ssh 10.42.0.1". `orin-remote` works whenever the USB tether is up.

### Planned: dashboard at `http://kart/` (name instead of IP) — NOT YET APPLIED
mDNS (`orin.local`) was rejected as unreliable on Android. The reliable way: on the kart AP the Orin is every client's DHCP **and DNS** server (NM shared mode runs dnsmasq), so a plain DNS entry works on all devices. Two steps, both on the Orin:

1. **Name → IP.** NM's shared-mode dnsmasq reads `/etc/NetworkManager/dnsmasq-shared.d/`:
   ```bash
   echo 'address=/kart/10.42.0.1' | sudo tee /etc/NetworkManager/dnsmasq-shared.d/kart-name.conf
   sudo nmcli connection down kart-ap && sudo nmcli connection up kart-ap   # reload dnsmasq (drops AP clients ~5 s)
   ```
   (Verify first that the running dnsmasq uses `--conf-dir=/etc/NetworkManager/dnsmasq-shared.d` — check `ps aux | grep dnsmasq`. If not, the line can go in the NM dnsmasq config it does read.)
2. **Drop the :9090 — the dashboard binds port 80 directly** (design decision 2026-07-06: no hidden iptables redirect; the server listens where browsers look). The code defaults to port 80 (`dashboard_node.py` + all launch files). Linux blocks ports <1024 for non-root, so on every machine that runs the dashboard, lower the floor once:
   ```bash
   echo 'net.ipv4.ip_unprivileged_port_start=80' | sudo tee /etc/sysctl.d/99-unprivileged-port-80.conf
   sudo sysctl --system
   ```
   Also point the Cloudflare tunnel at the new port: in `/etc/cloudflared/config.yml` change `localhost:9090` → `localhost:80`, then `sudo systemctl restart cloudflared`.
   Symptom of a missing sysctl: dashboard node dies with `PermissionError: [Errno 13]` binding port 80. Override port per-run with the ROS `port` param if ever needed.

Result: anyone on the kart Wi-Fi opens **`http://kart/`** (type it with the slash or `http://`, otherwise the browser searches the word); bare-IP URLs need no port either. Caveats: the name only resolves for clients of the kart AP (not on the USB-tethered iPhone itself, whose DNS is cellular — it uses `http://172.20.10.2`; not via Cloudflare).

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

## JetPack / OS version — why we stay on JetPack 6 + Humble (checked 2026-07-15)
We run **JetPack 6.2.2 (Ubuntu 22.04, L4T R36.5)**, and Ubuntu 22.04 is why ROS 2 is **Humble** (Jazzy needs 24.04). That gap could be closed now, but one blocker keeps us here:

- **JetPack 7 exists and supports the AGX Orin.** JetPack 7 (Ubuntu 24.04, kernel 6.8, CUDA 13) originally shipped for Jetson Thor only. **JetPack 7.2 (announced 2026-06-01 at GTC Taipei, Jetson Linux 39.2, CUDA 13.2.1, TensorRT 10.16.2)** is the release that first brought the whole Orin family — AGX Orin / Orin NX / Orin Nano — into the JetPack 7 line. So our AGX Orin devkit *is* officially supported by 7.2. Moving to it would unlock ROS 2 Jazzy.
- **BLOCKER — ZED SDK has no JetPack 7.2 / Orin build yet.** Our perception depends on the ZED 2 (USB) camera via the ZED SDK. As of **2026-06-03**, Stereolabs staff (Myzhar) said on their forum that JP7.2 (Jetson Linux 39.2) support for Orin is only "in our roadmap" — no release, no date, and they flagged the 22.04→24.04 jump as a possible delay. The current JetPack-7 ZED SDK (5.1) targets **Jetson Thor, not Orin**. Flashing JP7.2 today would leave the camera with no working SDK → perception dead. The camera model (ZED 2, USB) is fine; the SDK-vs-JP7.2 gap is the problem.
- **Other components would survive the move.** YOLO `.engine` files are TensorRT-version-locked and would break, but we re-export those from the `.pt` anyway (routine). ESP32/AS5600/BMS-over-BLE/dashboard are pure Python + USB/serial/BLE, OS-version-agnostic.
- **A JP6→JP7 move is a full reflash + Humble→Jazzy port**, not `apt upgrade` — not a mid-season task. Humble is supported until **May 2027**, so there's no forced deadline.

**Decision: stay on JetPack 6.2.2 / Humble.** Re-check trigger: a released ZED SDK build for **JetPack 7.2 / Jetson Linux 39.2 on Orin** — watch the [Stereolabs thread](https://community.stereolabs.com/t/jetpack-7-2-orin-agx-support/11390) and the [ZED SDK releases page](https://github.com/stereolabs/zed-sdk/releases). When that lands, reconsider the migration.

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
