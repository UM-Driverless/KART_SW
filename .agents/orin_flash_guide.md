<!-- reference — read when relevant -->
# Orin NVMe Flash & Setup Guide

Complete guide for flashing the Jetson AGX Orin to boot from NVMe SSD with JetPack 6.2.2 (L4T R36.5).

## Overview

| Item | Value |
|---|---|
| Target | Jetson AGX Orin Developer Kit |
| JetPack | 6.2.2 (L4T R36.5, released 2026-02-06) |
| Target OS | Ubuntu 22.04 |
| CUDA | 12.6 (bundled with JetPack, not chosen separately) |
| cuDNN | 9.x (bundled) |
| TensorRT | 10.x (bundled) |
| All installed via | `sudo apt install nvidia-jetpack` (meta-package) |
| Flash host | y540 laptop (Ubuntu 24.04, x86_64) — `ssh y540` |
| Connection | USB-C from y540 → Orin flashing port (next to 40-pin GPIO header) |

## Prerequisites

### On the flash host (y540)

```bash
# Install flash dependencies
sudo apt-get install -y abootimg binfmt-support binutils cpio cpp \
  device-tree-compiler dosfstools lbzip2 libxml2-utils nfs-kernel-server \
  python3-yaml sshpass udev

# Download L4T R36.5 BSP and root filesystem
mkdir -p ~/jetson-flash && cd ~/jetson-flash
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/release/Jetson_Linux_r36.5.0_aarch64.tbz2
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/release/Tegra_Linux_Sample-Root-Filesystem_r36.5.0_aarch64.tbz2
```

### Extract and prepare

```bash
cd ~/jetson-flash
tar xf Jetson_Linux_r36.5.0_aarch64.tbz2
sudo tar xpf Tegra_Linux_Sample-Root-Filesystem_r36.5.0_aarch64.tbz2 -C Linux_for_Tegra/rootfs/
cd Linux_for_Tegra/
sudo ./tools/l4t_flash_prerequisites.sh
sudo ./apply_binaries.sh
```

## Put Orin in Recovery Mode

The Orin must be in Force Recovery Mode (RCM) for flashing.

### If Orin is powered on:
1. Press and hold the **middle button** (Force Recovery)
2. While holding it, press and release the **Reset button** (leftmost)
3. Release the Force Recovery button after ~2 seconds

### If Orin is powered off:
1. Press and hold the **Force Recovery button** (middle, labeled 2)
2. Power on: plug USB-C power into port 4 (above DC jack), or plug DC into jack 5
3. If the white LED (0) is not lit, press the **Power button** (1)
4. Release both buttons

### Verify recovery mode
On the y540:
```bash
lsusb | grep -i nvidia
# Should show: 0955:7023 (recovery mode)
# NOT: 0955:7020 (normal mode)
```

### USB-C cable placement
The USB-C cable from the y540 goes to the Orin's **flashing port** — the USB-C port **next to the 40-pin GPIO header**, NOT the USB-C power port above the DC jack.

## How the Boot Chain Works

The Orin has three storage devices involved in booting:

```
QSPI flash (on-chip)     →  First-stage bootloader (firmware)
eMMC (57 GB, soldered)    →  Second-stage bootloader + boot partition (GPT + boot records)
NVMe M.2 SSD (476 GB)    →  Full Ubuntu root filesystem (where the OS lives)
```

**Boot sequence:** QSPI → eMMC bootloader → NVMe root filesystem

- **QSPI**: Tiny on-chip flash. Holds the very first code the CPU runs at power-on. Always needed.
- **eMMC**: The 57 GB internal storage soldered to the board. After NVMe flash, it only holds a small bootloader — NOT a full OS. It cannot be removed.
- **NVMe**: The M.2 SSD you plugged into the board. This is where Ubuntu and all your software lives. All 476 GB available for the root filesystem.

The flash tool writes to **all three**: QSPI gets firmware, eMMC gets the bootloader, NVMe gets the OS. This is expected — the eMMC is NOT getting a full OS, just the boot chain entry point.

**To verify after boot:** `df -h /` should show `/dev/nvme0n1p1`, NOT `/dev/mmcblk0p1`.

## Flash to NVMe

```bash
cd ~/jetson-flash/Linux_for_Tegra

# Flash to NVMe SSD (entire root filesystem on NVMe)
sudo ./tools/kernel_flash/l4t_initrd_flash.sh \
  --external-device nvme0n1p1 \
  -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
  --showlogs \
  -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
  --network usb0 \
  jetson-agx-orin-devkit \
  nvme0n1p1
```

This takes ~10-20 minutes. The flash tool outputs "Flash is successful" when done. The Orin will reboot automatically.

### What the flash tool does (step by step)
1. **Step 1**: Generates flash packages (signs bootloader, compresses images)
2. **Step 2**: Boots the Orin into a temporary initrd Linux via USB
3. **Step 3**: Writes bootloader to eMMC, creates GPT partition tables
4. **Step 4**: Writes the full root filesystem to NVMe
5. **Step 5**: Writes QSPI firmware
6. **Reboots** the Orin into the freshly installed Ubuntu on NVMe

## After First Boot

**First boot timing (important):**
- First boot after flash takes ~5 minutes with no display signal while the system initializes (filesystem setup, SSH key generation, hardware config)
- If no signal after 5 minutes: power cycle (hold power button 5s, then press again)
- After power cycle, BIOS/UEFI appears in a few seconds, then Ubuntu takes ~2 minutes to boot
- There may be a brief period of no signal during boot — this is normal, wait for it

The Orin boots into Ubuntu 22.04 setup wizard (language, user, password). Complete it, then:

### Set up SSH access
On the Orin (via monitor/keyboard or AnyDesk):
```bash
sudo apt install -y openssh-server
```

Then from your Mac, copy your SSH key:
```bash
sshpass -p "<password>" ssh-copy-id -o StrictHostKeyChecking=accept-new orin@<new-ip>
```

Update `~/.ssh/config` on the Mac with the new IP if it changed.

### Install everything
Copy and run the setup script:
```bash
scp jetson-orin-setup.sh orin:/tmp/
ssh orin "bash /tmp/jetson-orin-setup.sh"
```

The script installs (in order):
1. **nvidia-jetpack** — CUDA 12.6, cuDNN 9.3, TensorRT 10.3
2. **ROS 2 Humble** — full desktop + vision_msgs + dev tools
3. **ZED SDK 4.2** — for ZED 2 stereo camera (L4T 36.4 build, compatible with 36.5)
4. **PyTorch 2.10** — from NVIDIA's Jetson AI Lab wheels (jp6/cu126)
5. **Python deps** — numpy <2, ultralytics 8.4.14, etc.
6. **kart_brain** — clone, build with colcon

### Verify
```bash
nvcc --version                    # CUDA
python3 -c "import torch; print(torch.cuda.is_available())"  # PyTorch GPU
ros2 --help                       # ROS 2
~/kart_brain/run_live.sh          # Live perception
```

## Post-Setup Checklist

- [ ] NVMe is root (`df -h /` shows nvme0n1p1)
- [ ] CUDA working (`nvcc --version`)
- [ ] PyTorch sees GPU
- [ ] ROS 2 Humble sourced in `.bashrc`
- [ ] ZED camera detected (`ls /dev/video*`)
- [ ] kart_brain built and perception pipeline runs
- [ ] SSH key access from Mac (`ssh orin`)
- [ ] AnyDesk working (needs DP dummy plug)
- [ ] Deploy key for GitHub push (already added to kart_brain repo as "Jetson Orin")

## Troubleshooting

### Flash fails with "No Jetson device found"
- Orin is not in recovery mode. Check `lsusb | grep 0955:7023`
- Wrong USB-C port. Use the one next to the 40-pin header, not the power port.

### Flash fails with USB errors
- Try a different USB-C cable (data-capable, not charge-only)
- Connect directly (no hub)

### Orin doesn't boot after flash
- Enter recovery mode and reflash
- If persistent, the NVMe may have issues — try flashing to eMMC first to verify hardware

### "nvidia-jetpack" install fails
- Ensure the NVIDIA apt repo is configured. If not: `sudo apt-get install -y nvidia-l4t-core` first, which adds the repo.

## Notes

- **JetPack 7** does not support AGX Orin as of Feb 2026. JetPack 7.2 (with Orin support) is expected Q2 2026.
- The eMMC still has a bootloader. The old OS on eMMC is overwritten by the flash tool's boot partition. If you need a fallback OS, you'd need to reflash to eMMC explicitly.
- DHCP IPs may change after reflash. Always verify with `hostname -I` on the Orin.
- The flash host (y540) only needs Ubuntu 24.04 or 22.04 x86_64. Ubuntu 24.04 is supported for flashing JetPack 6.2.2 but NOT for "host development" (cross-compilation). This doesn't matter for us — we develop directly on the Orin.
- Flash files on y540 are at `~/jetson-flash/` (~5 GB total). Can be deleted after successful flash to free space.

## Decision Log

### Why JetPack 6.2.2 and not JetPack 7? (2026-02-22)
JetPack 7.0/7.1 only supports Jetson Thor, NOT AGX Orin. Orin support comes in JetPack 7.2 (expected Q2 2026, already slipped from Q1). Our full stack (ROS 2 Humble, ZED SDK, PyTorch 2.5, YOLOv5) is confirmed compatible with JetPack 6.2.2. No reason to wait.

### Why command-line flash and not SDK Manager? (2026-02-22)
SDK Manager has issues on Ubuntu 24.04 (the y540's OS). The command-line L4T tools (`l4t_initrd_flash.sh`) are what SDK Manager uses under the hood anyway, and they work reliably on Ubuntu 24.04 for flashing. We can drive the entire process over SSH.

### Why ZED SDK 4.2 and not 5.2? (2026-02-22)
ZED SDK 5.2 does not provide a build for L4T 36.5 (JetPack 6.2.2). The download URL pattern `https://download.stereolabs.com/zedsdk/5.2/l4t36.5/jetsons` returns an HTML page instead of a binary. ZED SDK 4.2 for L4T 36.4 installs successfully on L4T 36.5 — the installer warns `Detected Tegra_L4T36.5, required exact Tegra_L4T36.4` but proceeds and works. When SDK 5.2 for L4T 36.5 becomes available, upgrade.

### Why PyTorch 2.10 from Jetson AI Lab? (2026-02-22)
Standard PyPI torch wheels don't support Jetson (aarch64 + CUDA). NVIDIA's Jetson AI Lab provides pre-built wheels at `https://pypi.jetson-ai-lab.dev/jp6/cu126`. The `torch==2.10.0` version installed is compatible with JetPack 6.2.2 / CUDA 12.6. Install with: `pip3 install --extra-index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 torch torchvision`
