# Orin NVMe Flash Tool

One-command flash for the Jetson AGX Orin to NVMe SSD with JetPack 6.2.2 (L4T R36.5).

Runs entirely in Docker — no host dependencies beyond Docker itself. Works on any x86_64 Linux (Ubuntu 24.04, Fedora, etc.).

## Prerequisites

- Docker installed on the host machine
- USB-C cable from host to Orin's **flashing port** (next to 40-pin GPIO header, NOT the power port)
- Orin in Force Recovery Mode

## Usage

```bash
./flash.sh
```

First run builds the Docker image (~7 GB, downloads L4T BSP + rootfs). Subsequent runs are instant.

## Putting the Orin in Recovery Mode

### If powered on:
1. Hold the **middle button** (Force Recovery)
2. Press and release **Reset** (leftmost button)
3. Release Force Recovery after ~2 seconds

### If powered off:
1. Hold **Force Recovery** (middle button)
2. Power on (plug USB-C power or DC jack)
3. Press **Power** button if white LED isn't lit
4. Release both buttons

### Verify:
```bash
lsusb | grep -i nvidia
# Should show: 0955:7023 (recovery mode)
```

## What it does

The flash tool writes to three storage devices on the Orin:

| Storage | What gets written |
|---|---|
| QSPI (on-chip) | First-stage bootloader firmware |
| eMMC (57 GB, soldered) | Second-stage bootloader + boot records |
| NVMe SSD (476 GB) | Full Ubuntu 22.04 root filesystem |

Boot chain: QSPI → eMMC bootloader → NVMe root filesystem.

Flash takes ~10-20 minutes. The Orin reboots automatically when done.

## After flashing

See [`.agents/orin_flash_guide.md`](../../.agents/orin_flash_guide.md) for post-flash setup (SSH, JetPack packages, ROS 2, ZED SDK, PyTorch).

Verify NVMe is root:
```bash
df -h /
# Should show /dev/nvme0n1p1
```
