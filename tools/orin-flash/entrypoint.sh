#!/bin/bash
set -euo pipefail

WORKDIR="/jetson-flash/Linux_for_Tegra"
cd "$WORKDIR"

# Check if Orin is in recovery mode
if ! lsusb 2>/dev/null | grep -q "0955:7023"; then
    echo "ERROR: No Jetson device found in recovery mode."
    echo ""
    echo "Make sure:"
    echo "  1. USB-C cable is connected to the flashing port (next to 40-pin GPIO header)"
    echo "  2. Orin is in Force Recovery Mode:"
    echo "     - Hold the middle button (Force Recovery)"
    echo "     - Press and release Reset (leftmost button)"
    echo "     - Release Force Recovery after ~2 seconds"
    echo "  3. Verify on host: lsusb | grep '0955:7023'"
    exit 1
fi

echo "Jetson AGX Orin detected in recovery mode."
echo "Flashing to NVMe SSD (JetPack 6.2.2 / L4T R36.5)..."
echo ""

./tools/kernel_flash/l4t_initrd_flash.sh \
    --external-device nvme0n1p1 \
    -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
    --showlogs \
    -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
    --network usb0 \
    jetson-agx-orin-devkit \
    nvme0n1p1
