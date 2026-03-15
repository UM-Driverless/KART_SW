#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="orin-flash"

# Build if image doesn't exist
if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
    echo "Building flash image (first time only, downloads ~5 GB)..."
    docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
fi

echo "Starting Orin NVMe flash..."
echo "Make sure the Orin is in Force Recovery Mode and USB-C is connected."
echo ""

docker run --rm --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    "$IMAGE_NAME"
