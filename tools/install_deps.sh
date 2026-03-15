#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Humble workspace dependencies for this repo.
# Requires: Ubuntu 22.04 with ROS 2 Humble already installed.

sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-image-geometry \
  ros-humble-joy \
  ros-humble-message-filters \
  ros-humble-vision-msgs \
  ros-humble-zed-msgs

# Python dependencies for YOLO inference (via uv).
python3 -m pip install --upgrade pip
python3 -m pip install --upgrade uv

tmp_requirements="$(mktemp)"
uv pip compile pyproject.toml -o "${tmp_requirements}"
uv pip install --system -r "${tmp_requirements}"
rm -f "${tmp_requirements}"
