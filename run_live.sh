#!/bin/bash
# System CUDA libs MUST come before pip NVIDIA libs (pip cuBLAS 12.9 is
# incompatible with Jetson CUDA 12.6; system cuBLAS 12.6 works).
NVIDIA_LIBS=$(find ~/.local/lib/python3.10/site-packages/nvidia -name "lib" -type d 2>/dev/null | tr "\n" ":")
export LD_LIBRARY_PATH="/usr/local/cuda-12.6/targets/aarch64-linux/lib:${NVIDIA_LIBS}${LD_LIBRARY_PATH}"
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export PATH=/usr/local/cuda-12.6/bin:$PATH
cd ~/kart_brain
source /opt/ros/humble/setup.bash
source install/setup.bash

# Kill stale processes from previous runs
echo "Cleaning up previous instances..."
killall -q rqt_image_view rviz2 2>/dev/null
pkill -f "rqt_image_view|rviz2|yolo_detector|image_source" 2>/dev/null
sleep 1

cleanup() { echo "Shutting down..."; kill 0; wait; }
trap cleanup SIGINT SIGTERM

# Image source (ZED as webcam, left eye only)
ros2 run kart_perception image_source --ros-args   -p source:=/dev/video0   -p stereo_crop:=true   -p publish_rate:=10.0   -p image_topic:=/image_raw &

# YOLO detector
ros2 run kart_perception yolo_detector --ros-args   -p image_topic:=/image_raw   -p detections_topic:=/perception/cones_2d   -p debug_image_topic:=/perception/yolo/annotated   -p weights_path:=models/perception/yolo/nava_yolov11_2026_02.pt &

sleep 25
echo 'Opening viewer...'
ros2 run rqt_image_view rqt_image_view /perception/yolo/annotated &
wait
