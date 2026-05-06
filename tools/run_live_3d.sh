#!/bin/bash
# Full 3D pipeline: ZED wrapper (RGB + depth) → YOLO → depth localizer → cone follower → HUD
# System CUDA libs MUST come before pip NVIDIA libs (pip cuBLAS 12.9 is
# incompatible with Jetson CUDA 12.6; system cuBLAS 12.6 works).
NVIDIA_LIBS=$(find ~/.local/lib/python3.10/site-packages/nvidia -name "lib" -type d 2>/dev/null | tr "\n" ":")
export LD_LIBRARY_PATH="/usr/local/cuda-12.6/targets/aarch64-linux/lib:${NVIDIA_LIBS}${LD_LIBRARY_PATH}"
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export PATH=/usr/local/cuda-12.6/bin:$PATH
cd ~/kart-brain
source /opt/ros/humble/setup.bash
source install/setup.bash

# Kill stale processes from previous runs
echo "Cleaning up previous instances..."
killall -q rqt_image_view component_container_isolated robot_state_publisher 2>/dev/null
pkill -f "yolo_detector|steering_hud|cone_depth_localizer|cone_follower_node|image_source" 2>/dev/null
sleep 1

cleanup() { echo "Shutting down..."; kill 0; wait; }
trap cleanup SIGINT SIGTERM

# ZED camera (publishes RGB + depth + camera_info)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 &
sleep 8  # Give ZED time to initialise

# YOLO detector
ros2 run kart_perception yolo_detector --ros-args \
  -p image_topic:=/zed/zed_node/rgb/image_rect_color \
  -p detections_topic:=/perception/cones_2d \
  -p debug_image_topic:=/perception/yolo/annotated \
  -p weights_path:=models/perception/yolo/ruben_yolov11n_2026_03.pt &

# Depth localizer (2D cones + depth → 3D cones)
ros2 run kart_perception cone_depth_localizer --ros-args \
  -p detections_topic:=/perception/cones_2d \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/rgb/camera_info \
  -p output_topic:=/perception/cones_3d &

# Cone follower (3D cones → steering) — neural net v2 controller
WEIGHTS_JSON=$(ros2 pkg prefix kart_control)/share/kart_control/config/neural_v2_weights.json
ros2 run kart_control cone_follower_node.py --ros-args \
  -p detections_topic:=/perception/cones_3d \
  -p cmd_vel_topic:=/kart/cmd_vel \
  -p controller_type:=neural_v2 \
  -p weights_json:="$WEIGHTS_JSON" &

# Steering HUD overlay
ros2 run kart_perception steering_hud --ros-args \
  -p annotated_topic:=/perception/yolo/annotated \
  -p cones_3d_topic:=/perception/cones_3d \
  -p cmd_vel_topic:=/kart/cmd_vel \
  -p camera_info_topic:=/zed/zed_node/rgb/camera_info \
  -p output_topic:=/perception/hud &

sleep 15
echo "Opening HUD viewer..."
ros2 run rqt_image_view rqt_image_view /perception/hud &

wait
