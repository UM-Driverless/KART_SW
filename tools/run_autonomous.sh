#\!/bin/bash
# Full autonomous pipeline: ZED → YOLO → cone follower → ESP32 steering
# Based on run_live_3d.sh + KB_Coms_micro + cmd_vel_bridge + dashboard
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
killall -q rqt_image_view component_container_isolated robot_state_publisher KB_Coms_micro 2>/dev/null
pkill -f "yolo_detector|steering_hud|cone_depth_localizer|cone_follower_node|image_source|cmd_vel_bridge|dashboard" 2>/dev/null
fuser -k 9090/tcp 2>/dev/null
sleep 1

# Software USB reset for ZED (avoids CAMERA NOT DETECTED after reboot)
echo "0" | sudo -S bash -c "echo 0 > /sys/bus/usb/devices/2-3.2/authorized && sleep 1 && echo 1 > /sys/bus/usb/devices/2-3.2/authorized" 2>/dev/null
sleep 2

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
  -p weights_path:=models/perception/yolo/nava_yolov11_2026_02.pt &

# Depth localizer (2D cones + depth → 3D cones)
ros2 run kart_perception cone_depth_localizer --ros-args \
  -p detections_topic:=/perception/cones_2d \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/rgb/camera_info \
  -p output_topic:=/perception/cones_3d &

# Cone follower (3D cones → steering)
ros2 run kart_control cone_follower_node.py --ros-args \
  -p detections_topic:=/perception/cones_3d \
  -p cmd_vel_topic:=/kart/cmd_vel \
  -p controller_type:=geometric &

# cmd_vel bridge (Twist → Frame msgs for ESP32)
ros2 run kart_control cmd_vel_bridge_node.py --ros-args \
  -r __node:=cmd_vel_bridge &

# KB_Coms_micro (serial bridge to ESP32)
ros2 run kb_coms_micro KB_Coms_micro --ros-args \
  -r __node:=kb_coms_micro &

# Dashboard (web UI on port 9090)
ros2 run kb_dashboard dashboard --ros-args \
  -r __node:=kb_dashboard -p port:=9090 &

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
