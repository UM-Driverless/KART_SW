#!/bin/bash
# Stop all autonomous pipeline processes cleanly.
# Usage: ./tools/stop_autonomous.sh

echo "Stopping autonomous pipeline..."

# ROS2 nodes launched by launch.py
pkill -f 'yolo_detector|cone_follower|steering_hud|state_machine|cmd_vel_bridge|KB_Coms_micro|dashboard_node|cone_depth|cone_marker_viz' 2>/dev/null

# ZED camera
pkill -f 'zed_wrapper|component_container_isolated' 2>/dev/null
killall -q robot_state_publisher 2>/dev/null

# Visualization
killall -q rviz2 rqt_image_view 2>/dev/null

# Kill any ros2 launch that's still hanging
pkill -f 'ros2.launch' 2>/dev/null

# Free dashboard port
fuser -k 9090/tcp 2>/dev/null

# Clean up FastRTPS shared memory (prevents stale DDS state)
rm -rf /dev/shm/fastrtps_* 2>/dev/null

sleep 0.5

# Verify nothing is left
REMAINING=$(pgrep -af 'yolo_detector|cone_follower|steering_hud|state_machine|cmd_vel_bridge|KB_Coms_micro|dashboard_node|cone_depth|zed_wrapper|component_container_isolated' 2>/dev/null)
if [ -n "$REMAINING" ]; then
    echo "Warning: some processes still alive, sending SIGKILL..."
    pkill -9 -f 'yolo_detector|cone_follower|steering_hud|state_machine|cmd_vel_bridge|KB_Coms_micro|dashboard_node|cone_depth|zed_wrapper|component_container_isolated' 2>/dev/null
    sleep 0.3
fi

echo "All autonomous processes stopped."
