# Gazebo Simulation Guide

## Overview

The simulation uses **Gazebo Fortress** (ign-gazebo 6.16.0, the Ignition-era naming) with `ros-humble-ros-gz` as the ROS 2 bridge. It can run headless or with GUI (GUI is slow — CPU rendering via llvmpipe, no GPU in the VM).

## Quick Start

```bash
# Full simulation with perfect perception (ground truth)
ros2 launch kart_sim simulation.launch.py

# With YOLO vision pipeline instead
ros2 launch kart_sim simulation.launch.py use_perception:=true use_perfect_perception:=false
```
> **Note:** ROS, the workspace, and `IGN_GAZEBO_RESOURCE_PATH` are all in `.bashrc`. No manual sourcing needed.

## Manual Step-by-Step Launch

Useful for debugging individual components:

```bash
# Terminal 1: Gazebo server
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix kart_sim)/share/kart_sim/models
WORLD=$(ros2 pkg prefix kart_sim)/share/kart_sim/worlds/fs_track.sdf
ign gazebo -s -r --headless-rendering "$WORLD"

# Terminal 2: Bridge
ros2 run ros_gz_bridge parameter_bridge \
  "/world/fs_track/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock" \
  "/model/kart/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
  "/kart/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist" \
  --ros-args -r /world/fs_track/clock:=/clock

# Terminal 3: Perfect perception
WORLD=$(ros2 pkg prefix kart_sim)/share/kart_sim/worlds/fs_track.sdf
ros2 run kart_sim perfect_perception_node.py --ros-args \
  -p world_sdf:="$WORLD" -p kart_start_x:=20.0 -p kart_start_y:=0.0 \
  -p kart_start_yaw:=1.5708 -p fov_deg:=120.0

# Terminal 4: Controller
ros2 run kart_sim cone_follower_node.py --ros-args \
  -p cmd_vel_topic:=/kart/cmd_vel -p max_speed:=2.0

# Terminal 5: Send manual velocity
ros2 topic pub --once /kart/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.0}}"
```

## Visualization Options

### RViz (via X11 forwarding)
```bash
ssh -X utm
rviz2
# Add: MarkerArray → /perception/cones_3d_markers
# Add: TF
# Add: Odometry → /model/kart/odometry
# Fixed Frame: odom
```

### Gazebo GUI (via X11 forwarding — slow)
```bash
ssh -X utm
ign gazebo -g   # Connects to running server
```

### Foxglove Studio (best experience)
```bash
# On VM: install and run rosbridge
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# On Mac: open Foxglove, connect to ws://192.168.65.2:9090
```

## Gazebo Fortress Specifics

### CLI is `ign`, not `gz`
```bash
ign gazebo --version        # NOT gz sim --version
ign topic -l                # list Ignition topics
ign topic -e -t /topic      # echo a topic
ign service -l              # list services
```

### Message types use `ignition.msgs.*`
```bash
# Bridge syntax: ROS_topic@ROS_type[ign_type  (Gazebo→ROS)
#                ROS_topic@ROS_type]ign_type  (ROS→Gazebo)
"/kart/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"
```

### No `<cone>` geometry
Gazebo Fortress (SDF 1.6) does not support `<cone>` geometry — it shows as invisible (geometry type 0). Use `<cylinder>` instead. Cones in the world are cylinders with colored materials.

### Rendering
- Uses OGRE2 by default with `--headless-rendering` (EGL offscreen)
- Works on ARM64 LLVMpipe (software rendering), verified
- If OGRE2 fails, try `<render_engine>ogre</render_engine>` in the Sensors plugin
- Camera at 640x360 @10Hz is the practical limit for CPU rendering

### Physics
- `real_time_factor: 1` — runs at wall-clock speed. Set to `0` for max speed (but the control loop may not keep up)
- `max_step_size: 0.004` — 250 Hz physics
- Ackermann plugin: `ignition-gazebo-ackermann-steering-system`

## Known Issues and Fixes

### 1. Odometry is relative to spawn, not world frame
**Problem:** `/model/kart/odometry` reports position (0,0) at startup regardless of where the kart is placed in the world SDF.
**Fix:** The `perfect_perception_node.py` has `kart_start_x`, `kart_start_y`, `kart_start_yaw` parameters that offset odom into world coordinates. These MUST match the `<pose>` in `fs_track.sdf`.

### 2. Kart drifts if sim runs before controller starts
**Problem:** With `real_time_factor: 0`, the sim sprints ahead during the seconds it takes ROS nodes to start. The kart can end up hundreds of meters off-track.
**Fix:** Either (a) start Gazebo paused (omit `-r` flag) and unpause after all nodes are up, or (b) use `real_time_factor: 1`. Current setup uses option (b).

### 3. Steering joints need effort limits
**Problem:** Without `<effort>` limits on steering revolute joints, Gazebo logs warnings: "Velocity control does not respect positional limits."
**Fix:** Add `<effort>1e6</effort>` to both `front_left_steering_joint` and `front_right_steering_joint` limits.

### 4. Kart wheels clip through ground
**Problem:** If the model's z-offset is too low, wheels start embedded in the ground plane, causing physics jitter and odometry drift.
**Fix:** The kart model has `<pose>0 0 0.22 0 0 0</pose>` — wheel bottoms at z ≈ 0.02 (just above ground). Adjust if wheel radius changes.

### 5. Cone follower stalls on curves
**Problem:** The simple midpoint-follower works well on straights but can lose sight of cones on curves (both sides not simultaneously visible).
**Status:** Known limitation. Needs either wider FOV (currently 120deg), more cone density on curves, or a smarter lookahead algorithm.

### 6. Neural controller oscillates and drives off-track
**Problem:** The `neural_v2` controller (trained in 2D sim) oscillates wildly in Gazebo — steering flips between -0.4 and +0.4 rad every 100ms. Within seconds the kart leaves the track, sees 0 cones, and defaults to constant steer=0.168 rad (9.6°).
**Root cause:** The 2D sim has different physics/timing than Gazebo. The neural network was never validated in Gazebo.
**Status:** Known. The control algorithm needs work — either retrain in Gazebo or use a different controller.

### 7. YOLO won't detect Gazebo cylinders
**Problem:** The YOLO model was trained on real cones. Gazebo renders solid-color cylinders which look nothing like real textured cones.
**Workaround:** Use `perfect_perception_node.py` (default). To use YOLO, either fine-tune on simulated images or add realistic cone textures/meshes to the models.

## Useful Debug Commands

```bash
# Check Gazebo is running
ign topic -l | grep clock

# See what the camera sees (raw topic stats)
ign topic -e -t /kart/rgbd/image -n 1 | head -5

# Check ROS bridge is working
ros2 topic hz /clock
ros2 topic hz /model/kart/odometry

# Check perception output
ros2 topic echo /perception/cones_3d --once | grep class_id

# Check controller output
ros2 topic echo /kart/cmd_vel --once

# Manual driving
ros2 topic pub /kart/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0}, angular: {z: 0.3}}" -r 10

# Pause/unpause simulation
ign service -s /world/fs_track/control \
  --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 --req 'pause: true'

# Kill everything (see "Clean Restart" section below)
```

## Clean Restart Procedure

**CRITICAL**: Always kill ALL processes before relaunching. Duplicate instances cause silent failures — odom stops reaching perception, ports stay occupied, the kart spawns at a stale position.

### Why this matters

Gazebo + ROS 2 launch files spawn ~10 processes (ign server, ign gui, parameter_bridge, perfect_perception, cone_follower, esp32_sim, dashboard, cone_marker_viz_3d, ackermann_to_vel, camera_info_fix). If any survive from a previous run:
- **Duplicate nodes** subscribe/publish on the same topics → messages get split between instances, data goes missing (e.g. odom never reaches perception → `got_odom=False`)
- **Port 8080 stays occupied** → dashboard crashes with `OSError: address already in use`
- **Stale Gazebo state** → kart inherits the old position (off-track) instead of respawning at (20,0)
- **`cone_marker_viz_3d` is especially sticky** — it survives `killall python3` because the process name in `ps` is `/usr/bin/python3 /path/to/cone_marker_viz_3d`

### Kill procedure (run ALL of these)

```bash
# Each pkill must be a separate command — chaining with ; causes SSH exit code issues
ssh utm "pkill -9 -f cone_marker_viz"
ssh utm "pkill -9 -f 'ros2 launch'"
ssh utm "pkill -9 -f 'ign gazebo'"
ssh utm "pkill -9 -f parameter_bridge"
ssh utm "pkill -9 -f perfect_perception"
ssh utm "pkill -9 -f cone_follower"
ssh utm "pkill -9 -f esp32_sim"
ssh utm "pkill -9 -f ackermann_to_vel"
ssh utm "pkill -9 -f camera_info_fix"
ssh utm "pkill -9 -f dashboard"
ssh utm "fuser -k 8080/tcp 2>/dev/null"
```

Wait 2-3 seconds, then verify:
```bash
ssh utm 'ps aux | grep -E "ign|kart" | grep -v grep | grep -v unattended | wc -l'
# Must return 0
```

### Launch (single instance)

```bash
# Headless (no GUI) — most reliable for automated testing
ssh utm 'nohup bash -c "source /opt/ros/humble/setup.bash && source ~/kart_brain/install/setup.bash && ros2 launch kart_sim simulation.launch.py" > /tmp/sim.log 2>&1 &'

# With GUI (visible in UTM window) — slow, CPU rendering
ssh utm 'DISPLAY=:0 nohup bash -c "source /opt/ros/humble/setup.bash && source ~/kart_brain/install/setup.bash && DISPLAY=:0 ros2 launch kart_sim simulation.launch.py gui:=true" > /tmp/sim_gui.log 2>&1 &'
```

### Launch file

`simulation.launch.py` in `kart_sim` — runs everything: Gazebo + bridge + perception + controller + esp32_sim + dashboard (port 8080).

### Verifying it works

```bash
# Check kart position and cone visibility
ssh utm 'grep "Published.*cones" /tmp/sim_gui.log | tail -3'
# Should show "Published N cones  kart=(20.0,Y.Y)" with N > 0

# Check dashboard is up
curl -s http://192.168.64.3:8080/ | head -1
# Should return HTML
```

## Kart Model Parameters

| Parameter | Value | Notes |
|---|---|---|
| Wheelbase | 1.05 m | Front-to-rear axle |
| Track width | 1.2 m | Left-to-right wheel |
| Wheel radius | 0.15 m | Collision = sphere |
| Chassis mass | 80 kg | |
| Camera position | Front, 0.55m forward, 0.25m up from chassis | |
| Camera FOV | 80 deg horizontal | |
| Camera resolution | 640 x 360 @ 10 Hz | RGBD |
| Max speed | 5 m/s | Ackermann plugin limit |
| Max steering | 0.5 rad (~29 deg) | |

## Installing on Jetson Orin

**Status:** Not yet installed. Blocked by root filesystem space (~5 GB free on eMMC).

Once root is migrated to NVMe, install with:
```bash
echo '0' | sudo -S apt install -y ros-humble-ros-gz
```

Everything else (worlds, models, launch files) is already in `src/kart_sim/`.
Set `IGN_GAZEBO_RESOURCE_PATH` and launch as documented above.

The Orin's Ampere GPU should handle Gazebo rendering natively (unlike the VM which uses llvmpipe), so higher resolutions and real-time factors >1 should be possible.
