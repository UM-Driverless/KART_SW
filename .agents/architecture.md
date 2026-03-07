# System Architecture

## Workspace Structure

```
~/kart_brain/                          (colcon ROS 2 workspace)
├── AGENTS.md                       ← Agent entry point
├── .agents/                        ← This directory
├── src/
│   ├── kart_sim/                   (ament_cmake) Gazebo simulation
│   │   ├── worlds/fs_track.sdf    44-cone oval track
│   │   ├── models/kart/           Ackermann kart + RGBD camera
│   │   ├── models/cone_{blue,yellow,orange}/
│   │   ├── scripts/
│   │   │   ├── perfect_perception_node.py
│   │   │   └── cone_follower_node.py
│   │   └── launch/simulation.launch.py
│   │
│   ├── kart_perception/            (ament_python) Perception pipeline
│   │   ├── kart_perception/
│   │   │   ├── yolo_detector_node.py         YOLO 2D detection
│   │   │   ├── cone_depth_localizer_node.py  Depth → 3D projection
│   │   │   ├── cone_marker_viz_3d_node.py    RViz markers
│   │   │   ├── cone_marker_viz_node.py       2D markers (legacy)
│   │   │   └── image_source_node.py          File/video publisher
│   │   └── launch/
│   │       ├── perception_3d.launch.py       Full 3D pipeline
│   │       └── perception_test.launch.py     Offline testing
│   │
│   ├── kart_bringup/               (ament_cmake) Hardware launch files
│   │   ├── launch/
│   │   │   ├── autonomous.launch.py      Full pipeline (perception→control→comms→dashboard)
│   │   │   ├── dashboard.launch.py        Dashboard + comms (no commands sent to kart)
│   │   │   └── teleop_launch.py          Joystick teleop
│   │   ├── scripts/
│   │   │   └── cmd_vel_bridge_node.py    Twist → Frame msgs (100 Hz)
│   │   └── config/teleop_params.yaml
│   │
│   ├── kb_coms_micro/              (ament_cmake, C++) Serial bridge (ROS ↔ ESP32 UART)
│   ├── kb_interfaces/              (ament_cmake) Custom msg/srv (Frame.msg)
│   ├── kb_serial_driver_lib/       (ament_cmake, C++) Low-level serial driver
│   ├── kb_dashboard/               (ament_python) Web dashboard (port 8080)
│   ├── joy_to_cmd_vel/             (ament_cmake, C++) Joystick → Twist
│   └── ThirdParty/
│
├── models/perception/yolo/nava_yolov11_2026_02.pt  YOLO weights (YOLOv11, primary)
├── test_data/driverless_test_media/      Test images/videos
├── scripts/                              Workspace utility scripts
├── build/ install/ log/                  colcon output (gitignored)
└── pyproject.toml
```

## Node Graph

### Simulation Mode (kart_sim)

```
┌─────────────────────────────────────────────────────────┐
│  Gazebo Fortress (ign gazebo -s --headless-rendering)   │
│                                                         │
│  World: fs_track.sdf                                    │
│  ├── ground_plane                                       │
│  ├── sun (no shadows)                                   │
│  ├── kart (AckermannSteering + RGBD camera)             │
│  └── 44 cone models (static cylinders)                  │
│                                                         │
│  Publishes (Ignition topics):                           │
│    /kart/rgbd/image, /depth_image, /camera_info         │
│    /model/kart/odometry                                 │
│    /world/fs_track/clock                                │
│  Subscribes:                                            │
│    /kart/cmd_vel (Twist → AckermannSteering)            │
└────────────────────┬────────────────────────────────────┘
                     │ ros_gz_bridge
                     ▼
┌─────────────────────────────────────────────────────────┐
│  ROS 2 Topics                                           │
│                                                         │
│  /zed/zed_node/rgb/image_rect_color  (remapped)         │
│  /zed/zed_node/depth/depth_registered (remapped)        │
│  /zed/zed_node/rgb/camera_info       (remapped)         │
│  /model/kart/odometry                                   │
│  /clock                              (remapped)         │
│  /kart/cmd_vel                       (ROS→Gazebo)       │
└────────────────────┬────────────────────────────────────┘
                     │
        ┌────────────┴────────────┐
        ▼                         ▼
┌──────────────────┐   ┌──────────────────────┐
│ Perfect Percep.  │   │ YOLO Pipeline        │
│ (ground truth)   │   │ (camera-based)       │
│                  │   │                      │
│ Reads SDF cones  │   │ yolo_detector        │
│ + odom → 3D det  │   │ → cone_depth_local.  │
│                  │   │                      │
│ Publishes:       │   │ Publishes:           │
│ /perception/     │   │ /perception/         │
│   cones_3d       │   │   cones_3d           │
└────────┬─────────┘   └──────────┬───────────┘
         │    (one or the other)  │
         └────────────┬───────────┘
                      ▼
         ┌────────────────────────┐
         │  Cone Follower         │
         │                        │
         │  Subscribes:           │
         │    /perception/cones_3d│
         │  Publishes:            │
         │    /kart/cmd_vel       │
         │                        │
         │  Algorithm:            │
         │  1. Separate blue/     │
         │     yellow cones       │
         │  2. Find nearest pair  │
         │  3. Steer to midpoint  │
         │  4. Speed ∝ straightness│
         └────────────────────────┘
```

### Real Hardware Mode (kart_bringup)

> **All hardware runs on the Jetson Orin.** The ESP32, ZED camera, and actuators are physically connected to the Orin. Code is edited on the Mac, then pushed/copied to the Orin via git or scp. Never attempt to flash the ESP32, check USB devices, or run ROS hardware nodes from the Mac. Always `ssh orin` first.

```
ZED Camera → /zed/zed_node/rgb/image_rect_color
           → /zed/zed_node/depth/depth_registered
           → /zed/zed_node/rgb/camera_info

Perception pipeline (same nodes, same topics)
  → /perception/cones_3d

Controller (cone_follower or future planner)
  → /kart/cmd_vel (Twist)

cmd_vel_bridge_node.py (100 Hz)
  → /orin/steering, /orin/throttle, /orin/brake (Frame msgs)

KB_Coms_micro (C++ serial bridge)
  → UART0 (USB /dev/ttyUSB0) → ESP32

ESP32 (kart_medulla firmware)
  → steering motor (H-bridge), throttle DAC, brake DAC
  → AS5600 angle sensor (I2C) → steering feedback
  → publishes: /esp32/heartbeat, /esp32/steering, etc.
```

### ESP32 UART Routing

The ESP32 uses only UART0:

| UART | Pins | Connection | Purpose |
|------|------|------------|---------|
| UART0 | GPIO1 (TX), GPIO3 (RX) | USB to Orin (`/dev/ttyUSB0`) | **Binary protocol only** — framed messages between ESP32 and Orin |

**UART2 was removed** — GPIO17/GPIO16 are reserved for hall sensors on the PCB. All ESP-IDF logs are suppressed (`esp_log_level_set("*", ESP_LOG_NONE)`) because UART0 is shared with the binary protocol.

### ESP32 Protocol (km_coms)

Frame format: `| SOF (0xAA) | LEN | TYPE | PAYLOAD | CRC8 |`

- CRC8: poly 0x07 over LEN, TYPE, and PAYLOAD bytes
- Max frame size: 255 bytes
- UART0 @ **115200** baud (CP2102 USB bridge — max reliable flash/runtime baud for CP2102)
- Comms task: **20 Hz**, Control task: **10 Hz**, Heartbeat: **1 Hz**

**Steering encoding**: int16 big-endian, value = radians × 1000.
- Example: 0.25 rad → 250 → payload `[0x00, 0xFA]`
- Range: -32.768 to +32.767 rad (far exceeds physical limits)

**Throttle/brake encoding**: single byte, 0-255 = % of max effort (maps to 8-bit DAC).

Key message types (ESP32 → Orin):
- `ESP_HEARTBEAT` (0x08): 4-byte payload `[0xDE, 0xAD, 0xBE, 0xEF]`, 1 Hz
- `ESP_ACT_STEERING` (0x04): 2-byte int16 rad×1000 feedback, 100 Hz

Key message types (Orin → ESP32):
- `ORIN_TARG_THROTTLE` (0x20): 1-byte effort 0-255
- `ORIN_TARG_BRAKING` (0x21): 1-byte effort 0-255
- `ORIN_TARG_STEERING` (0x22): 2-byte int16 rad×1000 target
- `ORIN_COMPLETE` (0x27): 7 bytes (throttle, brake, steering×2, mission, state, shutdown)

## Message Types

| Topic | ROS 2 Type | Key Fields |
|---|---|---|
| `/perception/cones_2d` | `vision_msgs/Detection2DArray` | bbox center, class_id, score |
| `/perception/cones_3d` | `vision_msgs/Detection3DArray` | 3D position, class_id, score |
| `/perception/yolo/annotated` | `sensor_msgs/Image` | Camera feed with YOLO bounding boxes (view with rqt_image_view) |
| `/kart/cmd_vel` | `geometry_msgs/Twist` | linear.x (speed), angular.z (steering rad) |
| `/orin/steering` | `kb_interfaces/Frame` | Steering target frame (int16 BE, rad×1000) |
| `/orin/throttle` | `kb_interfaces/Frame` | Throttle target frame (u8, 0-255) |
| `/orin/brake` | `kb_interfaces/Frame` | Brake target frame (u8, 0-255) |
| `/esp32/steering` | `kb_interfaces/Frame` | Steering feedback from ESP32 |
| `/esp32/heartbeat` | `kb_interfaces/Frame` | ESP32 heartbeat |
| `/model/kart/odometry` | `nav_msgs/Odometry` | pose (position + orientation), twist |
| Camera topics | `sensor_msgs/Image` | RGB 640x360, Depth 32FC1 |
| `/zed/.../camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (fx, fy, cx, cy) |

## Cone Class IDs (String Constants)

These strings are used everywhere — in YOLO class names, Detection messages, and visualization:

| Class ID | Color | Role | YOLO class name |
|---|---|---|---|
| `blue_cone` | Blue (0.1, 0.3, 1.0) | Left track boundary | Same |
| `yellow_cone` | Yellow (1.0, 0.9, 0.1) | Right track boundary | Same |
| `orange_cone` | Orange (1.0, 0.5, 0.1) | Start/finish markers | Same |
| `large_orange_cone` | Dark orange (1.0, 0.3, 0.0) | Large start/finish | Same |

## Track Layout (fs_track.sdf)

Oval track centered at (0, 0) in world coordinates:
- **Right straight:** x=20, y from -10 to +10 (blue at x=18.5, yellow at x=21.5)
- **Left straight:** x=-20, y from +10 to -10 (blue at x=-18.5, yellow at x=-21.5)
- **Top curve:** semicircle center (0, 10), radius 18.5 (blue inner) / 21.5 (yellow outer)
- **Bottom curve:** semicircle center (0, -10), same radii
- **Start/finish:** 4 orange cones at y≈0 on right straight
- **Kart spawn:** (20, 0) facing +Y (yaw = π/2), drives counterclockwise

Track width: 3m. Cone spacing: ~5m on straights, ~8m on curves.
