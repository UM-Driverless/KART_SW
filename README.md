# kart_brain

ROS workspace for the UM-Driverless kart software stack.

## Jetson Orin Access
- **SSH**: `ssh orin` (WiFi, DHCP — IP may change)
- **AnyDesk ID**: `721489674`

## Hardware

| Component | Details |
|---|---|
| **Chassis** | Tony Kart Extreme (late 90s, 30/32mm CrMo tubing) |
| **Computer** | NVIDIA Jetson AGX Orin (JetPack 6.2.2, CUDA 12.6, 62 GB RAM) |
| **Camera** | ZED 2 stereo (USB 3.0) |
| **Microcontroller** | ESP32 "Kart Medulla" (UART 115200 baud) |
| **Gamepad** | USB/Bluetooth (deadman switch on R1) |
| **Actuators** | Steering motor (H-bridge) + throttle (DAC) via ESP32 |

See [kart_docs](https://um-driverless.github.io/kart_docs/assembly/) for full hardware documentation, wiring diagrams, and setup guides.

## Install (ROS 2 Humble)
Assumes Ubuntu 22.04 with ROS 2 Humble already installed and sourced.

```bash
./scripts/install_deps.sh
```

## Build
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Launch Files

### Real Hardware (Orin)

| Launch file | Command | Description | Nodes |
|---|---|---|---|
| **autonomous.launch.py** | `ros2 launch kart_bringup autonomous.launch.py` | Full autonomous pipeline — camera, perception, control, comms, and dashboard | ZED camera, perception_3d (YOLO + depth localizer), steering_hud, cone_follower, cmd_vel_bridge, KB_Coms_micro, kb_dashboard |
| **teleop.launch.py** | `ros2 launch kart_bringup teleop.launch.py` | Manual driving with a joystick. Requires gamepad at `/dev/input/js0` | joy_node, joy_to_cmd_vel, cmd_vel_bridge, KB_Coms_micro |
| **dashboard.launch.py** | `ros2 launch kart_bringup dashboard.launch.py` | Comms + web dashboard only — safe for firmware testing, sends no commands to the kart | KB_Coms_micro, kb_dashboard |
| **gui.launch.py** | `ros2 launch kart_bringup gui.launch.py` | HUD viewer window on Orin display (launch separately from autonomous) | hud_viewer |

### Simulation (VM)

| Launch file | Command | Description | Nodes |
|---|---|---|---|
| **simulation.launch.py** | `ros2 launch kart_sim simulation.launch.py` | Full Gazebo simulation with dashboard | Gazebo, ros_gz_bridge, camera_info_fix, perfect_perception (or YOLO pipeline), cone_follower, cone_marker_viz_3d, ackermann_to_vel, esp32_sim, kb_dashboard |

```bash
# Example: autocross track, geometric controller, with Gazebo GUI
ros2 launch kart_sim simulation.launch.py gui:=true controller:=geometric track:=autocross

# Check all available arguments:
ros2 launch kart_sim simulation.launch.py --show-args
```

Arguments: `track:=oval|hairpin|autocross`, `use_yolo:=true|false`, `gui:=true|false`, `controller:=geometric|neural|neural_v2`, `weights_json:=<path>`

### Perception (Testing)

| Launch file | Command | Description | Nodes |
|---|---|---|---|
| **perception_test.launch.py** | `ros2 launch kart_perception perception_test.launch.py source:=<path> weights:=<path>` | 2D YOLO on recorded images/video — no camera needed | image_source, yolo_detector, cone_marker_viz, static_tf |
| **perception_3d.launch.py** | `ros2 launch kart_perception perception_3d.launch.py` | Live 3D perception — YOLO + depth localization. Used as sub-launch by autonomous.launch.py | yolo_detector, cone_depth_localizer, cone_marker_viz_3d |

### Standalone

| Launch file | Command | Description | Nodes |
|---|---|---|---|
| **kb_dashboard** | `ros2 launch kb_dashboard dashboard.launch.py` | Dashboard web UI only (no comms) | kb_dashboard |
| **display_zed_cam.launch.py** | `ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2` | ZED camera + RViz2 visualization | rviz2, zed_camera (optional) |

## FAQ

**Why ROS 2 and not ROS 1?**
ROS 1 (Noetic) reached end-of-life in May 2025. ROS 2 is the actively maintained version with better real-time support, native DDS communication, and lifecycle node management — all important for a safety-critical autonomous vehicle.

**Why Humble and not Jazzy?**
The Jetson Orin runs Ubuntu 22.04 (JetPack 6), and Humble is the LTS release that targets 22.04. Jazzy requires Ubuntu 24.04 which is not yet supported on the Jetson platform.

**Why C++ instead of Python?**
We use Python. The perception and control nodes are written in Python for faster prototyping and because PyTorch (used by YOLO) is a Python library. C++ nodes may be introduced later for performance-critical paths, but Python is the default.

**Why is the microcontroller code in a separate repo?**
The ESP32 firmware (`kart_medulla`) runs on bare metal with FreeRTOS — it has its own toolchain (PlatformIO/Arduino), its own flashing process, and no ROS dependency. Keeping it separate avoids coupling the embedded build with the ROS workspace.

**Why YOLOv5 and not a newer version?**
The trained weights (`best_adri.pt`) were produced with YOLOv5 on our custom cone dataset. YOLOv5's PyTorch Hub integration makes it simple to load custom weights, and the model runs well on Jetson via TensorRT export. Migrating to a newer YOLO version is tracked as a future task but not a priority — the current model detects cones reliably.

**What was the old Python repo (`driverless`) used for?**
That was the 2024 prototype: a monolithic Python script that handled ZED camera capture, YOLO inference, and CAN bus commands all in one process. It proved the concept but wasn't modular or maintainable. The current ROS 2 architecture splits those responsibilities into independent nodes.

**How do we get 3D cone positions from the camera?**
The ZED is a stereo camera — its SDK computes a depth map from the left/right image pair. We run YOLO on the 2D image to get bounding boxes, then look up the depth at each detection's center pixel to get the distance. With the depth value and the camera intrinsics, we back-project into 3D (x, y, z) in the camera frame. This is all handled by the `cone_depth_localizer` node. No custom stereo matching needed — the ZED SDK does that internally.

**Is image masking part of the YOLO model or done separately?**
Done separately. The YOLO model itself doesn't know which regions of the image to ignore (e.g. sky, dashboard, parts of the kart body). A mask is applied to the image before passing it to YOLO — this simplifies the input and avoids false detections in irrelevant areas. The mask is not baked into the trained weights.

**Why use a virtual machine (UTM) for development?**
The target hardware is a Jetson Orin Nano running Ubuntu 22.04 (aarch64). UTM lets us run the same OS and architecture on a Mac for development and testing without needing the physical board connected. SSH is configured so the VM is accessible as `ssh utm`.

**How do you run the system without the physical kart?**
The `perception_test.launch.py` file starts an `image_source` node that publishes images or video from local files onto the same ROS topics the ZED camera would use. This lets you test the full perception pipeline (YOLO detection, visualization, even 3D localization with recorded depth data) entirely offline.

## References
- Kart Docs: https://github.com/UM-Driverless/kart_docs
- Kart Docs site: https://um-driverless.github.io/kart_docs/
- ROS 2 installation: https://docs.ros.org/en/rolling/Installation.html
- ROS 2 Humble docs: https://docs.ros.org/en/humble/

## Test Media
Pulled from https://github.com/UM-Driverless/driverless and stored locally at
`test_data/driverless_test_media`.

## YOLO Weights
Pulled from https://github.com/UM-Driverless/driverless and stored locally at
`models/perception/yolo/best_adri.pt`.

## YOLO Quick Test
Run YOLO on a test image or video and save annotated output.

```bash
python3 scripts/run_yolo_on_media.py \
  --source test_data/driverless_test_media/cones_test.png \
  --weights models/perception/yolo/best_adri.pt \
  --output outputs/yolo
```

First run will download the YOLOv5 code via Torch Hub and cache it locally.

## Running ROS Nodes
Launch the image source and YOLO detector nodes on test media.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kart_perception
source install/setup.bash

ros2 launch kart_perception perception_test.launch.py \
  source:=test_data/driverless_test_media/cones_test.png \
  weights:=models/perception/yolo/best_adri.pt
```
