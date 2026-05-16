# Project Vision

## Purpose
Build the software stack that turns the UM-Driverless kart into an autonomous system:
perceive cones with a ZED camera, estimate 3D positions, plan/control steering and
acceleration, and send actuator targets over USB to an ESP32.

## High-Level System
- Compute: NVIDIA Jetson Orin DevKit (Ubuntu 22.04, ROS 2 Humble).
- Perception: ZED camera -> YOLO cone detection -> 3D cone positions.
- Planning/Control: start with simple control, evolve to a learned controller.
- Actuation: target steering/throttle sent via USB to an ESP32.

## Data Flow (initial)
1. ZED publishes RGB (and depth if needed).
2. YOLO detects cones in RGB frames.
3. 2D detections + depth -> 3D cone positions in kart frame.
4. Planner produces target steering + throttle.
5. USB node sends actuator targets to ESP32.

## Near-Term Milestones
- Bring up ZED + YOLO inference pipeline.
- 3D cone localization and frame transforms.
- Minimal controller (e.g., follow centerline of cone pairs).
- USB bridge to ESP32 with safety constraints.
- Add simulation or replay to validate nodes without hardware.

## Interfaces (expected)
- `zed` topics: images, depth, camera info.
- `perception` outputs: `cone_detections`, `cone_positions_3d`.
- `control` outputs: `ackermann_cmd` or `steering/throttle` targets.
- `esp32_bridge` consumes targets over USB.

## Assumptions
- ROS 2 Humble on Ubuntu 22.04.
- ZED SDK available on the Jetson.
- YOLO model and weights versioned outside this repo or via artifacts.

## References
- Kart Docs: https://um-driverless.github.io/kart-docs/
