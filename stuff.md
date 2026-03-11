# Stuff

- Source selection: choose ZED vs media replay at launch, or add a mux later that republishes to `/image_raw`.
- Safety: watchdog that zeros `/kart/cmd_vel` if perception/control inputs are stale or a node dies.
- Health signals: heartbeat topics from critical nodes for a supervisor to monitor.
- Command interface: `/kart/cmd_vel` carries `geometry_msgs/Twist`. `cmd_vel_bridge` converts to protobuf Frames for the ESP32.
- Perception pipeline: ZED -> YOLO 2D detections -> 3D triangulation -> planner -> controller -> `/kart/cmd_vel`.
- UV on aarch64 lacks `uv sync --system`; use `uv pip compile pyproject.toml` then `uv pip install --system -r <requirements>` so ROS can import system Python deps.
- Python packaging prefers centralizing metadata in `pyproject.toml`, but ROS packages still require `package.xml` for build/test/runtime dependencies; `pyproject.toml` does not replace ROS metadata.
- Simulation: CARLA cannot run on arm64 — the server is Unreal Engine, x86_64 only, needs RTX 2070+ with 6GB dedicated VRAM. Same for FSDS (also Unreal/AirSim, and AirSim is deprecated by Microsoft). Gazebo Classic (gazebo11) has no arm64 binaries either. Ignition Gazebo Fortress has arm64 packages but EUFS Sim would need porting from Classic to Fortress. Realistic options: (1) split architecture — CARLA/Gazebo server on a desktop PC with GPU, ROS 2 stack on Jetson, connected over Ethernet; (2) build Gazebo Fortress + port EUFS Sim from source on Jetson (expect Tegra EGL rendering issues); (3) lightweight custom Python sim node that renders cones and publishes to ROS topics.
