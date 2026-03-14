import glob
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    steering_gain_arg = DeclareLaunchArgument(
        "steering_gain",
        default_value="0.5",
        description="Gain applied to lateral cone angle before sending to steering. "
        "Lower values reduce oversteering. Default 0.5 (was 1.0).",
    )
    steering_gain = LaunchConfiguration("steering_gain")

    use_zed_od_arg = DeclareLaunchArgument(
        "use_zed_od",
        default_value="false",
        description="Use ZED SDK built-in object detection (true) or custom YOLO node (false).",
    )
    use_zed_od = LaunchConfiguration("use_zed_od")

    # System CUDA libs must precede pip NVIDIA libs to avoid cuBLAS version mismatch
    # (pip installs cuBLAS 12.9 which is incompatible with Jetson's CUDA 12.6)
    cuda_sys = "/usr/local/cuda-12.6/targets/aarch64-linux/lib"
    pip_nvidia_dirs = glob.glob(
        os.path.expanduser("~/.local/lib/python3.10/site-packages/nvidia/*/lib")
    )
    ld_path = ":".join(
        [cuda_sys] + pip_nvidia_dirs + [os.environ.get("LD_LIBRARY_PATH", "")]
    )
    set_ld_path = SetEnvironmentVariable("LD_LIBRARY_PATH", ld_path)

    # Kill stale GUI processes from previous runs to prevent accumulation on Orin
    cleanup_gui = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "killall -q rviz2 rqt_image_view 2>/dev/null; "
            "pkill -f 'rviz2|rqt_image_view' 2>/dev/null; "
            "sleep 0.5; exit 0",
        ],
        name="cleanup_stale_gui",
        output="log",
    )

    bringup_share = get_package_share_directory("kart_bringup")
    perception_share = get_package_share_directory("kart_perception")
    zed_share = get_package_share_directory("zed_wrapper")

    zed_overrides = os.path.join(bringup_share, "config", "zed_overrides.yaml")
    zed_overrides_od = os.path.join(bringup_share, "config", "zed_overrides_od.yaml")

    # ZED camera with built-in object detection
    zed_camera_with_od = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_share, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": "zed2",
            "ros_params_override_path": zed_overrides_od,
        }.items(),
        condition=IfCondition(use_zed_od),
    )

    # ZED camera without object detection (custom YOLO path)
    zed_camera_no_od = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_share, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": "zed2",
            "ros_params_override_path": zed_overrides,
        }.items(),
        condition=UnlessCondition(use_zed_od),
    )

    # ZED SDK OD path: bridge node converts ObjectsStamped → Detection2D/3DArray
    perception_zed_od = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, "launch", "perception_zed_od.launch.py")
        ),
        condition=IfCondition(use_zed_od),
    )

    # Custom YOLO path: our yolo_detector + cone_depth_localizer
    perception_custom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, "launch", "perception_3d.launch.py")
        ),
        condition=UnlessCondition(use_zed_od),
    )

    cone_follower = Node(
        package="kart_sim",
        executable="cone_follower_node.py",
        name="cone_follower",
        output="screen",
        parameters=[
            {
                "controller_type": "geometric",
                "steering_gain": steering_gain,
            }
        ],
    )

    state_machine = Node(
        package="kart_bringup",
        executable="state_machine_node.py",
        name="state_machine",
        output="screen",
    )

    cmd_vel_bridge = Node(
        package="kart_bringup",
        executable="cmd_vel_bridge_node.py",
        name="cmd_vel_bridge",
        output="screen",
    )

    comms = Node(
        package="kb_coms_micro",
        executable="KB_Coms_micro",
        name="kb_coms_micro",
        output="screen",
    )

    dashboard = Node(
        package="kb_dashboard",
        executable="dashboard",
        name="kb_dashboard",
        parameters=[{"port": 8080}],
        output="screen",
    )

    steering_hud = Node(
        package="kart_perception",
        executable="steering_hud",
        name="steering_hud",
        output="screen",
    )

    return LaunchDescription(
        [
            steering_gain_arg,
            use_zed_od_arg,
            set_ld_path,
            cleanup_gui,
            zed_camera_with_od,
            zed_camera_no_od,
            perception_zed_od,
            perception_custom,
            steering_hud,
            cone_follower,
            state_machine,
            cmd_vel_bridge,
            comms,
            dashboard,
        ]
    )
