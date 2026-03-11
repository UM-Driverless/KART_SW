import glob
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
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

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            )
        ),
        launch_arguments={"camera_model": "zed2"}.items(),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kart_perception"),
                "launch",
                "perception_3d.launch.py",
            )
        )
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
            set_ld_path,
            cleanup_gui,
            zed_camera,
            perception_launch,
            steering_hud,
            cone_follower,
            cmd_vel_bridge,
            comms,
            dashboard,
        ]
    )
