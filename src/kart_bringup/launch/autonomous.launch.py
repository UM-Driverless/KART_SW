from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import glob
import os


def generate_launch_description():
    # System CUDA libs must precede pip NVIDIA libs to avoid cuBLAS version mismatch
    # (pip installs cuBLAS 12.9 which is incompatible with Jetson's CUDA 12.6)
    cuda_sys = "/usr/local/cuda-12.6/targets/aarch64-linux/lib"
    pip_nvidia_dirs = glob.glob(
        os.path.expanduser("~/.local/lib/python3.10/site-packages/nvidia/*/lib")
    )
    ld_path = ":".join([cuda_sys] + pip_nvidia_dirs + [os.environ.get("LD_LIBRARY_PATH", "")])
    set_ld_path = SetEnvironmentVariable("LD_LIBRARY_PATH", ld_path)
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
        parameters=[{"controller_type": "geometric"}],
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

    return LaunchDescription([
        set_ld_path,
        zed_camera,
        perception_launch,
        cone_follower,
        cmd_vel_bridge,
        comms,
        dashboard,
    ])
